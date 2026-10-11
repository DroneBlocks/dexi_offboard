#!/usr/bin/env python3
"""
Velocity control test for DEXI offboard manager.

Tests velocity commands (set_velocity_body / stop_velocity).
Sequence:
  1. Takeoff to 1.5m
  2. Forward 0.3 m/s for 3s -> stop -> hold 2s
  3. Right 0.3 m/s for 2s -> stop -> hold 2s
  4. Backward 0.3 m/s for 3s -> stop -> hold 2s
  5. Yaw rate 30 deg/s for 3s -> stop -> hold 2s
  6. Arc: forward 0.2 m/s + yaw 20 deg/s for 3s -> stop -> hold 2s
  7. Land -> disarm

Topic or service? This script uses both, on purpose:
    - Steps (arm, takeoff, stop, land, disarm) go to the /dexi/flight/command
      service, so each call waits until the step is done and stops the script
      if it fails.
    - The velocity is streamed on the /dexi/offboard_manager topic at 10 Hz. The
      manager turns a body-frame velocity into a world direction when it
      receives it, so while the drone turns the velocity has to keep coming.
    Use a service when you need to know it finished. Use a topic when you're
    streaming something many times a second, or just listening.

Usage:
    python3 velocity_control_demo.py
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
from dexi_interfaces.msg import OffboardNavCommand
from dexi_interfaces.srv import FlightCommand


class VelocityTest(Node):
    def __init__(self):
        super().__init__('velocity_test')
        self.flight_client = self.create_client(FlightCommand, '/dexi/flight/command')
        if not self.flight_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/dexi/flight/command is not available; is the offboard manager running?')
        self.flying = False

        # Velocity is a stream, so it goes on the manager's command topic
        self.velocity_publisher = self.create_publisher(
            OffboardNavCommand,
            '/dexi/offboard_manager',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )
        self.get_logger().info("Velocity test initialized")

    def flight(self, command, value=0.0, timeout=30.0, north=0.0, east=0.0, down=0.0, yaw=0.0, check=True):
        """Run one step on /dexi/flight/command and wait until it is done.

        Raises RuntimeError if the step fails, unless check=False (used while
        cleaning up, where a failure should not stop the rest of the cleanup).
        """
        request = FlightCommand.Request()
        request.command = command
        request.value = float(value)
        request.north = float(north)
        request.east = float(east)
        request.down = float(down)
        request.yaw = float(yaw)
        request.timeout = float(timeout)

        if command == 'offboard_takeoff':
            self.flying = True   # set before the call: a failed takeoff may still leave the ground
        future = self.flight_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result is None or not result.success:
            message = result.message if result is not None else 'no response'
            self.get_logger().error(f"{command}: FAILED ({message})")
            if check:
                raise RuntimeError(f"{command} failed: {message}")
            return False

        shown = f" {value:g}" if value else ""
        self.get_logger().info(f"{command}{shown}: {result.message} ({result.execution_time:.1f} s)")
        if command == 'land':
            self.flying = False
        return True

    def fly_velocity(self, forward, right, down, yaw_rate, duration):
        """Stream a body-frame velocity (m/s) and yaw rate (deg/s) for duration seconds."""
        self.get_logger().info(
            f"Velocity: forward={forward} right={right} down={down} yaw_rate={yaw_rate:.1f} for {duration:.1f}s"
        )
        msg = OffboardNavCommand()
        msg.command = 'set_velocity_body'
        msg.north = float(forward)
        msg.east = float(right)
        msg.down = float(down)
        msg.yaw = float(yaw_rate)

        interval = 0.1  # 10 Hz
        end_time = time.time() + duration
        while time.time() < end_time:
            self.velocity_publisher.publish(msg)
            time.sleep(interval)

    def stop_velocity(self, hold_time=2.0):
        """Stop and hold position for hold_time seconds."""
        self.flight("stop_velocity")
        self.get_logger().info(f"Stop -> hold {hold_time}s")
        time.sleep(hold_time)

    def emergency_land(self):
        """Land if the drone is in the air, then disarm. Never raises."""
        self.get_logger().warn("Emergency landing...")
        if self.flying:
            self.flight("land", timeout=60.0, check=False)
        self.flight("disarm", check=False)

    def execute(self):
        try:
            self.get_logger().info("=== Velocity Control Test ===")

            # Setup
            self.flight("arm")
            self.flight("offboard_takeoff", 1.5)

            # Forward
            self.get_logger().info("--- Forward 0.3 m/s for 3s ---")
            self.fly_velocity(0.3, 0.0, 0.0, 0.0, 3.0)
            self.stop_velocity()

            # Right
            self.get_logger().info("--- Right 0.3 m/s for 2s ---")
            self.fly_velocity(0.0, 0.3, 0.0, 0.0, 2.0)
            self.stop_velocity()

            # Backward
            self.get_logger().info("--- Backward 0.3 m/s for 3s ---")
            self.fly_velocity(-0.3, 0.0, 0.0, 0.0, 3.0)
            self.stop_velocity()

            # Yaw rate
            self.get_logger().info("--- Yaw 30 deg/s for 3s ---")
            self.fly_velocity(0.0, 0.0, 0.0, 30.0, 3.0)
            self.stop_velocity()

            # Arc: forward + yaw
            self.get_logger().info("--- Arc: fwd 0.2 + yaw 20 deg/s for 3s ---")
            self.fly_velocity(0.2, 0.0, 0.0, 20.0, 3.0)
            self.stop_velocity()

            # Land
            self.get_logger().info("--- Landing ---")
            self.flight("land", timeout=60.0)
            self.flight("disarm")

            self.get_logger().info("=== Test complete ===")

        except RuntimeError as e:
            self.get_logger().error(f"Error: {e}")
            self.emergency_land()


def main(args=None):
    # Ctrl+C raises KeyboardInterrupt instead of shutting ROS down, so the
    # emergency landing below can still call the service.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VelocityTest()

    try:
        node.execute()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
        node.emergency_land()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
