#!/usr/bin/env python3
"""
DEXI square flight pattern example

This example demonstrates more complex flight patterns using the DEXI offboard manager.
The drone will:
1. Arm and take off using offboard mode
2. Fly a square pattern (forward, right, backward, left)
3. Perform a 360-degree rotation
4. Land and disarm

Steps go to the /dexi/flight/command service, so each call waits until the
step is done and stops the script if it fails.
    Use a service when you need to know it finished. Use a topic when you're
    streaming something many times a second, or just listening.

Usage:
    python3 position_square.py

Requirements:
    - ROS2 environment sourced
    - dexi_offboard node running
    - dexi_interfaces package available
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from dexi_interfaces.srv import FlightCommand


class SquarePatternFlight(Node):
    def __init__(self):
        super().__init__('square_pattern_flight')

        # Flight parameters
        self.takeoff_altitude = 1.5  # meters
        self.square_size = 1.0       # meters per side

        self.flight_client = self.create_client(FlightCommand, '/dexi/flight/command')
        if not self.flight_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/dexi/flight/command is not available; is the offboard manager running?')
        self.flying = False
        self.get_logger().info("Square pattern flight example initialized")

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

    def emergency_land(self):
        """Land if the drone is in the air, then disarm. Never raises."""
        self.get_logger().warn("Emergency landing...")
        if self.flying:
            self.flight("land", timeout=60.0, check=False)
        self.flight("disarm", check=False)

    def execute_square_pattern(self):
        """Execute the complete square pattern flight"""
        try:
            self.get_logger().info("Starting square pattern flight...")

            # 1. Arm and take off in offboard mode
            self.flight("arm")
            self.get_logger().info(f"Taking off to {self.takeoff_altitude} meters (offboard mode)")
            self.flight("offboard_takeoff", self.takeoff_altitude)

            # 2. Fly the square. Each side returns once the drone reaches its corner.
            self.get_logger().info(f"Flying square pattern ({self.square_size}m sides)")

            self.get_logger().info("Side 1: Flying forward")
            self.flight("fly_forward", self.square_size)

            self.get_logger().info("Side 2: Flying right")
            self.flight("fly_right", self.square_size)

            self.get_logger().info("Side 3: Flying backward")
            self.flight("fly_backward", self.square_size)

            self.get_logger().info("Side 4: Flying left (returning to start)")
            self.flight("fly_left", self.square_size)

            # 3. Turn a full circle in four quarter turns
            self.get_logger().info("Adding 360-degree rotation")
            for _ in range(4):
                self.flight("yaw_right", 90.0)

            # 4. Land and disarm
            self.get_logger().info("Landing...")
            self.flight("land", timeout=60.0)
            self.flight("disarm")

            self.get_logger().info("Square pattern flight completed successfully!")

        except RuntimeError as e:
            self.get_logger().error(f"Square pattern flight failed: {e}")
            self.emergency_land()


def main(args=None):
    # Ctrl+C raises KeyboardInterrupt instead of shutting ROS down, so the
    # emergency landing below can still call the service.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SquarePatternFlight()

    try:
        node.execute_square_pattern()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
        node.emergency_land()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
