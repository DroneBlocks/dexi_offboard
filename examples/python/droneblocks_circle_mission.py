#!/usr/bin/env python3
"""
DroneBlocks Circle Mission Example using the FlightCommand service

This example demonstrates how to fly in a smooth circle pattern:
1. Arm the drone
2. Take off to 1.5 meters
3. Fly in a circle with specified radius (smooth trajectory at 20Hz)
4. Land and disarm

The circle is flown by the offboard manager using continuous trajectory
setpoints for smooth circular motion (not discrete waypoints).

Steps go to the /dexi/flight/command service, so each call waits until the
step is done and stops the script if it fails.
    Use a service when you need to know it finished. Use a topic when you're
    streaming something many times a second, or just listening.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from dexi_interfaces.srv import FlightCommand

RADIUS = 1.0  # meters; change the circle size here


class DroneBlocksCircleMissionClient(Node):
    def __init__(self):
        super().__init__('droneblocks_circle_mission_client')
        self.flight_client = self.create_client(FlightCommand, '/dexi/flight/command')
        if not self.flight_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/dexi/flight/command is not available; is the offboard manager running?')
        self.flying = False
        self.get_logger().info('Service available! Starting mission...')

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

    def run_circle_mission(self, radius=RADIUS):
        """Execute complete circle mission"""
        try:
            # 1. Arm the drone
            self.flight('arm')

            # 2. Offboard takeoff to 1.5 meters (matches DroneBlocks GUI flow)
            self.flight('offboard_takeoff', 1.5)

            # 3. Fly in a circle. The call returns when the circle is done.
            # Duration scales with radius (circumference / 1.0 m/s target speed);
            # the timeout adds 50% margin.
            estimated_duration = (2.0 * math.pi * radius) / 1.0
            timeout = estimated_duration * 1.5
            self.get_logger().info(
                f'Circle radius {radius}m, estimated {estimated_duration:.1f}s, timeout {timeout:.1f}s'
            )
            self.flight('circle', radius, timeout=timeout)
            self.get_logger().info('Circle completed!')

            # 4. Land (returns on touchdown) and disarm
            self.flight('land', timeout=60.0)
            self.flight('disarm')

            self.get_logger().info('Circle mission completed successfully!')
            return True

        except RuntimeError as e:
            self.get_logger().error(f'Mission failed: {e}')
            self.emergency_land()
            return False


def main(args=None):
    # Ctrl+C raises KeyboardInterrupt instead of shutting ROS down, so the
    # emergency landing below can still call the service.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DroneBlocksCircleMissionClient()

    try:
        node.run_circle_mission()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
        node.emergency_land()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
