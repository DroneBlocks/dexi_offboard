#!/usr/bin/env python3
"""
DEXI square flight pattern example

This example demonstrates more complex flight patterns using the DEXI offboard manager.
The drone will:
1. Arm and takeoff
2. Fly a square pattern (forward, right, backward, left)
3. Return to start position
4. Land and disarm

Usage:
    python3 square_pattern.py

Requirements:
    - ROS2 environment sourced
    - dexi_offboard node running
    - dexi_interfaces package available
"""

import rclpy
from rclpy.node import Node
from dexi_interfaces.msg import OffboardNavCommand
import time


class SquarePatternFlight(Node):
    def __init__(self):
        super().__init__('square_pattern_flight')

        # Create publisher for offboard commands
        self.cmd_publisher = self.create_publisher(
            OffboardNavCommand,
            '/dexi/offboard_manager',
            10
        )

        # Flight parameters
        self.takeoff_altitude = 3.0  # meters
        self.square_size = 4.0       # meters per side
        self.move_wait_time = 5.0    # seconds to wait between moves

        # Wait for publisher to be ready
        time.sleep(1)
        self.get_logger().info("Square pattern flight example initialized")

    def send_command(self, command, distance_or_degrees=0.0, wait_time=2.0):
        """Send a command to the offboard manager and wait"""
        msg = OffboardNavCommand()
        msg.command = command
        msg.distance_or_degrees = distance_or_degrees

        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"Command: {command} (value: {distance_or_degrees})")

        # Wait for command to process
        time.sleep(wait_time)

    def execute_square_pattern(self):
        """Execute the complete square pattern flight"""
        try:
            self.get_logger().info("Starting square pattern flight...")

            # 1. Initialize offboard mode
            self.get_logger().info("Initializing offboard mode...")
            self.send_command("start_offboard_heartbeat", wait_time=3.0)
            self.send_command("arm", wait_time=3.0)

            # 2. Takeoff
            self.get_logger().info(f"Taking off to {self.takeoff_altitude} meters")
            self.send_command("takeoff", distance_or_degrees=self.takeoff_altitude, wait_time=8.0)

            # 3. Stabilize after takeoff
            self.get_logger().info("Stabilizing position...")
            time.sleep(3.0)

            # 4. Fly square pattern
            self.get_logger().info(f"Flying square pattern ({self.square_size}m sides)")

            # Side 1: Forward
            self.get_logger().info("Side 1: Flying forward")
            self.send_command("fly_forward", distance_or_degrees=self.square_size, wait_time=self.move_wait_time)

            # Side 2: Right
            self.get_logger().info("Side 2: Flying right")
            self.send_command("fly_right", distance_or_degrees=self.square_size, wait_time=self.move_wait_time)

            # Side 3: Backward
            self.get_logger().info("Side 3: Flying backward")
            self.send_command("fly_backward", distance_or_degrees=self.square_size, wait_time=self.move_wait_time)

            # Side 4: Left (return to start)
            self.get_logger().info("Side 4: Flying left (returning to start)")
            self.send_command("fly_left", distance_or_degrees=self.square_size, wait_time=self.move_wait_time)

            # 5. Optional: Add some rotations
            self.get_logger().info("Adding 360-degree rotation")
            self.send_command("yaw_right", distance_or_degrees=90.0, wait_time=3.0)
            self.send_command("yaw_right", distance_or_degrees=90.0, wait_time=3.0)
            self.send_command("yaw_right", distance_or_degrees=90.0, wait_time=3.0)
            self.send_command("yaw_right", distance_or_degrees=90.0, wait_time=3.0)

            # 6. Hold final position
            self.get_logger().info("Holding final position...")
            time.sleep(3.0)

            # 7. Land and shutdown
            self.get_logger().info("Landing...")
            self.send_command("land", wait_time=8.0)
            self.send_command("disarm", wait_time=2.0)
            self.send_command("stop_offboard_heartbeat", wait_time=1.0)

            self.get_logger().info("Square pattern flight completed successfully!")

        except Exception as e:
            self.get_logger().error(f"Square pattern flight failed: {str(e)}")
            self._emergency_land()

    def _emergency_land(self):
        """Emergency landing procedure"""
        self.get_logger().warn("Executing emergency landing procedure...")
        try:
            self.send_command("land", wait_time=2.0)
            time.sleep(5)
            self.send_command("disarm", wait_time=1.0)
            self.send_command("stop_offboard_heartbeat", wait_time=1.0)
        except Exception as e:
            self.get_logger().error(f"Emergency landing failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    flight_controller = SquarePatternFlight()

    try:
        # Execute the square pattern flight
        flight_controller.execute_square_pattern()

    except KeyboardInterrupt:
        flight_controller.get_logger().info("Flight interrupted by user")
        flight_controller._emergency_land()

    finally:
        flight_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()