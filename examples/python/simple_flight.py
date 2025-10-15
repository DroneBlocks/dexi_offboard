#!/usr/bin/env python3
"""
Simple DEXI flight example: Arm, takeoff, and land

This example demonstrates the basic flight sequence using the DEXI offboard manager.
It shows how to:
1. Start offboard heartbeat
2. Arm the vehicle
3. Takeoff to a specified altitude (using offboard mode)
4. Hold position
5. Land
6. Disarm and stop heartbeat

Usage:
    python3 simple_flight.py

Requirements:
    - ROS2 environment sourced
    - dexi_offboard node running
    - dexi_interfaces package available

Note:
    This script uses 'offboard_takeoff' to keep the drone in offboard mode throughout
    the entire flight sequence, ensuring all commands work seamlessly without mode switching.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from dexi_interfaces.msg import OffboardNavCommand
import time


class SimpleFlight(Node):
    def __init__(self):
        super().__init__('simple_flight_example')

        # Configure QoS to match offboard manager
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publisher for offboard commands
        self.cmd_publisher = self.create_publisher(
            OffboardNavCommand,
            '/dexi/offboard_manager',
            qos_profile
        )

        # Wait for publisher to be ready
        time.sleep(1)
        self.get_logger().info("Simple flight example initialized")

    def send_command(self, command, distance_or_degrees=0.0, wait_time=2.0):
        """Send a command to the offboard manager and wait"""
        msg = OffboardNavCommand()
        msg.command = command
        msg.distance_or_degrees = distance_or_degrees

        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"Sent command: {command} (value: {distance_or_degrees})")

        # Wait for command to process
        time.sleep(wait_time)

    def execute_flight_sequence(self):
        """Execute the complete flight sequence"""
        try:
            self.get_logger().info("Starting flight sequence...")

            # 1. Start offboard heartbeat
            self.get_logger().info("Step 1: Starting offboard heartbeat")
            self.send_command("start_offboard_heartbeat", wait_time=3.0)

            # 2. Arm the vehicle
            self.get_logger().info("Step 2: Arming vehicle")
            self.send_command("arm", wait_time=3.0)

            # 3. Takeoff to 2 meters using offboard mode
            self.get_logger().info("Step 3: Taking off to 2 meters (offboard mode)")
            self.send_command("offboard_takeoff", distance_or_degrees=2.0, wait_time=8.0)

            # 4. Hold position for a moment
            self.get_logger().info("Step 4: Holding position for 5 seconds")
            time.sleep(5.0)

            # 5. Land
            self.get_logger().info("Step 5: Landing")
            self.send_command("land", wait_time=8.0)

            # 6. Disarm
            self.get_logger().info("Step 6: Disarming vehicle")
            self.send_command("disarm", wait_time=2.0)

            # 7. Stop offboard heartbeat
            self.get_logger().info("Step 7: Stopping offboard heartbeat")
            self.send_command("stop_offboard_heartbeat", wait_time=1.0)

            self.get_logger().info("Flight sequence completed successfully!")

        except Exception as e:
            self.get_logger().error(f"Flight sequence failed: {str(e)}")
            # Emergency stop - try to land and disarm
            self.get_logger().info("Attempting emergency landing...")
            self.send_command("land")
            time.sleep(5)
            self.send_command("disarm")
            self.send_command("stop_offboard_heartbeat")


def main(args=None):
    rclpy.init(args=args)

    flight_controller = SimpleFlight()

    try:
        # Execute the flight sequence
        flight_controller.execute_flight_sequence()

    except KeyboardInterrupt:
        flight_controller.get_logger().info("Flight interrupted by user")
        # Emergency stop
        flight_controller.send_command("land")
        time.sleep(3)
        flight_controller.send_command("disarm")
        flight_controller.send_command("stop_offboard_heartbeat")

    finally:
        flight_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()