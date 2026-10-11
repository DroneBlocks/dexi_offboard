#!/usr/bin/env python3
"""
Simple DEXI flight example: Arm, takeoff, and land

This example demonstrates the basic flight sequence using the DEXI offboard manager.
It shows how to:
1. Arm the vehicle
2. Take off to 1.5 meters (offboard mode)
3. Hold position
4. Land
5. Disarm

Steps go to the /dexi/flight/command service, so each call waits until the
step is done and stops the script if it fails.
    Use a service when you need to know it finished. Use a topic when you're
    streaming something many times a second, or just listening.

Usage:
    python3 simple_flight.py

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


class SimpleFlight(Node):
    def __init__(self):
        super().__init__('simple_flight_example')
        self.flight_client = self.create_client(FlightCommand, '/dexi/flight/command')
        if not self.flight_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/dexi/flight/command is not available; is the offboard manager running?')
        self.flying = False
        self.get_logger().info("Simple flight example initialized")

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

    def execute_flight_sequence(self):
        """Execute the complete flight sequence"""
        try:
            self.get_logger().info("Starting flight sequence...")

            # 1. Arm the vehicle (this also starts the offboard heartbeat)
            self.get_logger().info("Step 1: Arming vehicle")
            self.flight("arm")

            # 2. Take off to 1.5 meters in offboard mode; returns at that height
            self.get_logger().info("Step 2: Taking off to 1.5 meters (offboard mode)")
            self.flight("offboard_takeoff", 1.5)

            # 3. Hold position for a moment
            self.get_logger().info("Step 3: Holding position for 5 seconds")
            time.sleep(5.0)

            # 4. Land; returns on touchdown
            self.get_logger().info("Step 4: Landing")
            self.flight("land", timeout=60.0)

            # 5. Disarm
            self.get_logger().info("Step 5: Disarming vehicle")
            self.flight("disarm")

            self.get_logger().info("Flight sequence completed successfully!")

        except RuntimeError as e:
            self.get_logger().error(f"Flight sequence failed: {e}")
            self.emergency_land()


def main(args=None):
    # Ctrl+C raises KeyboardInterrupt instead of shutting ROS down, so the
    # emergency landing below can still call the service.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimpleFlight()

    try:
        node.execute_flight_sequence()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
        node.emergency_land()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
