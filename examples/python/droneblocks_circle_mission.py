#!/usr/bin/env python3
"""
DroneBlocks Circle Mission Example using ExecuteBlocklyCommand Service

This example demonstrates how to fly in a smooth circle pattern:
1. Arm the drone
2. Takeoff to 3 meters
3. Switch to offboard mode
4. Fly in a circle with specified radius (smooth trajectory at 20Hz)
5. Land and disarm

The circle is flown using continuous trajectory setpoints for smooth,
professional-looking circular motion (not discrete waypoints).
Takes approximately 15 seconds to complete one full circle.
"""

import rclpy
from rclpy.node import Node
from dexi_interfaces.srv import ExecuteBlocklyCommand


class DroneBlocksCircleMissionClient(Node):
    def __init__(self):
        super().__init__('droneblocks_circle_mission_client')

        # Create service client
        self.client = self.create_client(
            ExecuteBlocklyCommand,
            '/dexi/execute_blockly_command'
        )

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for execute_blockly_command service...')

        self.get_logger().info('Service available! Starting circle mission...')

    def execute_command(self, command, parameter=0.0, timeout=60.0):
        """Execute a single command and wait for completion"""
        request = ExecuteBlocklyCommand.Request()
        request.command = command
        request.parameter = parameter
        request.timeout = timeout

        self.get_logger().info(f'Sending command: {command} (param: {parameter})')

        # Call service and wait for response
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response.success:
            self.get_logger().info(
                f'✓ {command} completed in {response.execution_time:.2f}s - {response.message}'
            )
        else:
            self.get_logger().error(
                f'✗ {command} FAILED: {response.message}'
            )

        return response.success

    def run_circle_mission(self, radius=2.0):
        """Execute complete circle mission"""
        import time

        # 1. Arm the drone
        if not self.execute_command('arm'):
            return False

        time.sleep(1.0)

        # 2. Use standard takeoff to get airborne (auto mode)
        if not self.execute_command('takeoff', parameter=3.0, timeout=30.0):
            return False

        # 3. Switch to offboard mode for precision flight
        if not self.execute_command('start_offboard_heartbeat'):
            return False

        time.sleep(2.0)  # Wait for offboard mode to be established

        # 4. Fly in a circle
        self.get_logger().info(f'Starting smooth circle flight with radius {radius}m...')

        # Circle duration scales with radius (circumference / 1.0 m/s target speed)
        # Add 50% margin for timeout: duration = 2*pi*radius / 1.0, timeout = duration * 1.5
        import math
        estimated_duration = (2.0 * math.pi * radius) / 1.0
        timeout = estimated_duration * 1.5

        self.get_logger().info(f'Estimated duration: {estimated_duration:.1f}s, timeout: {timeout:.1f}s')

        if not self.execute_command('circle', parameter=radius, timeout=timeout):
            return False

        self.get_logger().info('Circle completed!')

        # 5. Land (blocks until landing is complete)
        if not self.execute_command('land', timeout=30.0):
            return False

        # 6. Disarm the drone
        if not self.execute_command('disarm'):
            return False

        self.get_logger().info('Circle mission completed successfully!')
        return True


def main(args=None):
    rclpy.init(args=args)

    client = DroneBlocksCircleMissionClient()

    try:
        # You can change the radius here (in meters)
        # Default is 2.0 meters
        radius = 2.0

        success = client.run_circle_mission(radius=radius)
        if not success:
            client.get_logger().error('Mission failed!')
    except KeyboardInterrupt:
        client.get_logger().info('Mission interrupted by user')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
