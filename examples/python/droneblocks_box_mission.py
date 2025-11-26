#!/usr/bin/env python3
"""
DroneBlocks Box Mission Example using ExecuteBlocklyCommand Service

This example demonstrates how to chain commands together to:
1. Start offboard heartbeat
2. Arm the drone
3. Takeoff to 3 meters
4. Fly in a 1m x 1m box pattern
5. Land

Each command waits for completion before sending the next one.
"""

import rclpy
from rclpy.node import Node
from dexi_interfaces.srv import ExecuteBlocklyCommand


class DroneBlocksMissionClient(Node):
    def __init__(self):
        super().__init__('droneblocks_mission_client')

        # Create service client
        self.client = self.create_client(
            ExecuteBlocklyCommand,
            '/dexi/execute_blockly_command'
        )

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for execute_blockly_command service...')

        self.get_logger().info('Service available! Starting mission...')

    def execute_command(self, command, parameter=0.0, timeout=30.0):
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

    def run_box_mission(self):
        """Execute complete box mission"""
        import time

        # 1. Arm the drone
        if not self.execute_command('arm'):
            return False

        time.sleep(1.0)

        # 2. Use standard takeoff to get airborne (auto mode)
        # Now blocks until altitude is reached
        if not self.execute_command('takeoff', parameter=3.0, timeout=30.0):
            return False

        # 3. Now switch to offboard mode for precision flight
        if not self.execute_command('start_offboard_heartbeat'):
            return False

        time.sleep(2.0)  # Wait for offboard mode to be established

        # 4. Fly in a 1m x 1m box pattern
        self.get_logger().info('Starting box pattern (1m x 1m)...')

        # Forward 1m
        if not self.execute_command('fly_forward', parameter=1.0, timeout=20.0):
            return False

        # Right 1m
        if not self.execute_command('fly_right', parameter=1.0, timeout=20.0):
            return False

        # Backward 1m
        if not self.execute_command('fly_backward', parameter=1.0, timeout=20.0):
            return False

        # Left 1m (back to start position)
        if not self.execute_command('fly_left', parameter=1.0, timeout=20.0):
            return False

        self.get_logger().info('Box pattern completed!')

        # 5. Land (now blocks until landing is complete)
        if not self.execute_command('land', timeout=30.0):
            return False

        # 6. Disarm the drone
        if not self.execute_command('disarm'):
            return False

        # Note: land() already stops the offboard heartbeat

        self.get_logger().info('Mission completed successfully!')
        return True


def main(args=None):
    rclpy.init(args=args)

    client = DroneBlocksMissionClient()

    try:
        success = client.run_box_mission()
        if not success:
            client.get_logger().error('Mission failed!')
    except KeyboardInterrupt:
        client.get_logger().info('Mission interrupted by user')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
