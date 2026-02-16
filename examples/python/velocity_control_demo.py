#!/usr/bin/env python3
"""
Velocity control test for DEXI offboard manager.

Tests continuous velocity setpoints (set_velocity_body / stop_velocity commands).
Sequence:
  1. Takeoff to 2m
  2. Forward 0.3 m/s for 3s -> stop -> hold 2s
  3. Right 0.3 m/s for 2s -> stop -> hold 2s
  4. Backward 0.3 m/s for 3s -> stop -> hold 2s
  5. Yaw rate 30 deg/s for 3s -> stop -> hold 2s
  6. Arc: forward 0.2 m/s + yaw 20 deg/s for 3s -> stop -> hold 2s
  7. Land -> disarm -> stop heartbeat

Each velocity command is re-published at 10Hz during the duration for robustness
against BEST_EFFORT message drops.

Usage:
    python3 velocity_test.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from dexi_interfaces.msg import OffboardNavCommand
import time


class VelocityTest(Node):
    def __init__(self):
        super().__init__('velocity_test')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_publisher = self.create_publisher(
            OffboardNavCommand,
            '/dexi/offboard_manager',
            qos_profile
        )

        time.sleep(1)
        self.get_logger().info("Velocity test initialized")

    def send_command(self, command, distance_or_degrees=0.0, wait_time=2.0):
        """Send a simple command and wait."""
        msg = OffboardNavCommand()
        msg.command = command
        msg.distance_or_degrees = distance_or_degrees
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"Command: {command} ({distance_or_degrees})")
        time.sleep(wait_time)

    def send_velocity(self, vx, vy, vz, yaw_rate, duration):
        """Send velocity command repeatedly at 10Hz for the given duration."""
        msg = OffboardNavCommand()
        msg.command = 'set_velocity_body'
        msg.north = float(vx)
        msg.east = float(vy)
        msg.down = float(vz)
        msg.yaw = float(yaw_rate)

        self.get_logger().info(
            f"Velocity: vx={vx} vy={vy} vz={vz} yaw_rate={yaw_rate} for {duration}s"
        )

        interval = 0.1  # 10Hz
        elapsed = 0.0
        while elapsed < duration:
            self.cmd_publisher.publish(msg)
            time.sleep(interval)
            elapsed += interval

    def stop_velocity(self, hold_time=2.0):
        """Send stop_velocity and hold position."""
        msg = OffboardNavCommand()
        msg.command = 'stop_velocity'
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"Stop velocity -> hold {hold_time}s")
        time.sleep(hold_time)

    def execute(self):
        try:
            self.get_logger().info("=== Velocity Control Test ===")

            # Setup
            self.send_command("start_offboard_heartbeat", wait_time=3.0)
            self.send_command("arm", wait_time=3.0)
            self.send_command("offboard_takeoff", distance_or_degrees=2.0, wait_time=8.0)

            self.get_logger().info("--- Stabilize 3s ---")
            time.sleep(3.0)

            # Forward
            self.get_logger().info("--- Forward 0.3 m/s for 3s ---")
            self.send_velocity(0.3, 0.0, 0.0, 0.0, 3.0)
            self.stop_velocity()

            # Right
            self.get_logger().info("--- Right 0.3 m/s for 2s ---")
            self.send_velocity(0.0, 0.3, 0.0, 0.0, 2.0)
            self.stop_velocity()

            # Backward
            self.get_logger().info("--- Backward 0.3 m/s for 3s ---")
            self.send_velocity(-0.3, 0.0, 0.0, 0.0, 3.0)
            self.stop_velocity()

            # Yaw rate
            self.get_logger().info("--- Yaw 30 deg/s for 3s ---")
            self.send_velocity(0.0, 0.0, 0.0, 30.0, 3.0)
            self.stop_velocity()

            # Arc: forward + yaw
            self.get_logger().info("--- Arc: fwd 0.2 + yaw 20 deg/s for 3s ---")
            self.send_velocity(0.2, 0.0, 0.0, 20.0, 3.0)
            self.stop_velocity()

            # Land
            self.get_logger().info("--- Landing ---")
            self.send_command("land", wait_time=8.0)
            self.send_command("disarm", wait_time=2.0)
            self.send_command("stop_offboard_heartbeat", wait_time=1.0)

            self.get_logger().info("=== Test complete ===")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            self.get_logger().info("Emergency landing...")
            self.send_command("land")
            time.sleep(5)
            self.send_command("disarm")
            self.send_command("stop_offboard_heartbeat")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityTest()
    try:
        node.execute()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted - emergency landing")
        node.send_command("land")
        time.sleep(3)
        node.send_command("disarm")
        node.send_command("stop_offboard_heartbeat")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
