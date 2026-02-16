#!/usr/bin/env python3
"""
Velocity-based figure-8 flight (1m radius per lobe).

Traces a figure-8 by flying two circles in opposite yaw directions:
  1. Circle right (positive yaw rate) — clockwise lobe
  2. Circle left (negative yaw rate) — counter-clockwise lobe

Each lobe: radius=1.0m, forward_speed=0.3m/s, duration≈20.9s

Usage:
    python3 velocity_figure_eight.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from dexi_interfaces.msg import OffboardNavCommand
import time
import math


RADIUS = 1.0          # meters
FORWARD_SPEED = 0.3   # m/s
YAW_RATE = math.degrees(FORWARD_SPEED / RADIUS)  # ~17.19 deg/s
LOBE_DURATION = (2.0 * math.pi * RADIUS) / FORWARD_SPEED  # ~20.9s per lobe


class VelocityFigureEight(Node):
    def __init__(self):
        super().__init__('velocity_figure_eight')

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
        self.get_logger().info(
            f"Velocity figure-8: radius={RADIUS}m, speed={FORWARD_SPEED}m/s, "
            f"yaw_rate={YAW_RATE:.1f}deg/s, lobe_duration={LOBE_DURATION:.1f}s"
        )

    def send_command(self, command, distance_or_degrees=0.0, wait_time=2.0):
        msg = OffboardNavCommand()
        msg.command = command
        msg.distance_or_degrees = distance_or_degrees
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"Command: {command} ({distance_or_degrees})")
        time.sleep(wait_time)

    def send_velocity(self, vx, vy, vz, yaw_rate, duration):
        msg = OffboardNavCommand()
        msg.command = 'set_velocity_body'
        msg.north = float(vx)
        msg.east = float(vy)
        msg.down = float(vz)
        msg.yaw = float(yaw_rate)

        interval = 0.1  # 10Hz
        elapsed = 0.0
        while elapsed < duration:
            self.cmd_publisher.publish(msg)
            time.sleep(interval)
            elapsed += interval

    def stop_velocity(self, hold_time=2.0):
        msg = OffboardNavCommand()
        msg.command = 'stop_velocity'
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"Stop -> hold {hold_time}s")
        time.sleep(hold_time)

    def execute(self):
        try:
            self.get_logger().info("=== Velocity Figure-8 (1m radius) ===")

            # Setup
            self.send_command("start_offboard_heartbeat", wait_time=3.0)
            self.send_command("arm", wait_time=3.0)
            self.send_command("offboard_takeoff", distance_or_degrees=2.0, wait_time=8.0)

            self.get_logger().info("Stabilizing 3s...")
            time.sleep(3.0)

            # Lobe 1: circle right (positive yaw)
            self.get_logger().info(f"Lobe 1: right circle for {LOBE_DURATION:.1f}s...")
            self.send_velocity(FORWARD_SPEED, 0.0, 0.0, YAW_RATE, LOBE_DURATION)

            # Lobe 2: circle left (negative yaw) — no stop between lobes for smooth transition
            self.get_logger().info(f"Lobe 2: left circle for {LOBE_DURATION:.1f}s...")
            self.send_velocity(FORWARD_SPEED, 0.0, 0.0, -YAW_RATE, LOBE_DURATION)

            self.stop_velocity(hold_time=3.0)

            # Land
            self.get_logger().info("Landing...")
            self.send_command("land", wait_time=8.0)
            self.send_command("disarm", wait_time=2.0)
            self.send_command("stop_offboard_heartbeat", wait_time=1.0)

            self.get_logger().info("=== Figure-8 complete ===")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            self.send_command("land")
            time.sleep(5)
            self.send_command("disarm")
            self.send_command("stop_offboard_heartbeat")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityFigureEight()
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
