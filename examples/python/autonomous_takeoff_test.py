#!/usr/bin/env python3
"""
Autonomous takeoff for flow-only drones (no GPS).

Bypasses MAV_CMD_NAV_TAKEOFF which is rejected when EKF has no horizontal
position source on the ground. Instead:

1. Start offboard heartbeat with velocity climb setpoints
2. Arm (PX4 accepts arm in offboard when setpoints are flowing)
3. Climb at 0.3 m/s via velocity setpoint (no position controller needed)
4. Monitor altitude — once at target, switch to position hold
5. Hold position for 10s
6. Land

This matches the PX4 community recommendation for flow-only indoor drones
(see PX4-Autopilot issues #24145, #22250, #21524).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)
import time
import math
import threading


class AutonomousTakeoffTest(Node):
    def __init__(self):
        super().__init__('autonomous_takeoff_test')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # Position tracking
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.heading = 0.0
        self.vz = 0.0
        self.xy_valid = False
        self.z_valid = False
        self.pos_received = False
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_cb, qos)

        # Setpoint state — controlled by the main thread, published by heartbeat
        self.setpoint_mode = 'velocity'  # 'velocity' or 'position'
        self.sp_vx = 0.0
        self.sp_vy = 0.0
        self.sp_vz = 0.0
        self.sp_x = 0.0
        self.sp_y = 0.0
        self.sp_z = 0.0
        self.heartbeat_running = False
        self.lock = threading.Lock()

        # Spin in background
        self.spin_thread = threading.Thread(target=lambda: rclpy.spin(self), daemon=True)
        self.spin_thread.start()
        time.sleep(1)

    def pos_cb(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.heading = msg.heading
        self.vz = msg.vz
        self.xy_valid = msg.xy_valid
        self.z_valid = msg.z_valid
        self.pos_received = True

    def get_timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_timestamp()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def start_heartbeat(self):
        self.heartbeat_running = True
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()

    def stop_heartbeat(self):
        self.heartbeat_running = False
        if hasattr(self, 'heartbeat_thread'):
            self.heartbeat_thread.join(timeout=2.0)

    def _heartbeat_loop(self):
        while self.heartbeat_running:
            with self.lock:
                mode = self.setpoint_mode
                vx, vy, vz = self.sp_vx, self.sp_vy, self.sp_vz
                px, py, pz = self.sp_x, self.sp_y, self.sp_z

            # Offboard control mode
            mode_msg = OffboardControlMode()
            mode_msg.timestamp = self.get_timestamp()
            if mode == 'velocity':
                mode_msg.position = False
                mode_msg.velocity = True
            else:
                mode_msg.position = True
                mode_msg.velocity = False
            mode_msg.acceleration = False
            mode_msg.attitude = False
            mode_msg.body_rate = False
            self.offboard_mode_pub.publish(mode_msg)

            # Trajectory setpoint
            sp_msg = TrajectorySetpoint()
            sp_msg.timestamp = self.get_timestamp()
            if mode == 'velocity':
                sp_msg.position = [float('nan'), float('nan'), float('nan')]
                sp_msg.velocity = [vx, vy, vz]
            else:
                sp_msg.position = [px, py, pz]
                sp_msg.velocity = [float('nan'), float('nan'), float('nan')]
            sp_msg.yaw = float('nan')
            sp_msg.acceleration = [float('nan'), float('nan'), float('nan')]
            self.setpoint_pub.publish(sp_msg)

            time.sleep(0.05)  # 20Hz

    def set_velocity(self, vx, vy, vz):
        with self.lock:
            self.setpoint_mode = 'velocity'
            self.sp_vx = vx
            self.sp_vy = vy
            self.sp_vz = vz

    def set_position(self, x, y, z):
        with self.lock:
            self.setpoint_mode = 'position'
            self.sp_x = x
            self.sp_y = y
            self.sp_z = z

    def run(self, target_altitude=1.0, climb_speed=0.3, hold_time=10.0):
        self.get_logger().info('=== Autonomous Takeoff Test (Flow-Only) ===')

        # Wait for position data
        self.get_logger().info('Waiting for position data...')
        t0 = time.time()
        while not self.pos_received:
            if time.time() - t0 > 5.0:
                self.get_logger().error('No position data — aborting')
                return
            time.sleep(0.1)

        ground_z = self.z
        target_z = ground_z - target_altitude  # NED: up is negative
        self.get_logger().info(
            f'Ground: z={ground_z:.2f}, Target: z={target_z:.2f} '
            f'(climb {target_altitude:.1f}m)')
        self.get_logger().info(
            f'xy_valid={self.xy_valid}, z_valid={self.z_valid}')

        # Step 1: Start heartbeat with gentle climb velocity
        # NED: negative vz = up
        self.set_velocity(0.0, 0.0, -climb_speed)
        self.start_heartbeat()
        self.get_logger().info(
            f'Heartbeat started — velocity climb at {climb_speed} m/s')
        self.get_logger().info('Waiting 2s for PX4 to see setpoints...')
        time.sleep(2.0)

        # Step 2: Switch to offboard mode
        self.send_command(176, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode requested')
        time.sleep(0.5)

        # Step 3: Arm
        self.send_command(400, param1=1.0)
        self.get_logger().info('Arm sent — drone should start climbing')

        # Step 4: Monitor altitude, wait until target reached
        self.get_logger().info(
            f'Climbing to {target_altitude:.1f}m (target_z={target_z:.2f})...')
        climb_start = time.time()
        climb_timeout = 15.0  # seconds

        while time.time() - climb_start < climb_timeout:
            altitude = -(self.z - ground_z)  # positive up
            if altitude >= target_altitude * 0.9:  # within 90% of target
                self.get_logger().info(
                    f'Target altitude reached! alt={altitude:.2f}m '
                    f'z={self.z:.2f} xy_valid={self.xy_valid}')
                break
            if int((time.time() - climb_start) * 2) % 2 == 0:
                # Log every ~1s
                pass
            time.sleep(0.1)
        else:
            self.get_logger().warn(
                f'Climb timeout! alt={-(self.z - ground_z):.2f}m, '
                f'expected {target_altitude:.1f}m')

        # Step 5: Transition to position hold
        # Capture current position as hold target
        hold_x = self.x
        hold_y = self.y
        hold_z = self.z
        self.set_position(hold_x, hold_y, hold_z)
        self.get_logger().info(
            f'Switched to position hold at ({hold_x:.2f}, {hold_y:.2f}, {hold_z:.2f}) '
            f'heading={math.degrees(self.heading):.1f}°')

        # Step 6: Hold
        self.get_logger().info(f'Holding position for {hold_time:.0f}s...')
        time.sleep(hold_time)
        drift_x = self.x - hold_x
        drift_y = self.y - hold_y
        drift_z = self.z - hold_z
        drift_total = math.sqrt(drift_x**2 + drift_y**2)
        self.get_logger().info(
            f'After hold: ({self.x:.2f}, {self.y:.2f}, {self.z:.2f}) '
            f'drift: xy={drift_total:.2f}m z={drift_z:.2f}m '
            f'heading={math.degrees(self.heading):.1f}°')

        # Step 7: Land
        self.get_logger().info('Landing...')
        self.stop_heartbeat()
        self.send_command(21)  # MAV_CMD_NAV_LAND
        time.sleep(8.0)

        # Step 8: Disarm
        self.send_command(400, param1=0.0)
        self.get_logger().info('Disarmed')
        self.get_logger().info('=== Test complete ===')


def main():
    rclpy.init()
    node = AutonomousTakeoffTest()
    try:
        node.run(target_altitude=1.0, climb_speed=0.3, hold_time=10.0)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — stopping + landing')
        node.set_velocity(0.0, 0.0, 0.0)  # Stop climb
        time.sleep(0.5)
        node.stop_heartbeat()
        node.send_command(21)  # Land
        time.sleep(3.0)
        node.send_command(400, param1=0.0)  # Disarm
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
