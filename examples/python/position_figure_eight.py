#!/usr/bin/env python3
"""
DEXI figure 8 flight pattern example

This example demonstrates a figure 8 (lemniscate) flight pattern with the drone's
nose following the flight path. The drone will:
1. Arm and takeoff using offboard mode
2. Fly a smooth figure 8 pattern with heading tangent to the path
3. Land and disarm

Note:
    Uses 'offboard_takeoff' to maintain offboard mode throughout the flight,
    allowing direct trajectory setpoint control for the smooth figure 8 pattern.

The figure 8 is generated using parametric equations:
    x(t) = A * sin(t)
    y(t) = (A/2) * sin(2*t)
where A is the size parameter and t goes from 0 to 2π

Usage:
    python3 figure_eight_pattern.py

Requirements:
    - ROS2 environment sourced
    - dexi_offboard node running
    - px4_msgs and dexi_interfaces packages available
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from dexi_interfaces.msg import OffboardNavCommand
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleLocalPosition
import time
import math


class FigureEightFlight(Node):
    def __init__(self):
        super().__init__('figure_eight_flight')

        # Flight parameters
        self.takeoff_altitude = 3.0  # meters
        self.figure8_size = 3.0      # meters (amplitude of the pattern)
        self.flight_speed = 0.5      # m/s (speed along the path)
        self.update_rate = 20.0      # Hz (trajectory update rate)

        # State variables
        self.current_position = None
        self.start_position = None
        self.pattern_started = False

        # Configure QoS profile for PX4 topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.cmd_publisher = self.create_publisher(
            OffboardNavCommand,
            '/dexi/offboard_manager',
            qos_profile
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )

        # Create subscriber for vehicle position
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile
        )

        # Timer for trajectory updates
        self.trajectory_timer = None
        self.t = 0.0  # Parameter for figure 8 trajectory

        # Wait for setup
        time.sleep(1)
        self.get_logger().info("Figure 8 flight example initialized")
        self.get_logger().info(f"Pattern size: {self.figure8_size}m, Speed: {self.flight_speed}m/s")

    def position_callback(self, msg):
        """Update current position from vehicle"""
        self.current_position = {
            'x': msg.x,
            'y': msg.y,
            'z': msg.z,
            'heading': msg.heading
        }

    def send_command(self, command, distance_or_degrees=0.0, wait_time=2.0):
        """Send a command to the offboard manager and wait"""
        msg = OffboardNavCommand()
        msg.command = command
        msg.distance_or_degrees = distance_or_degrees

        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"Command: {command} (value: {distance_or_degrees})")

        # Spin during wait to keep receiving position updates
        end_time = time.time() + wait_time
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)  # Small sleep to prevent busy waiting

    def publish_offboard_control_mode(self):
        """Publish offboard control mode to indicate we're sending position setpoints"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)

    def calculate_figure8_point(self, t):
        """
        Calculate position and heading for figure 8 at parameter t

        Parametric equations:
            x(t) = A * sin(t)
            y(t) = (A/2) * sin(2*t)

        Derivatives (for heading):
            dx/dt = A * cos(t)
            dy/dt = A * cos(2*t)
            heading = atan2(dy/dt, dx/dt)

        Args:
            t: Parameter from 0 to 2π

        Returns:
            tuple: (x, y, heading) relative to start position
        """
        A = self.figure8_size

        # Position
        x = A * math.sin(t)
        y = (A / 2.0) * math.sin(2.0 * t)

        # Derivatives for heading calculation
        dx_dt = A * math.cos(t)
        dy_dt = A * math.cos(2.0 * t)

        # Calculate heading (tangent to the curve)
        # In NED frame, heading is measured from North (x-axis) clockwise
        heading = math.atan2(dy_dt, dx_dt)

        return x, y, heading

    def publish_trajectory_setpoint(self, x, y, z, yaw):
        """Publish trajectory setpoint to PX4"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)

        self.trajectory_pub.publish(msg)

    def trajectory_update(self):
        """Timer callback to update trajectory along figure 8"""
        if not self.pattern_started or self.start_position is None:
            return

        # Publish offboard control mode heartbeat
        self.publish_offboard_control_mode()

        # Calculate current point on figure 8
        rel_x, rel_y, heading = self.calculate_figure8_point(self.t)

        # Convert to absolute position (NED frame)
        abs_x = self.start_position['x'] + rel_x
        abs_y = self.start_position['y'] + rel_y
        abs_z = self.start_position['z']  # Maintain altitude

        # Adjust heading to absolute frame
        abs_heading = self.start_position['heading'] + heading

        # Publish trajectory setpoint
        self.publish_trajectory_setpoint(abs_x, abs_y, abs_z, abs_heading)

        # Update parameter for next iteration
        # dt = 1/update_rate, arc_length = speed * dt
        # For the figure 8 curve, we approximate by advancing t proportionally
        dt = 1.0 / self.update_rate
        t_increment = (self.flight_speed * dt) / self.figure8_size  # Rough approximation
        self.t += t_increment

        # One complete figure 8 is t from 0 to 2π
        if self.t >= 2.0 * math.pi:
            self.get_logger().info("Figure 8 pattern completed!")
            self.pattern_started = False
            if self.trajectory_timer:
                self.trajectory_timer.cancel()

    def execute_figure8_pattern(self):
        """Execute the complete figure 8 pattern flight"""
        try:
            self.get_logger().info("Starting figure 8 pattern flight...")

            # 1. Initialize offboard mode using the manager
            self.get_logger().info("Step 1: Initializing...")
            self.send_command("start_offboard_heartbeat", wait_time=3.0)
            self.send_command("arm", wait_time=3.0)

            # 2. Wait for position data before takeoff
            self.get_logger().info("Step 2: Waiting for position data...")
            for i in range(20):  # Try for 2 seconds (20 * 100ms)
                rclpy.spin_once(self, timeout_sec=0.1)  # Spin to process callbacks
                if self.current_position is not None:
                    self.get_logger().info("Position data received!")
                    break

            if self.current_position is None:
                self.get_logger().error("No position data received! Cannot start pattern.")
                self._emergency_land()
                return

            # 3. Takeoff using offboard mode
            self.get_logger().info(f"Step 3: Taking off to {self.takeoff_altitude} meters (offboard mode)")
            self.send_command("offboard_takeoff", distance_or_degrees=self.takeoff_altitude, wait_time=10.0)

            # 4. Wait for position stabilization
            self.get_logger().info("Step 4: Waiting for position stabilization...")
            for _ in range(30):  # 3 seconds (30 * 100ms)
                rclpy.spin_once(self, timeout_sec=0.1)

            # Save start position
            self.start_position = self.current_position.copy()
            self.get_logger().info(f"Start position: x={self.start_position['x']:.2f}, "
                                 f"y={self.start_position['y']:.2f}, "
                                 f"z={self.start_position['z']:.2f}, "
                                 f"heading={math.degrees(self.start_position['heading']):.1f}°")

            # 5. Stop offboard manager heartbeat to take full control
            self.get_logger().info("Step 5: Taking control - stopping offboard manager heartbeat...")
            self.send_command("stop_offboard_heartbeat", wait_time=0.5)

            # 6. Start figure 8 pattern with direct setpoint control
            self.get_logger().info("Step 6: Starting figure 8 pattern...")
            self.get_logger().info("Publishing trajectory setpoints at 20Hz...")

            self.t = 0.0
            self.pattern_started = True

            # Create timer for trajectory updates
            self.trajectory_timer = self.create_timer(
                1.0 / self.update_rate,
                self.trajectory_update
            )

            # Wait for pattern to complete (2π / t_increment iterations)
            # Estimate: circumference ≈ 4*π*A, time ≈ circumference/speed
            estimated_time = (4.0 * math.pi * self.figure8_size) / self.flight_speed
            self.get_logger().info(f"Estimated completion time: {estimated_time:.1f} seconds")

            # Wait for completion with some buffer
            wait_time = estimated_time + 5.0
            end_time = time.time() + wait_time
            while time.time() < end_time and self.pattern_started:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.05)

            # 7. Return to start and land
            self.get_logger().info("Step 7: Returning to start position...")
            # Stop the trajectory timer if still running
            if self.trajectory_timer and not self.trajectory_timer.is_canceled():
                self.trajectory_timer.cancel()

            # Brief hold at current position
            for _ in range(20):  # 2 seconds (20 * 100ms)
                rclpy.spin_once(self, timeout_sec=0.1)

            # 8. Land (this will automatically stop offboard mode)
            self.get_logger().info("Step 8: Landing...")
            self.send_command("land", wait_time=10.0)
            self.send_command("disarm", wait_time=2.0)
            # No need to stop heartbeat - it was already stopped before the pattern

            self.get_logger().info("Figure 8 pattern flight completed successfully!")

        except Exception as e:
            self.get_logger().error(f"Figure 8 pattern flight failed: {str(e)}")
            self._emergency_land()

    def _emergency_land(self):
        """Emergency landing procedure"""
        self.get_logger().warn("Executing emergency landing procedure...")
        try:
            # Stop trajectory updates
            self.pattern_started = False
            if self.trajectory_timer and not self.trajectory_timer.is_canceled():
                self.trajectory_timer.cancel()

            self.send_command("land", wait_time=2.0)
            for _ in range(80):  # 8 seconds (80 * 100ms)
                rclpy.spin_once(self, timeout_sec=0.1)
            self.send_command("disarm", wait_time=1.0)
            # Heartbeat was already stopped before pattern started
        except Exception as e:
            self.get_logger().error(f"Emergency landing failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    flight_controller = FigureEightFlight()

    try:
        # Execute the figure 8 pattern flight
        flight_controller.execute_figure8_pattern()

    except KeyboardInterrupt:
        flight_controller.get_logger().info("Flight interrupted by user")
        flight_controller._emergency_land()

    finally:
        flight_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
