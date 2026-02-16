#!/usr/bin/env python3
"""
DEXI circle orbit flight pattern example

This example demonstrates a circular orbit pattern with two heading modes:
1. Tangent mode: Drone nose follows the flight path (faces direction of travel)
2. Center-facing mode: Drone nose always points toward the center of the circle

The circle is generated using parametric equations:
    x(t) = r * cos(t)
    y(t) = r * sin(t)
where r is the radius and t goes from 0 to 2π

Heading calculations:
    Tangent mode: heading = atan2(dy/dt, dx/dt) = atan2(r*cos(t), -r*sin(t))
    Center-facing: heading = atan2(-y, -x) = t + π

Usage:
    python3 circle_orbit.py

Requirements:
    - ROS2 environment sourced
    - dexi_offboard node running
    - px4_msgs and dexi_interfaces packages available

Note:
    Uses 'offboard_takeoff' to maintain offboard mode throughout the flight,
    allowing direct trajectory setpoint control for smooth circular motion.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from dexi_interfaces.msg import OffboardNavCommand
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleLocalPosition
import time
import math


class CircleOrbitFlight(Node):
    def __init__(self):
        super().__init__('circle_orbit_flight')

        # Flight parameters
        self.takeoff_altitude = 3.0      # meters
        self.circle_diameter = 1.0       # meters
        self.circle_radius = self.circle_diameter / 2.0
        self.flight_speed = 0.3          # m/s (speed along the path)
        self.update_rate = 20.0          # Hz (trajectory update rate)
        self.num_orbits = 2              # Number of complete circles to fly

        # Heading mode: 'tangent' or 'center'
        # 'tangent': drone faces direction of travel (nose follows path)
        # 'center': drone always faces the center of the circle
        self.heading_mode = 'tangent'    # Change to 'center' for center-facing mode

        # State variables
        self.current_position = None
        self.start_position = None
        self.pattern_started = False

        # Configure QoS profile to match offboard manager
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
        self.t = 0.0  # Parameter for circle trajectory

        # Wait for setup
        time.sleep(1)
        self.get_logger().info("Circle orbit flight example initialized")
        self.get_logger().info(f"Diameter: {self.circle_diameter}m, Speed: {self.flight_speed}m/s")
        self.get_logger().info(f"Heading mode: {self.heading_mode}")
        self.get_logger().info(f"Number of orbits: {self.num_orbits}")

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

    def calculate_circle_point(self, t):
        """
        Calculate position and heading for circle at parameter t

        Parametric equations:
            x(t) = r * cos(t)
            y(t) = r * sin(t)

        Derivatives (for tangent heading):
            dx/dt = -r * sin(t)
            dy/dt = r * cos(t)
            tangent_heading = atan2(dy/dt, dx/dt)

        Center-facing heading:
            Points from current position toward center
            center_heading = atan2(-y, -x) = t + π

        Args:
            t: Parameter from 0 to 2π (one complete circle)

        Returns:
            tuple: (x, y, heading) relative to start position
        """
        r = self.circle_radius

        # Position on circle
        x = r * math.cos(t)
        y = r * math.sin(t)

        if self.heading_mode == 'center':
            # Point toward center of circle
            # Vector from current position to center is (-x, -y)
            heading = math.atan2(-y, -x)
        else:  # tangent mode
            # Calculate heading tangent to circle (direction of travel)
            dx_dt = -r * math.sin(t)
            dy_dt = r * math.cos(t)
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
        """Timer callback to update trajectory along circle"""
        if not self.pattern_started or self.start_position is None:
            return

        # Publish offboard control mode heartbeat
        self.publish_offboard_control_mode()

        # Calculate current point on circle
        rel_x, rel_y, heading = self.calculate_circle_point(self.t)

        # Convert to absolute position (NED frame)
        abs_x = self.start_position['x'] + rel_x
        abs_y = self.start_position['y'] + rel_y
        abs_z = self.start_position['z']  # Maintain altitude

        # Adjust heading to absolute frame
        abs_heading = self.start_position['heading'] + heading

        # Publish trajectory setpoint
        self.publish_trajectory_setpoint(abs_x, abs_y, abs_z, abs_heading)

        # Update parameter for next iteration
        # Calculate angular velocity: v = r*ω → ω = v/r
        dt = 1.0 / self.update_rate
        angular_velocity = self.flight_speed / self.circle_radius
        t_increment = angular_velocity * dt
        self.t += t_increment

        # Check if we've completed the desired number of orbits
        max_t = 2.0 * math.pi * self.num_orbits
        if self.t >= max_t:
            self.get_logger().info(f"Completed {self.num_orbits} orbit(s)!")
            self.pattern_started = False
            if self.trajectory_timer:
                self.trajectory_timer.cancel()

    def execute_circle_orbit(self):
        """Execute the complete circle orbit pattern flight"""
        try:
            self.get_logger().info("Starting circle orbit flight...")

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

            # 6. Start circle orbit pattern with direct setpoint control
            self.get_logger().info("Step 6: Starting circle orbit pattern...")
            self.get_logger().info(f"Flying {self.num_orbits} orbit(s) with {self.heading_mode} heading mode")
            self.get_logger().info("Publishing trajectory setpoints at 20Hz...")

            self.t = 0.0
            self.pattern_started = True

            # Create timer for trajectory updates
            self.trajectory_timer = self.create_timer(
                1.0 / self.update_rate,
                self.trajectory_update
            )

            # Calculate estimated completion time
            # Circumference = 2πr, time = distance/speed
            circumference = 2.0 * math.pi * self.circle_radius
            time_per_orbit = circumference / self.flight_speed
            estimated_time = time_per_orbit * self.num_orbits
            self.get_logger().info(f"Estimated completion time: {estimated_time:.1f} seconds")

            # Wait for pattern to complete
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

            self.get_logger().info("Circle orbit flight completed successfully!")

        except Exception as e:
            self.get_logger().error(f"Circle orbit flight failed: {str(e)}")
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

    flight_controller = CircleOrbitFlight()

    try:
        # Execute the circle orbit pattern flight
        flight_controller.execute_circle_orbit()

    except KeyboardInterrupt:
        flight_controller.get_logger().info("Flight interrupted by user")
        flight_controller._emergency_land()

    finally:
        flight_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
