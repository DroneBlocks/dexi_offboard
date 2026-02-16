#!/usr/bin/env python3
"""
Takeoff diagnostics monitor for optical flow altitude issues

This script monitors PX4 estimator and land detector topics to diagnose
takeoff problems with optical flow + rangefinder setups (no GPS).

It logs key fields to CSV and prints real-time warnings for:
- Land detector false positives during takeoff
- Terrain estimate resets (cs_in_air toggling)
- Optical flow fusion dropout
- High terrain variance

Usage:
    python3 takeoff_diagnostics.py

    Then run your flight script in another terminal. This will log data
    from arming until disarm and save to a timestamped CSV file.

Requirements:
    - ROS2 environment sourced
    - px4_msgs package available
    - PX4 connected via DDS (micro-XRCE agent running)

Related PX4 Issues:
    - https://github.com/PX4/PX4-Autopilot/issues/24653
    - https://github.com/PX4/PX4-Autopilot/issues/25258
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import csv
from datetime import datetime
from pathlib import Path

from px4_msgs.msg import (
    EstimatorStatusFlags,
    VehicleLandDetected,
    VehicleLocalPosition,
    SensorOpticalFlow,
    VehicleStatus,
)


class TakeoffDiagnostics(Node):
    def __init__(self):
        super().__init__('takeoff_diagnostics')

        # QoS profile for PX4 topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # State tracking
        self.data = {
            'armed': False,
            'cs_in_air': False,
            'cs_opt_flow': False,
            'cs_rng_hgt': False,
            'cs_baro_hgt': False,
            'cs_rng_terrain': False,
            'cs_opt_flow_terrain': False,
            'landed': False,
            'ground_contact': False,
            'maybe_landed': False,
            'dist_bottom': 0.0,
            'dist_bottom_valid': False,
            'dist_bottom_var': 0.0,
            'z': 0.0,
            'vz': 0.0,
            'flow_quality': 0,
        }

        self.prev_in_air = None
        self.prev_landed = None
        self.prev_opt_flow = None
        self.start_time = None
        self.issues_detected = []

        # CSV setup
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = Path(f'takeoff_diag_{timestamp}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        fields = ['t_ms', 'armed', 'cs_in_air', 'cs_opt_flow', 'cs_rng_hgt',
                  'cs_baro_hgt', 'cs_rng_terrain', 'cs_opt_flow_terrain',
                  'landed', 'ground_contact', 'maybe_landed',
                  'dist_bottom', 'dist_bottom_valid', 'dist_bottom_var',
                  'z', 'vz', 'flow_quality', 'event']
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fields)
        self.csv_writer.writeheader()

        # Subscribers
        self.create_subscription(
            EstimatorStatusFlags,
            '/fmu/out/estimator_status_flags',
            self.estimator_flags_cb, qos)

        self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self.land_detected_cb, qos)

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_cb, qos)

        self.create_subscription(
            SensorOpticalFlow,
            '/fmu/out/sensor_optical_flow',
            self.optical_flow_cb, qos)

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_cb, qos)

        # Logging timer at 20Hz
        self.create_timer(0.05, self.log_data)

        self.print_banner()

    def print_banner(self):
        print("\n" + "=" * 70)
        print("  TAKEOFF DIAGNOSTICS MONITOR")
        print("  Monitoring for optical flow / altitude issues")
        print("=" * 70)
        print(f"  Logging to: {self.csv_path}")
        print("  Watching for:")
        print("    - cs_in_air toggling (terrain reset trigger)")
        print("    - False positive landing detection")
        print("    - Optical flow fusion dropout")
        print("    - High terrain variance")
        print("=" * 70)
        print("  Arm and takeoff when ready. Press Ctrl+C to stop.")
        print("=" * 70 + "\n")

    def estimator_flags_cb(self, msg):
        self.data['cs_in_air'] = msg.cs_in_air
        self.data['cs_opt_flow'] = msg.cs_opt_flow
        self.data['cs_rng_hgt'] = msg.cs_rng_hgt
        self.data['cs_baro_hgt'] = msg.cs_baro_hgt
        self.data['cs_rng_terrain'] = msg.cs_rng_terrain
        self.data['cs_opt_flow_terrain'] = msg.cs_opt_flow_terrain

        # Detect cs_in_air toggle
        if self.prev_in_air is not None and self.prev_in_air != msg.cs_in_air:
            event = f"CS_IN_AIR: {self.prev_in_air} -> {msg.cs_in_air}"
            self.log_event(event)
            print(f"\n>>> {event} <<<")
            if not msg.cs_in_air and self.data['armed']:
                print("    ^^^ TERRAIN RESET TRIGGERED! This may cause altitude issues ^^^")
                self.issues_detected.append(('TERRAIN_RESET', self.get_elapsed_ms()))
        self.prev_in_air = msg.cs_in_air

        # Detect optical flow fusion change
        if self.prev_opt_flow is not None and self.prev_opt_flow != msg.cs_opt_flow:
            event = f"CS_OPT_FLOW: {self.prev_opt_flow} -> {msg.cs_opt_flow}"
            self.log_event(event)
            print(f"\n>>> {event} <<<")
            if not msg.cs_opt_flow and self.data['cs_in_air']:
                print("    ^^^ OPTICAL FLOW STOPPED FUSING IN FLIGHT! ^^^")
                self.issues_detected.append(('OPT_FLOW_DROPOUT', self.get_elapsed_ms()))
        self.prev_opt_flow = msg.cs_opt_flow

    def land_detected_cb(self, msg):
        self.data['landed'] = msg.landed
        self.data['ground_contact'] = msg.ground_contact
        self.data['maybe_landed'] = msg.maybe_landed

        # Detect landed flag change
        if self.prev_landed is not None and self.prev_landed != msg.landed:
            event = f"LANDED: {self.prev_landed} -> {msg.landed}"
            self.log_event(event)
            print(f"\n>>> {event} <<<")
            if msg.landed and self.data['armed'] and self.data['cs_in_air']:
                print("    ^^^ FALSE POSITIVE LANDING DETECTION DURING FLIGHT! ^^^")
                self.issues_detected.append(('FALSE_LAND_DETECT', self.get_elapsed_ms()))
        self.prev_landed = msg.landed

    def local_position_cb(self, msg):
        self.data['z'] = msg.z
        self.data['vz'] = msg.vz
        self.data['dist_bottom'] = msg.dist_bottom
        self.data['dist_bottom_valid'] = msg.dist_bottom_valid
        self.data['dist_bottom_var'] = msg.dist_bottom_var

    def optical_flow_cb(self, msg):
        self.data['flow_quality'] = msg.quality

    def vehicle_status_cb(self, msg):
        was_armed = self.data['armed']
        self.data['armed'] = (msg.arming_state == 2)

        if not was_armed and self.data['armed']:
            self.start_time = self.get_clock().now()
            self.issues_detected = []
            self.log_event("ARMED")
            print("\n*** ARMED - Monitoring started ***\n")

        elif was_armed and not self.data['armed']:
            self.log_event("DISARMED")
            print("\n*** DISARMED - Monitoring stopped ***")
            self.print_summary()

    def get_elapsed_ms(self):
        if self.start_time is None:
            return 0
        return int((self.get_clock().now() - self.start_time).nanoseconds / 1e6)

    def log_event(self, event):
        row = {'t_ms': self.get_elapsed_ms(), 'event': event}
        row.update(self.data)
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def log_data(self):
        # Only log when armed
        if not self.data['armed']:
            return

        t_ms = self.get_elapsed_ms()

        # Log to CSV
        row = {'t_ms': t_ms, 'event': ''}
        row.update(self.data)
        self.csv_writer.writerow(row)

        # Print status line
        status = (
            f"t={t_ms:6d}ms | "
            f"in_air={str(self.data['cs_in_air']):5} | "
            f"landed={str(self.data['landed']):5} | "
            f"opt_flow={str(self.data['cs_opt_flow']):5} | "
            f"dist={self.data['dist_bottom']:5.2f}m | "
            f"var={self.data['dist_bottom_var']:6.3f} | "
            f"qual={self.data['flow_quality']:3d}"
        )
        print(status, end='\r')

        # Check for warning conditions
        if self.data['dist_bottom_var'] > 0.5:
            event = f"HIGH_VARIANCE: {self.data['dist_bottom_var']:.3f}"
            print(f"\n!!! {event} !!!")
            self.log_event(event)

        if not self.data['cs_opt_flow'] and self.data['cs_in_air']:
            if self.data['flow_quality'] < 10:
                print(f"\n!!! LOW FLOW QUALITY: {self.data['flow_quality']} !!!")

    def print_summary(self):
        print("\n" + "=" * 70)
        print("  FLIGHT SUMMARY")
        print("=" * 70)
        print(f"  Log file: {self.csv_path}")

        if self.issues_detected:
            print(f"\n  Issues detected ({len(self.issues_detected)}):")
            for issue, t_ms in self.issues_detected:
                print(f"    - {issue} at t={t_ms}ms")
            print("\n  Recommended actions:")
            print("    1. Check EKF2_HGT_REF parameter (should be 2 for indoor)")
            print("    2. Increase LNDMC_TRIG_TIME to 2.0s")
            print("    3. Verify rangefinder is working (check dist_bottom)")
        else:
            print("\n  No critical issues detected during flight.")

        print("=" * 70 + "\n")

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f'CSV saved to {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffDiagnostics()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user.")
        if node.issues_detected:
            node.print_summary()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
