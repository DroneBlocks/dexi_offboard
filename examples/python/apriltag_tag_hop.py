#!/usr/bin/env python3
"""
AprilTag tag hop in Python, the same mission the DroneBlocks blocks and the
Node-RED flow fly: center on the first tag, fly until the next tag is seen,
center on it, repeat down the route, land.

Two services carry everything, both with the ExecuteBlocklyCommand request:

    /dexi/execute_blockly_command   the offboard manager (arm, takeoff, land, hand-off stream)
    /dexi/tag_nav/execute           the AprilTag primitives (center_on_tag, fly_until_tag, ...)

Two ways to start:

    # Pilot hand-off (default). Fly by hand over the first tag, flip the RC
    # Offboard switch; the mission takes over.
    python3 apriltag_tag_hop.py --route 0 2 4

    # From the ground, no pilot: arm, take off, run the route, land.
    python3 apriltag_tag_hop.py --route 0 2 4 --takeoff 1.2

Tag ids increase along the corridor, so a lower id than the previous one means
"fly backward": --route 0 2 4 2 0 goes out and comes back. Transit speed is
0.25 m/s by default; faster than ~0.3 m/s outruns the detector on a 6 in tag.

One mission owner at a time: the Node-RED tag navigation flow also starts a
mission when the pilot flips to Offboard. If that flow is deployed on the
aircraft, press its DISENGAGE/RESET and leave the switch alone, or disable the
flow's tab, before running this script in hand-off mode.
"""

import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dexi_interfaces.srv import ExecuteBlocklyCommand

MANAGER = '/dexi/execute_blockly_command'
TAG_NAV = '/dexi/tag_nav/execute'
NAV_STATE_OFFBOARD = 14


class TagHop(Node):
    def __init__(self):
        super().__init__('apriltag_tag_hop')
        self.manager = self.create_client(ExecuteBlocklyCommand, MANAGER)
        self.tag_nav = self.create_client(ExecuteBlocklyCommand, TAG_NAV)
        for client, name in ((self.manager, MANAGER), (self.tag_nav, TAG_NAV)):
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'waiting for {name} ...')
        # tag_nav's status tells us whether the mission still has the aircraft.
        self.status = {}
        self.create_subscription(String, '/dexi/tag_nav/status', self._on_status, 10)

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass

    def call(self, client, command, parameter=0.0, timeout=30.0, **ned):
        """One command, blocking until the service answers. Returns (success, message)."""
        req = ExecuteBlocklyCommand.Request()
        req.command = command
        req.parameter = float(parameter)
        req.timeout = float(timeout)
        for k, v in ned.items():            # north / east / down / yaw
            setattr(req, k, float(v))
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        mark = '✓' if res.success else '✗'
        self.get_logger().info(f'{mark} {command} {parameter:g}: {res.message} ({res.execution_time:.1f} s)')
        return res.success, res.message

    def mission_has_aircraft(self):
        """True while PX4 is in Offboard and tag_nav is engaged, i.e. not handed back to the pilot."""
        rclpy.spin_once(self, timeout_sec=0.3)
        return self.status.get('nav_state') == NAV_STATE_OFFBOARD and self.status.get('engaged') is True

    def run(self, route, speed, takeoff_alt, timeout_center, timeout_transit):
        if takeoff_alt:
            # Ground start: the GCS way. The heartbeat commands Offboard itself.
            for cmd, param, t in (('start_offboard_heartbeat', 0, 5), ('arm', 0, 10), ('offboard_takeoff', takeoff_alt, 30)):
                ok, _ = self.call(self.manager, cmd, param, t)
                if not ok:
                    return self.fail('could not get airborne', land=cmd == 'offboard_takeoff')
        else:
            # Pilot hand-off: stream setpoints without commanding Offboard, then wait for the switch.
            self.call(self.manager, 'start_setpoint_stream', 0, 5)
            self.get_logger().info('hover over tag %d and flip the Offboard switch ...' % route[0])
            ok, _ = self.call(self.tag_nav, 'wait_for_offboard', 0, 600)
            if not ok:
                return self.fail('hand-off did not happen', land=False)

        # The route, exactly as the Node-RED mission node derives it.
        steps = [('center_on_tag', route[0], timeout_center, {})]
        for prev, tag in zip(route, route[1:]):
            direction = 1 if tag > prev else -1
            steps.append(('fly_until_tag', tag, timeout_transit, {'north': direction * speed}))
            steps.append(('center_on_tag', tag, timeout_center, {}))

        for command, tag, t, ned in steps:
            ok, message = self.call(self.tag_nav, command, tag, t, **ned)
            if not ok:
                # From the ground there is no pilot: always land. After a hand-off, land
                # only while the mission still has the aircraft.
                return self.fail(f'{command} {tag}: {message}', land=bool(takeoff_alt) or self.mission_has_aircraft())

        ok, _ = self.call(self.manager, 'land', 0, 40)
        return 0 if ok else 1

    def fail(self, why, land):
        # A step that fails because the pilot took the aircraft back must not land under the pilot.
        if land:
            self.get_logger().error(f'{why} -> landing')
            self.call(self.manager, 'land', 0, 40)
        else:
            self.get_logger().error(f'{why} -> pilot has the aircraft, stopping')
        return 1


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--route', type=int, nargs='+', default=[0, 1, 2, 3, 4, 5], help='tag ids in flight order')
    p.add_argument('--speed', type=float, default=0.25, help='transit speed, m/s')
    p.add_argument('--takeoff', type=float, default=0.0, metavar='ALT', help='start from the ground at this altitude (m) instead of a pilot hand-off')
    p.add_argument('--timeout-center', type=float, default=25.0)
    p.add_argument('--timeout-transit', type=float, default=25.0)
    args = p.parse_args()

    rclpy.init()
    node = TagHop()
    try:
        code = node.run(args.route, args.speed, args.takeoff, args.timeout_center, args.timeout_transit)
    except KeyboardInterrupt:
        node.get_logger().warning('interrupted; the aircraft keeps its last hold, land it from the radio')
        code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
