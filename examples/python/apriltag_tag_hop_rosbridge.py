#!/usr/bin/env python3
"""
The AprilTag tag hop over rosbridge: the same mission as apriltag_tag_hop.py,
but it needs no ROS install, only `pip install roslibpy`. It runs from the
VS Code server in the sim stack, from a laptop on the aircraft's network, or
from the aircraft itself, and calls the same two services the blocks and the
Node-RED flow use.

    python3 apriltag_tag_hop_rosbridge.py --host ros2-dev --route 0 2 4 --takeoff 1.3
    python3 apriltag_tag_hop_rosbridge.py --host <aircraft-ip> --route 0 2 4      # pilot hand-off

See apriltag_tag_hop.py for the mission rules (route order = direction, one
mission owner at a time, never land under a pilot).
"""

import argparse
import json
import sys
import time

import roslibpy

MANAGER = '/dexi/execute_blockly_command'
TAG_NAV = '/dexi/tag_nav/execute'
SRV_TYPE = 'dexi_interfaces/srv/ExecuteBlocklyCommand'
NAV_STATE_OFFBOARD = 14


class TagHop:
    def __init__(self, host, port):
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        self.manager = roslibpy.Service(self.ros, MANAGER, SRV_TYPE)
        self.tag_nav = roslibpy.Service(self.ros, TAG_NAV, SRV_TYPE)
        self.status = {}
        roslibpy.Topic(self.ros, '/dexi/tag_nav/status', 'std_msgs/msg/String').subscribe(self._on_status)

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg['data'])
        except (ValueError, KeyError):
            pass

    def call(self, service, command, parameter=0.0, timeout=30.0, **ned):
        req = {'command': command, 'parameter': float(parameter), 'timeout': float(timeout),
               'north': 0.0, 'east': 0.0, 'down': 0.0, 'yaw': 0.0, 'index': 0, 'r': 0, 'g': 0, 'b': 0}
        req.update({k: float(v) for k, v in ned.items()})
        # rosbridge enforces its own call timeout; ask for a little more than the command's.
        res = service.call(roslibpy.ServiceRequest(req), timeout=timeout + 10)
        mark = '✓' if res['success'] else '✗'
        print(f"{mark} {command} {parameter:g}: {res['message']} ({res['execution_time']:.1f} s)", flush=True)
        return res['success'], res['message']

    def mission_has_aircraft(self):
        time.sleep(0.3)
        return self.status.get('nav_state') == NAV_STATE_OFFBOARD and self.status.get('engaged') is True

    def run(self, route, speed, takeoff_alt, timeout_center, timeout_transit):
        if takeoff_alt:
            for cmd, param, t in (('start_offboard_heartbeat', 0, 5), ('arm', 0, 10), ('offboard_takeoff', takeoff_alt, 30)):
                ok, _ = self.call(self.manager, cmd, param, t)
                if not ok:
                    return self.fail('could not get airborne', land=cmd == 'offboard_takeoff')
        else:
            self.call(self.manager, 'start_setpoint_stream', 0, 5)
            print(f'hover over tag {route[0]} and flip the Offboard switch ...', flush=True)
            ok, _ = self.call(self.tag_nav, 'wait_for_offboard', 0, 600)
            if not ok:
                return self.fail('hand-off did not happen', land=False)

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
        if land:
            print(f'{why} -> landing', flush=True)
            self.call(self.manager, 'land', 0, 40)
        else:
            print(f'{why} -> pilot has the aircraft, stopping', flush=True)
        return 1

    def close(self):
        self.ros.terminate()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--host', default='localhost', help='rosbridge host (ros2-dev in the sim stack, the aircraft IP in the lab)')
    p.add_argument('--port', type=int, default=9090)
    p.add_argument('--route', type=int, nargs='+', default=[0, 1, 2, 3, 4])
    p.add_argument('--speed', type=float, default=0.25)
    p.add_argument('--takeoff', type=float, default=0.0, metavar='ALT')
    p.add_argument('--timeout-center', type=float, default=25.0)
    p.add_argument('--timeout-transit', type=float, default=25.0)
    args = p.parse_args()
    hop = TagHop(args.host, args.port)
    try:
        code = hop.run(args.route, args.speed, args.takeoff, args.timeout_center, args.timeout_transit)
    except KeyboardInterrupt:
        print('interrupted; the aircraft keeps its last hold, land it from the radio', flush=True)
        code = 130
    finally:
        hop.close()
    sys.exit(code)


if __name__ == '__main__':
    main()
