#!/usr/bin/env python3
"""
AprilTag tag hop from a pilot hand-off: fly by hand over the first tag, flip
the RC Offboard switch, and the mission takes over. It centers on each tag down
the route and lands on the last one. Flip the switch back at any time to take
the aircraft; the mission stops without landing under the pilot.

    python3 apriltag_tag_hop_handoff.py                 # tag 0 to tag 1
    python3 apriltag_tag_hop_handoff.py --route 0 2 4

One mission owner at a time: the Node-RED tag navigation flow also starts a
mission when the pilot flips to Offboard. If that flow is deployed on the
aircraft, press its DISENGAGE/RESET and leave the switch alone, or disable the
flow's tab, before running this script.

To start from the ground with no pilot, run apriltag_tag_hop.py.
"""

import argparse

from apriltag_tag_hop import add_route_args, fly


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_route_args(p)
    fly(p.parse_args(), takeoff_alt=0.0)


if __name__ == '__main__':
    main()
