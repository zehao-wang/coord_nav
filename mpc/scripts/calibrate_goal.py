#!/usr/bin/env python3
"""Measure how far the car actually drives forward, to set goal B precisely.

Goal B is defined as "the position 5 s of forward driving reaches at the default
params". Motor deadzone and load make the nominal PWM->speed mapping approximate,
so this drives straight for a fixed time at a chosen magnitude and reports the
odom displacement -- use that as --goal-dist for run_live.py.

    roscar
    python scripts/calibrate_goal.py --magnitude 20 --seconds 5

Drives the car! Keep the area clear; Ctrl-C estops.
"""

import os
import sys
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import numpy as np
from carclient import CarClient, Action


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--magnitude", type=float, default=20.0)
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()

    client = CarClient(magnitude=args.magnitude)
    if client.wait_pose(timeout=5.0) is None:
        print("no /odom -- is the car up? (roscar + ping)")
        sys.exit(2)
    p0 = client.pose()
    start = np.array([p0.x, p0.y])
    print("driving FORWARD mag=%.0f for %.1fs ..." % (args.magnitude, args.seconds))
    try:
        # one forward pulse for the full duration (drive_action caps at 3 s, so
        # re-issue for longer runs); the car keep-alives it locally.
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.seconds:
            remain = args.seconds - (time.monotonic() - t0)
            client.drive(Action.FORWARD, args.magnitude, min(2.5, remain))
            time.sleep(min(2.3, max(0.1, remain)))
        client.stop()
        time.sleep(0.4)
        p1 = client.pose()
        disp = float(np.hypot(p1.x - start[0], p1.y - start[1]))
        print("\ndisplacement = %.3f m over %.1fs  (=> speed ~%.3f m/s)" % (
            disp, args.seconds, disp / args.seconds))
        print("use:  --goal-dist %.2f" % disp)
    except BaseException:
        client.estop()
        raise
    finally:
        client.close()


if __name__ == "__main__":
    main()
