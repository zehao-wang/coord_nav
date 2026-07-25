#!/usr/bin/env python3
"""Drive the real car A->B around obstacles with the MPC baseline.

Run inside the car ROS env:  `roscar` first, then e.g.

    python scripts/run_live.py                      # variant 2, mag 40, B = 1.0 m ahead
    python scripts/run_live.py --variant 1          # continuous v,omega
    python scripts/run_live.py --goal-dist 1.2 --magnitude 40

Variant 2 (discrete grid-hop) is the default baseline. START SMALL: the default is
LiveConfig.magnitude = 40 (the motors' friction floor is ~14 PWM, and below ~17.4
the planner cannot steer at all, so if the car does not move, raise --magnitude).

Ctrl-C hard-estops. Link loss and an imminent-collision guard also estop.
"""

import os
import sys
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from carclient import CarClient
from mpc_baseline import config as C
from mpc_baseline.runner import MPCRunner


def build_cfg(args):
    if args.variant == 1:
        cfg = C.live_config_v1()
        cfg.v_max = C._v_of_pwm(args.magnitude, cfg.robot)
        cfg.robot.wheel_pwm_cap = max(cfg.robot.wheel_pwm_cap, args.magnitude * 1.8)
        if args.pwm_offset is not None:
            cfg.robot.pwm_offset = args.pwm_offset
    else:
        cfg = C.live_config_v2()
        cfg.step_magnitude = args.magnitude
        cfg.step_duration = args.step_duration
        if args.allow_rotation:
            cfg.actions = tuple(cfg.actions) + (9, 10)
    cfg.goal.goal_dist = args.goal_dist
    cfg.goal.goal_tol = args.goal_tol
    cfg.goal.timeout_s = args.timeout
    return cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", type=int, default=2, choices=[1, 2],
                    help="2 = discrete grid (default baseline), 1 = continuous v,omega")
    ap.add_argument("--goal-dist", type=float, default=1.0, help="metres ahead to B")
    ap.add_argument("--goal-tol", type=float, default=0.15, help="reached radius (m)")
    ap.add_argument("--magnitude", type=float, default=C.LiveConfig.magnitude,
                    help="drive magnitude (PWM); default 40, the validated value")
    ap.add_argument("--step-duration", type=float, default=0.5,
                    help="variant 2 seconds per hop")
    ap.add_argument("--pwm-offset", type=float, default=None,
                    help="override the measured friction offset (PWM) added to every wheel")
    ap.add_argument("--tick-hz", type=float, default=C.TickConfig.rate_hz,
                    help="GLOBAL tick rate; must equal the car's perception rate")
    ap.add_argument("--timeout", type=float, default=25.0, help="episode timeout (s)")
    ap.add_argument("--allow-rotation", action="store_true",
                    help="variant 2: also allow in-place rotation actions")
    args = ap.parse_args()

    cfg = build_cfg(args)
    live = C.LiveConfig()
    live.tick.rate_hz = args.tick_hz
    live.magnitude = args.magnitude

    # An abnormal exit / Ctrl-C / link loss / imminent collision always hard-estops
    # (the MCU has no motor timeout). The runner refuses to drive unless the MCU
    # link reads healthy first, so the link-loss interlock is guaranteed armed.
    client = CarClient(magnitude=args.magnitude)
    runner = MPCRunner(args.variant, cfg, live, C.ObstacleConfig(), client)
    try:
        summary = runner.run()
        print("\nRESULT:", summary)
    finally:
        client.close()


if __name__ == "__main__":
    main()
