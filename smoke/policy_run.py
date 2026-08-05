#!/usr/bin/env python3
"""Run an MPC policy on the real car to a goal B, record the trajectory, and plot it.

The experiment/visualization tool for the MPC baseline (variant 1 / variant 2).
It drives the car with PolicyRunner, records pose + plan + obstacles every cycle,
then writes a trajectory plot (in the START body frame: +x forward, +y left, with
only the near obstacles so the go-around is clear) plus a JSON record to
smoke/results/.

    roscar
    python smoke/policy_run.py --variant 1 --bx 3 --by 0 --pose odom
    python smoke/policy_run.py --variant 2 --bx 1.5 --by 0.5 --mag 30

Drives the car! Keep the area clear; Ctrl-C estops. Pose source: 'odom' (motor,
accurate yaw, default), 'lidar' (ICP, drifts over long runs), 'dead_reckon'.
"""

import os
import sys
import json
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mpc"))

import numpy as np
import rospy
from sensor_msgs.msg import Imu
from carclient import CarClient
from mpc_baseline import config as C
from mpc_baseline.registry import POLICY_REGISTRY, build_policy
from mpc_baseline.runner import PolicyRunner

# legacy numeric variants map onto their registry entries; any other registry
# key (mpc_vw_t, mpc_grid_t, your own model) is usable directly via --variant
_VARIANT_KEYS = {"1": "mpc_vw", "2": "mpc_grid"}

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

# gyro sign relative to /odom CCW+ convention (from calibration/calib_gyro.py: OPPOSITE)
GYRO_SIGN = -1.0


class GyroYaw(object):
    """Second yaw estimate by integrating /imu angular_velocity.z, to cross-check the
    /odom yaw during a real drive. car_base_node now integrates the SAME gyro for odom
    yaw (yaw_source=gyro), so this checks the two integrations, not two sensors.
    Bias = mean of the first `nbias` samples (car must be still)."""

    def __init__(self, nbias=15):
        self._yaw = 0.0
        self._last_t = None
        self._bias = None
        self._acc = []
        self._nbias = nbias
        rospy.Subscriber("/imu/data_raw", Imu, self._cb)

    def _cb(self, m):
        t = m.header.stamp.to_sec()               # car clock; only dt is used
        gz = m.angular_velocity.z
        if self._bias is None:
            self._acc.append(gz)
            if len(self._acc) >= self._nbias:
                self._bias = float(np.mean(self._acc))
            self._last_t = t
            return
        if self._last_t is not None:
            dt = t - self._last_t
            if 0.0 < dt < 0.5:
                self._yaw += GYRO_SIGN * (gz - self._bias) * dt
        self._last_t = t

    def yaw(self):
        return self._yaw if self._bias is not None else None


def run(args):
    client = CarClient(magnitude=args.mag)
    if client.wait_pose(timeout=6) is None or client.wait_obstacles(timeout=6) is None:
        print("no /odom or /obstacles -- is the car up? (roscar + ping)")
        return None
    time.sleep(0.4)
    key = _VARIANT_KEYS.get(str(args.variant), str(args.variant))
    if key not in POLICY_REGISTRY:
        print("unknown policy %r; use 1, 2 or a registry key: %s"
              % (args.variant, ", ".join(sorted(POLICY_REGISTRY))))
        return None
    policy, cfg = build_policy(key, args.mag, args.bx, goal_y=args.by)
    cfg.goal.goal_tol = args.tol
    if args.extra_margin is not None:
        # experiment knob: widen the planner's obstacle inflation beyond the
        # benchmarked default (0.10). The 2026-08-05 A/B guard-stopped v2 at a
        # dead-centre box: it skims the inflated boundary by design, and one
        # diagonal hop of execution error (~0.1 m) can cross the guard line.
        cfg.cost.extra_margin = args.extra_margin
    cfg.goal.timeout_s = args.timeout
    live = C.LiveConfig()
    live.magnitude = args.mag
    live.tick.rate_hz = args.tick_hz
    live.execute_steps = args.exec_steps
    # Guard ON by default, matching LiveConfig and the GUI. This flag shipped OFF
    # under a "redundant, false-tripping" rationale that a real run then repudiated
    # (output/2026-07-25_20-01-44: 126 mm of the 130 mm footprint inside an obstacle
    # for 3 ticks, the guard would have fired on all three) -- the GUI default was
    # flipped after that incident but this CLI kept the pre-incident default.
    live.collision_abort = args.guard

    rec = []
    gyro = GyroYaw()
    time.sleep(2.0)                       # let the gyro settle its bias while still

    def on_step(d):
        o = client.obstacles()
        mp = client.pose()
        d["circles"] = list(o.circles) if o else []
        d["motor"] = [mp.x, mp.y, mp.yaw] if mp else None
        d["gyro_yaw"] = gyro.yaw()        # independent yaw (cross-check vs odom)
        d["t"] = time.time()
        rec.append(d)

    runner = PolicyRunner(policy, cfg, live, C.ObstacleConfig(), client,
                          log=(print if args.verbose else (lambda m: None)),
                          pose_source=args.pose, on_step=on_step,
                          collision_estop=False)
    print("RUN %s -> B=(%.2f,%.2f)  pose=%s mag=%.0f ..." % (
        key, args.bx, args.by, args.pose, args.mag))
    try:
        summary = runner.run()
    finally:
        try:
            client.close()
        except Exception:
            pass
    print("SUMMARY:", summary)
    return rec, summary


def plot(rec, summary, args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    P = np.array([r["pose"] for r in rec])
    V = np.array([r["v"] if r["v"] is not None else 0.0 for r in rec])
    W = np.array([r["w"] if r["w"] is not None else 0.0 for r in rec])
    x0, y0, th0 = P[0]
    c0, s0 = np.cos(-th0), np.sin(-th0)

    def to_start(x, y):
        dx, dy = x - x0, y - y0
        return np.array([c0 * dx - s0 * dy, s0 * dx + c0 * dy])

    traj = np.array([to_start(p[0], p[1]) for p in P])
    goal = to_start(*rec[0]["goal"])
    near = []
    for r in rec:
        rp = to_start(r["pose"][0], r["pose"][1])
        rth = r["pose"][2] - th0
        cc, ss = np.cos(rth), np.sin(rth)
        for bx, by, br in r["circles"]:
            p = rp + np.array([cc * bx - ss * by, ss * bx + cc * by])
            if np.min(np.hypot(traj[:, 0] - p[0], traj[:, 1] - p[1])) < 1.2:
                near.append((p[0], p[1], br))
    clr = [min([np.hypot(bx, by) - br for bx, by, br in r["circles"]], default=99)
           for r in rec]
    plen = float(np.sum(np.hypot(np.diff(traj[:, 0]), np.diff(traj[:, 1]))))
    t = np.array([r["t"] for r in rec]) - rec[0]["t"]

    fig, ax = plt.subplots(1, 2, figsize=(13, 6))
    a = ax[0]
    for x, y, rr in near:
        a.add_patch(plt.Circle((x, y), rr, color="tab:red", alpha=0.08))
    a.plot(traj[:, 0], traj[:, 1], "-o", ms=3, color="tab:blue", lw=2, label="car path")
    a.plot(0, 0, "ks", ms=10, label="start A")
    a.plot(goal[0], goal[1], "g*", ms=22, label="goal B")
    a.set_aspect("equal"); a.grid(alpha=0.3); a.legend(loc="upper left")
    a.set_xlabel("x forward (m)"); a.set_ylabel("y left (m)")
    a.set_title("%s  %s  reached=%s  path=%.1fm  clear=%.2fm" % (
        args.variant, args.pose, summary.get("reached"), plen, min(clr)))
    b = ax[1]
    b.plot(t, V, label="v (m/s)"); b.plot(t, W, label="w (rad/s)")
    b.plot(t, [cc if cc < 3 else np.nan for cc in clr], color="tab:green", label="clearance (m)")
    b.grid(alpha=0.3); b.legend(); b.set_xlabel("t (s)")
    b.set_title("control & clearance")

    # guard degenerate extents (a dead-straight or single-point path makes
    # set_aspect('equal') autoscale to zero height -> matplotlib singular matrix)
    xr = float(traj[:, 0].max() - traj[:, 0].min())
    yr = float(traj[:, 1].max() - traj[:, 1].min())
    if xr < 0.2 or yr < 0.2:
        a.set_aspect("auto")
        a.margins(0.2)

    os.makedirs(RESULTS, exist_ok=True)
    tag = args.tag or ("%s_%s_B%.1f_%.1f" % (args.variant, args.pose, args.bx, args.by))
    png = os.path.join(RESULTS, tag + ".png")
    js = os.path.join(RESULTS, tag + ".json")
    # save the DATA first so a plotting hiccup never loses the run record
    json.dump({"summary": summary, "steps": rec}, open(js, "w"))
    print("path=%.2fm  min_clearance=%.3fm  v_mean=%.2f  w in[%.2f,%.2f]" % (
        plen, min(clr), V.mean(), W.min(), W.max()))
    # gyro vs odom yaw cross-check: cumulative yaw change, both relative to start
    gy = [r.get("gyro_yaw") for r in rec]
    if any(g is not None for g in gy):
        oyaw = np.unwrap(P[:, 2])
        odom_dyaw = np.degrees(oyaw - oyaw[0])
        g0 = next(g for g in gy if g is not None)
        gyaw = np.degrees(np.array([(g - g0) if g is not None else np.nan for g in gy]))
        div = np.nanmax(np.abs(odom_dyaw - gyaw))
        print("YAW cross-check: odom total=%+.1fdeg  gyro total=%+.1fdeg  max|odom-gyro|=%.1fdeg"
              % (odom_dyaw[-1], gyaw[-1], div))
    try:
        fig.savefig(png, dpi=115, bbox_inches="tight")
        print("wrote %s" % png)
    except Exception as e:
        print("plot skipped (%s: %s)" % (type(e).__name__, e))
    print("wrote %s" % js)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", type=str, default="1",
                    help="1 (=mpc_vw), 2 (=mpc_grid), or any registry key "
                         "(mpc_vw_t, mpc_grid_t, your own model)")
    ap.add_argument("--bx", type=float, default=3.0, help="goal B forward x (m)")
    ap.add_argument("--by", type=float, default=0.0, help="goal B left y (m)")
    ap.add_argument("--pose", default="odom", choices=["odom", "lidar", "dead_reckon"])
    ap.add_argument("--mag", type=float, default=40.0)
    ap.add_argument("--tol", type=float, default=0.15)
    ap.add_argument("--extra-margin", type=float, default=None,
                    help="override CostConfig.extra_margin (benchmarked default "
                         "0.10; the planner keeps this much extra clearance)")
    ap.add_argument("--timeout", type=float, default=35.0)
    ap.add_argument("--tick-hz", type=float, default=3.0,
                    help="GLOBAL tick rate; must equal the car's perception rate")
    ap.add_argument("--exec-steps", type=int, default=1,
                    help="planned steps to apply before re-planning (1 = tight closed loop)")
    ap.add_argument("--guard", dest="guard", action="store_true", default=True,
                    help="collision guard (ON by default, like the GUI, since the "
                         "2026-07-25 penetration incident)")
    ap.add_argument("--no-guard", dest="guard", action="store_false",
                    help="disable the collision guard for this run (the soft-stop "
                         "near obstacles; only for controlled experiments)")
    ap.add_argument("--tag", default=None, help="output filename tag")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    out = run(args)
    if out:
        rec, summary = out
        if rec:
            plot(rec, summary, args)


if __name__ == "__main__":
    main()
