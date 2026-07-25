#!/usr/bin/env python3
"""Run a variant in the offline simulator and (optionally) plot the path.

    python scripts/run_sim.py --variant 2 --scenario dead_ahead
    python scripts/run_sim.py --variant 1 --scenario slalom --plot out.png
    python scripts/run_sim.py --variant 2 --all            # whole suite table

No ROS / no car needed -- this exercises the exact planning code that runs live.
"""

import os
import sys
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import numpy as np
from mpc_baseline import config as C
from mpc_baseline.sim import KinematicSim, run_episode, default_scenarios, goal_from_start
from mpc_baseline.policies import Variant1Policy, Variant2Policy
from mpc_baseline import eval as E


def _run_one(variant, world, goal_dist, live):
    cfg = (C.live_config_v1() if live else C.sim_config_v1()) if variant == 1 \
        else (C.live_config_v2() if live else C.sim_config_v2())
    cfg.goal.goal_dist = goal_dist
    sim = KinematicSim(world, robot_radius=cfg.robot.robot_radius)
    policy = Variant1Policy(cfg) if variant == 1 else Variant2Policy(cfg)
    plan_dt = cfg.mppi.dt if variant == 1 else cfg.step_duration
    res = run_episode(sim, policy, variant, C.ObstacleConfig(), cfg.goal,
                      plan_dt=plan_dt)
    return res, cfg


def _plot(res, world, goal, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print("plot skipped (matplotlib unavailable): %s" % e)
        return
    fig, ax = plt.subplots(figsize=(6, 6))
    for (x, y, r) in world.circles:
        ax.add_patch(plt.Circle((x, y), r, color="tab:red", alpha=0.5))
    tr = res.traj
    ax.plot(tr[:, 0], tr[:, 1], "-o", ms=2, color="tab:blue", label="path")
    ax.plot(0, 0, "ks", label="start")
    ax.plot(goal[0], goal[1], "g*", ms=15, label="goal B")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3); ax.legend()
    ax.set_title("%s  variant=%s  reached=%s coll=%s t=%.1fs" % (
        world.name, res.variant, res.reached, res.collided, res.sim_time))
    fig.savefig(path, dpi=110, bbox_inches="tight")
    print("wrote %s" % path)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--variant", type=int, default=2, choices=[1, 2])
    ap.add_argument("--scenario", default="dead_ahead")
    ap.add_argument("--goal-dist", type=float, default=1.0)
    ap.add_argument("--live-profile", action="store_true",
                    help="use the slow live config instead of the sim config")
    ap.add_argument("--all", action="store_true", help="run the whole suite (both variants)")
    ap.add_argument("--plot", default=None, help="save a path plot to this PNG")
    args = ap.parse_args()

    scenarios = default_scenarios(args.goal_dist)
    if args.all:
        r1 = E.run_variant(1, scenarios, live=args.live_profile, goal_dist=args.goal_dist)
        r2 = E.run_variant(2, scenarios, live=args.live_profile, goal_dist=args.goal_dist)
        E.print_table({"v2-grid": r2, "v1-vw": r1})
        return

    match = [w for w in scenarios if w.name == args.scenario]
    if not match:
        print("unknown scenario %r; choices: %s" %
              (args.scenario, ", ".join(w.name for w in scenarios)))
        sys.exit(2)
    world = match[0]
    res, cfg = _run_one(args.variant, world, args.goal_dist, args.live_profile)
    goal = goal_from_start(world.start, args.goal_dist)
    print("scenario=%s variant=%d reached=%s collided=%s time=%.1fs steps=%d "
          "path_len=%.2f min_clearance=%.3f final_gd=%.3f" % (
              world.name, args.variant, res.reached, res.collided, res.sim_time,
              res.steps, res.path_length, res.min_clearance, res.final_goal_dist))
    if args.plot:
        _plot(res, world, goal, args.plot)


if __name__ == "__main__":
    main()
