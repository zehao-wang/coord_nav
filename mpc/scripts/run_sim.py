#!/usr/bin/env python3
"""Run a policy in the offline simulator and (optionally) plot the path.

    python scripts/run_sim.py --variant 2 --scenario dead_ahead
    python scripts/run_sim.py --variant 1 --scenario slalom --plot out.png
    python scripts/run_sim.py --variant 2 --all               # whole suite table
    python scripts/run_sim.py --policy my_model --all         # YOUR registered model
    python scripts/run_sim.py --policy mpc_vw --plan-dt 0.25  # at the live cadence

--variant takes the two built-ins (1|2) with the sim profile; --policy takes any
key in POLICY_REGISTRY (see mpc_baseline/registry.py) with the live profile.

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
from mpc_baseline.registry import POLICY_REGISTRY, build_policy
from mpc_baseline import eval as E


def _run_one(variant, world, goal_dist, live, plan_dt=None):
    cfg = (C.live_config_v1() if live else C.sim_config_v1()) if variant == 1 \
        else (C.live_config_v2() if live else C.sim_config_v2())
    cfg.goal.goal_dist = goal_dist
    sim = KinematicSim(world, robot_radius=cfg.robot.robot_radius)
    policy = Variant1Policy(cfg) if variant == 1 else Variant2Policy(cfg)
    dt = plan_dt if plan_dt is not None else \
        (cfg.mppi.dt if variant == 1 else cfg.step_duration)
    res = run_episode(sim, policy, variant, C.ObstacleConfig(), cfg.goal,
                      plan_dt=dt, robot_cfg=cfg.robot)
    return res, cfg


def _run_one_policy(key, world, goal_dist, magnitude, plan_dt=None):
    """Same, for any REGISTERED policy (your own model included)."""
    policy, cfg = build_policy(key, magnitude, goal_dist)
    cfg.goal.goal_dist = goal_dist
    sim = KinematicSim(world, robot_radius=cfg.robot.robot_radius)
    dt = plan_dt if plan_dt is not None else E._default_plan_dt(cfg)
    res = run_episode(sim, policy, key, C.ObstacleConfig(), cfg.goal,
                      plan_dt=dt, robot_cfg=cfg.robot)
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
    ap.add_argument("--variant", type=int, default=None, choices=[1, 2],
                    help="built-in variant, sim profile (default 2 if --policy is unset)")
    ap.add_argument("--policy", default=None,
                    help="registry key instead of --variant, e.g. mpc_vw / your own "
                         "model (live profile). Choices: %s" % ", ".join(sorted(POLICY_REGISTRY)))
    ap.add_argument("--magnitude", type=float, default=40.0,
                    help="PWM magnitude for --policy (default 40; 20 cannot steer)")
    ap.add_argument("--plan-dt", type=float, default=None,
                    help="override the control period (live runner uses 0.25 s)")
    ap.add_argument("--scenario", default="dead_ahead")
    ap.add_argument("--goal-dist", type=float, default=1.0)
    ap.add_argument("--live-profile", action="store_true",
                    help="use the slow live config instead of the sim config")
    ap.add_argument("--all", action="store_true", help="run the whole suite")
    ap.add_argument("--plot", default=None, help="save a path plot to this PNG")
    args = ap.parse_args()

    if args.policy is not None and args.variant is not None:
        print("give --variant OR --policy, not both")
        sys.exit(2)
    if args.policy is not None and args.policy not in POLICY_REGISTRY:
        print("unknown policy %r; registered: %s"
              % (args.policy, ", ".join(sorted(POLICY_REGISTRY))))
        sys.exit(2)

    scenarios = default_scenarios(args.goal_dist)
    if args.all:
        if args.policy is not None:
            rs = E.run_policy(args.policy, scenarios, goal_dist=args.goal_dist,
                              magnitude=args.magnitude, plan_dt=args.plan_dt)
            E.print_table({args.policy: rs})
            return
        r1 = E.run_variant(1, scenarios, live=args.live_profile,
                           goal_dist=args.goal_dist, plan_dt=args.plan_dt)
        r2 = E.run_variant(2, scenarios, live=args.live_profile,
                           goal_dist=args.goal_dist, plan_dt=args.plan_dt)
        E.print_table({"v2-grid": r2, "v1-vw": r1})
        return

    match = [w for w in scenarios if w.name == args.scenario]
    if not match:
        print("unknown scenario %r; choices: %s" %
              (args.scenario, ", ".join(w.name for w in scenarios)))
        sys.exit(2)
    world = match[0]
    if args.policy is not None:
        label = args.policy
        res, cfg = _run_one_policy(args.policy, world, args.goal_dist,
                                   args.magnitude, args.plan_dt)
    else:
        label = "variant %d" % (args.variant if args.variant is not None else 2)
        res, cfg = _run_one(args.variant if args.variant is not None else 2, world,
                            args.goal_dist, args.live_profile, args.plan_dt)
    goal = goal_from_start(world.start, args.goal_dist)
    print("scenario=%s policy=%s reached=%s collided=%s time=%.1fs steps=%d "
          "path_len=%.2f min_clearance=%.3f final_gd=%.3f" % (
              world.name, label, res.reached, res.collided, res.sim_time,
              res.steps, res.path_length, res.min_clearance, res.final_goal_dist))
    if args.plot:
        _plot(res, world, goal, args.plot)


if __name__ == "__main__":
    main()
