#!/usr/bin/env python3
"""Benchmark policies over the simulator suite and compare them.

    python scripts/benchmark.py                       # table for both variants
    python scripts/benchmark.py --json out.json       # also dump metrics
    python scripts/benchmark.py --plots plotdir/      # per-scenario path plots
    python scripts/benchmark.py --policy my_model     # add YOUR registered model
    python scripts/benchmark.py --policy my_model --no-builtins   # only yours

Variant 2 (discrete grid-hop) is the default baseline the car is tested with;
variant 1 (continuous v,omega sampling MPC) is the continuous comparison. Any
policy registered in mpc_baseline/registry.py can join the table via --policy
(repeatable). NOTE the built-ins and --policy entries are NOT directly comparable:
--variant uses the sim profile, and even with --live-profile it takes magnitude from
LiveConfig.magnitude (20 -> v_max 0.10) while --policy builds through the registry at
--magnitude (default 40 -> v_max 0.20), a 2.00x speed difference. Match them with
--live-profile --magnitude 20.
"""

import os
import sys
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from mpc_baseline import eval as E
from mpc_baseline.registry import POLICY_REGISTRY
from mpc_baseline.sim import default_scenarios, goal_from_start


def _plot_all(results_by_variant, scenarios, plotdir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print("plots skipped (matplotlib unavailable): %s" % e)
        return
    os.makedirs(plotdir, exist_ok=True)
    for i, world in enumerate(scenarios):
        fig, ax = plt.subplots(figsize=(6, 6))
        for (x, y, r) in world.circles:
            ax.add_patch(plt.Circle((x, y), r, color="tab:red", alpha=0.4))
        for label, rs in results_by_variant.items():
            tr = rs[i].traj
            ax.plot(tr[:, 0], tr[:, 1], "-o", ms=2, label="%s (%s)" % (
                label, "R" if rs[i].reached else ("C" if rs[i].collided else "x")))
        goal = goal_from_start(world.start, 1.0)
        ax.plot(0, 0, "ks"); ax.plot(goal[0], goal[1], "g*", ms=15)
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3); ax.legend()
        ax.set_title(world.name)
        p = os.path.join(plotdir, "%02d_%s.png" % (i, world.name))
        fig.savefig(p, dpi=110, bbox_inches="tight"); plt.close(fig)
    print("wrote plots to %s" % plotdir)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--goal-dist", type=float, default=1.0)
    ap.add_argument("--live-profile", action="store_true")
    ap.add_argument("--policy", action="append", default=[],
                    help="registry key to add to the table (repeatable). Choices: %s"
                         % ", ".join(sorted(POLICY_REGISTRY)))
    ap.add_argument("--no-builtins", action="store_true",
                    help="skip variants 1/2, only run the --policy entries")
    ap.add_argument("--magnitude", type=float, default=40.0,
                    help="PWM magnitude for --policy entries (default 40)")
    ap.add_argument("--plan-dt", type=float, default=None,
                    help="override the control period, VELOCITY policies only (a discrete "
                         "policy runs each action for its own step_duration)")
    ap.add_argument("--json", default=None)
    ap.add_argument("--plots", default=None)
    args = ap.parse_args()

    unknown = [p for p in args.policy if p not in POLICY_REGISTRY]
    if unknown:
        print("unknown policy %s; registered: %s"
              % (", ".join(map(repr, unknown)), ", ".join(sorted(POLICY_REGISTRY))))
        sys.exit(2)
    if args.no_builtins and not args.policy:
        print("--no-builtins needs at least one --policy")
        sys.exit(2)

    scenarios = default_scenarios(args.goal_dist)
    by = {}
    if not args.no_builtins:
        by["v2-grid"] = E.run_variant(2, scenarios, live=args.live_profile,
                                      goal_dist=args.goal_dist, plan_dt=args.plan_dt)
        by["v1-vw"] = E.run_variant(1, scenarios, live=args.live_profile,
                                    goal_dist=args.goal_dist, plan_dt=args.plan_dt)
    for key in args.policy:
        by[key] = E.run_policy(key, scenarios, goal_dist=args.goal_dist,
                               magnitude=args.magnitude, plan_dt=args.plan_dt)
    E.print_table(by)
    if args.json:
        E.to_json(by, args.json)
        print("\nwrote %s" % args.json)
    if args.plots:
        _plot_all(by, scenarios, args.plots)


if __name__ == "__main__":
    main()
