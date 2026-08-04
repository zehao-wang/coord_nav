#!/usr/bin/env python3
"""Benchmark policies over the simulator suite and compare them.

    python scripts/benchmark.py                       # the README table: 20 seeds x suite
    python scripts/benchmark.py --seeds 1             # quick single-seed look
    python scripts/benchmark.py --suite realistic     # the realistic-regime suite (B=3m)
    python scripts/benchmark.py --json out.json       # also dump metrics
    python scripts/benchmark.py --plots plotdir/      # per-scenario path plots (seed 0)
    python scripts/benchmark.py --policy my_model     # add YOUR registered model
    python scripts/benchmark.py --policy my_model --no-builtins   # only yours

Variant 2 (discrete grid-hop) is the default baseline the car is tested with;
variant 1 (continuous v,omega sampling MPC) is the continuous comparison. Any
policy registered in mpc_baseline/registry.py can join the table via --policy
(repeatable). Comparability: --variant defaults to the SIM profile (v_max 0.22)
while --policy always builds the LIVE profile; with --live-profile both use
LiveConfig.magnitude = 40 and the numbers are directly comparable.

--seeds defaults to 20 so the shipped command reproduces the README's
20-seeds-x-suite protocol as-is: this script used to run seed 0 only, printing
exactly the single-seed numbers the README warns against, and the published
table needed a hand-written loop nobody else had.
"""

import os
import sys
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from mpc_baseline import eval as E
from mpc_baseline.registry import POLICY_REGISTRY
from mpc_baseline.sim import default_scenarios, realistic_scenarios, goal_from_start


def _plot_all(results_by_variant, scenarios, plotdir, goal_dist):
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
        # the ACTUAL goal, not a hardcoded 1 m: --goal-dist 3 used to draw the
        # star at 1 m while trajectories continued to 3 m, misgrading every plot
        goal = goal_from_start(world.start, goal_dist)
        ax.plot(0, 0, "ks"); ax.plot(goal[0], goal[1], "g*", ms=15)
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3); ax.legend()
        ax.set_title(world.name)
        p = os.path.join(plotdir, "%02d_%s.png" % (i, world.name))
        fig.savefig(p, dpi=110, bbox_inches="tight"); plt.close(fig)
    print("wrote plots to %s" % plotdir)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--goal-dist", type=float, default=None,
                    help="B forward distance (m); default 1.0 for the tight suite, "
                         "3.0 for --suite realistic")
    ap.add_argument("--suite", choices=("tight", "realistic"), default="tight",
                    help="tight = the built-in stress suite (obstacles at 0.4-0.7 m, "
                         "some beyond the car's turn radius); realistic = the regime "
                         "the car actually drives (B=3 m, obstacles 1.2-1.5 m)")
    ap.add_argument("--seeds", type=int, default=20,
                    help="run seeds 0..N-1 and aggregate (default 20 = the README "
                         "protocol; per-scenario rows and plots show seed 0)")
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
    ap.add_argument("--disturbed", action="store_true",
                    help="LIVE-FAITHFUL evaluation: measured execution disturbance "
                         "(yaw lag tau 0.48 s + speed noise, fit from 298 on-car "
                         "ticks) plus the runner's buffered tick loop. Screen any "
                         "smoothness/robustness tuning with this flag ON.")
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

    if args.seeds < 1:
        print("--seeds must be >= 1")
        sys.exit(2)
    goal_dist = args.goal_dist if args.goal_dist is not None else (
        3.0 if args.suite == "realistic" else 1.0)
    scenarios = (realistic_scenarios(goal_dist) if args.suite == "realistic"
                 else default_scenarios(goal_dist))

    by = {}          # label -> episodes across ALL seeds (what the aggregate is over)
    seed0 = {}       # label -> seed-0 episodes (per-scenario rows + plots)
    for seed in range(args.seeds):
        if not args.no_builtins:
            for label, variant in (("v2-grid", 2), ("v1-vw", 1)):
                rs = E.run_variant(variant, scenarios, live=args.live_profile,
                                   goal_dist=goal_dist, plan_dt=args.plan_dt,
                                   disturbed=args.disturbed, seed=seed)
                by.setdefault(label, []).extend(rs)
                seed0.setdefault(label, rs)
        for key in args.policy:
            rs = E.run_policy(key, scenarios, goal_dist=goal_dist,
                              magnitude=args.magnitude, plan_dt=args.plan_dt,
                              disturbed=args.disturbed, seed=seed)
            by.setdefault(key, []).extend(rs)
            seed0.setdefault(key, rs)

    if args.seeds == 1:
        E.print_table(by)
    else:
        print("\n[seed 0 of %d]" % args.seeds)
        E.print_table(seed0, scenario_rows=True, aggregate=False)
        print("\n=== Aggregate over %d seeds x %d scenarios = %d episodes ==="
              % (args.seeds, len(scenarios), args.seeds * len(scenarios)))
        E.print_table(by, scenario_rows=False)
    if args.json:
        E.to_json(by, args.json)
        print("\nwrote %s" % args.json)
    if args.plots:
        _plot_all(seed0, scenarios, args.plots, goal_dist)


if __name__ == "__main__":
    main()
