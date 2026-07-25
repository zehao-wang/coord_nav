#!/usr/bin/env python3
"""Benchmark both variants over the simulator suite and compare them.

    python scripts/benchmark.py                     # table for both variants
    python scripts/benchmark.py --json out.json     # also dump metrics
    python scripts/benchmark.py --plots plotdir/    # per-scenario path plots

Variant 2 (discrete grid-hop) is the default baseline the car is tested with;
variant 1 (continuous v,omega sampling MPC) is the continuous comparison.
"""

import os
import sys
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from mpc_baseline import eval as E
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
    ap.add_argument("--json", default=None)
    ap.add_argument("--plots", default=None)
    args = ap.parse_args()

    scenarios = default_scenarios(args.goal_dist)
    r2 = E.run_variant(2, scenarios, live=args.live_profile, goal_dist=args.goal_dist)
    r1 = E.run_variant(1, scenarios, live=args.live_profile, goal_dist=args.goal_dist)
    by = {"v2-grid": r2, "v1-vw": r1}
    E.print_table(by)
    if args.json:
        E.to_json(by, args.json)
        print("\nwrote %s" % args.json)
    if args.plots:
        _plot_all(by, scenarios, args.plots)


if __name__ == "__main__":
    main()
