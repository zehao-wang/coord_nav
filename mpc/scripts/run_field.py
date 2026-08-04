#!/usr/bin/env python3
"""The TestField CLI: evaluate ANY registered policy in the live-faithful field
(dynamic pymunk obstacles + occlusion + measured execution disturbance).

    python scripts/run_field.py                          # both baselines, full battery
    python scripts/run_field.py --policy my_model        # plug YOUR model in
    python scripts/run_field.py --anim /tmp/anims        # per-case animations (seed 0)
    python scripts/run_field.py --perfect-exec           # idealized execution world
    python scripts/run_field.py --mem 0                  # current-frame-only memory

LIVE-FAITHFUL BY DEFAULT: unlike benchmark.py (whose default is the perfect
execution world), the field runs the measured execution disturbance and the
buffered tick loop unless you pass --perfect-exec -- the field's whole point is
that the policy's inputs and execution match the real car. The battery is 5
named archetypes (crossers, oncoming, an occluded crossing) + seeded random
cases; --seeds 20 x 15 cases = 300 episodes per policy by default.
"""

import os
import sys
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from mpc_baseline import config as C
from mpc_baseline import eval as E
from mpc_baseline import testfield as TF
from mpc_baseline.registry import POLICY_REGISTRY


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policy", action="append", default=[],
                    help="registry key to evaluate (repeatable). Default: the two "
                         "baselines. Choices: %s" % ", ".join(sorted(POLICY_REGISTRY)))
    ap.add_argument("--only", action="store_true",
                    help="run ONLY the --policy entries (skip the two baselines)")
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--goal-dist", type=float, default=3.0)
    ap.add_argument("--random", type=int, default=10,
                    help="number of seeded random cases (default 10)")
    ap.add_argument("--rand-seed", type=int, default=1000,
                    help="seed for the random-case GENERATOR (fixed by default so "
                         "everyone runs the same battery; change to get new worlds)")
    ap.add_argument("--no-archetypes", action="store_true")
    ap.add_argument("--perfect-exec", action="store_true",
                    help="disable the measured execution disturbance + buffered "
                         "loop (NOT live-faithful; for debugging planners only)")
    ap.add_argument("--no-occlusion", action="store_true")
    ap.add_argument("--noise-xy", type=float, default=0.0,
                    help="perception centre noise std, m (knob, unfitted)")
    ap.add_argument("--dropout", type=float, default=0.0,
                    help="per-obstacle miss probability (knob, unfitted)")
    ap.add_argument("--mem", type=float, default=None,
                    help="override ObstacleConfig.mem_time_s (e.g. 0 = plan on the "
                         "current frame only -- quantifies the memory-trail cost)")
    ap.add_argument("--magnitude", type=float, default=40.0)
    ap.add_argument("--anim", default=None,
                    help="directory: render seed-0 animations of every case for "
                         "every policy")
    ap.add_argument("--anim-fmt", choices=("gif", "mp4"), default="gif")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    unknown = [p for p in args.policy if p not in POLICY_REGISTRY]
    if unknown:
        print("unknown policy %s; registered: %s"
              % (", ".join(map(repr, unknown)), ", ".join(sorted(POLICY_REGISTRY))))
        sys.exit(2)
    if args.only and not args.policy:
        print("--only needs at least one --policy")
        sys.exit(2)
    if args.seeds < 1:
        print("--seeds must be >= 1")
        sys.exit(2)
    if args.goal_dist < 2.0:
        print("--goal-dist must be >= 2.0 (the field's cases assume the "
              "realistic regime; short tight worlds are benchmark.py's suites)")
        sys.exit(2)

    # dict.fromkeys: de-duplicate while keeping order (--policy mpc_grid used to
    # silently run the whole battery for mpc_grid twice)
    specs = list(dict.fromkeys(
        ([] if args.only else ["mpc_grid", "mpc_vw"]) + args.policy))
    cases = ([] if args.no_archetypes else TF.archetype_cases(args.goal_dist)) + \
        TF.random_cases(args.random, seed=args.rand_seed, goal_dist=args.goal_dist)
    if not cases:
        print("no cases (--no-archetypes with --random 0)")
        sys.exit(2)
    perception = TF.PerceptionConfig(occlusion=not args.no_occlusion,
                                     noise_xy=args.noise_xy, dropout=args.dropout)
    obs_cfg = None
    if args.mem is not None:
        obs_cfg = C.ObstacleConfig(mem_time_s=args.mem)
    disturbed = not args.perfect_exec

    by = TF.run_field(specs, cases, seeds=args.seeds, goal_dist=args.goal_dist,
                      disturbed=disturbed, perception=perception, obs_cfg=obs_cfg,
                      magnitude=args.magnitude)

    n = len(cases)
    seed0 = {k: rs[:n] for k, rs in by.items()}
    mode = "PERFECT EXECUTION (not live-faithful)" if args.perfect_exec \
        else "live-faithful (disturbed + buffered)"
    print("\nTestField: %s | occlusion=%s | %d cases x %d seeds"
          % (mode, not args.no_occlusion, n, args.seeds))
    if args.seeds == 1:
        E.print_table(by)
    else:
        print("\n[seed 0 of %d]" % args.seeds)
        E.print_table(seed0, scenario_rows=True, aggregate=False)
        print("\n=== Aggregate over %d episodes per policy ===" % (n * args.seeds))
        E.print_table(by, scenario_rows=False)
    if args.json:
        E.to_json(by, args.json)
        print("\nwrote %s" % args.json)
    if args.anim:
        try:
            import matplotlib  # noqa: F401 -- optional [plot] extra
        except Exception as e:
            print("animations skipped (matplotlib unavailable): %s" % e)
            return
        os.makedirs(args.anim, exist_ok=True)
        for spec in specs:
            for i, case in enumerate(cases):
                p = os.path.join(args.anim, "%02d_%s_%s.%s"
                                 % (i, case.name, spec, args.anim_fmt))
                # obs_cfg MUST ride along: without it a --mem run scored one
                # episode and animated a different one under the same label
                TF.animate_case(spec, case, p, seed=0, goal_dist=args.goal_dist,
                                disturbed=disturbed, perception=perception,
                                magnitude=args.magnitude, obs_cfg=obs_cfg)
        print("wrote animations to %s" % args.anim)


if __name__ == "__main__":
    main()
