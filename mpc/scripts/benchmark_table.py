#!/usr/bin/env python3
"""Maintain the FIXED benchmark table (BENCHMARKS.md / benchmarks.json).

    python scripts/benchmark_table.py                     # all four baselines
    python scripts/benchmark_table.py --policy my_model   # add/update one row
    python scripts/benchmark_table.py --check mpc_grid_t  # stored row reproduces?

One committed, same-seed, difficulty-tiered protocol (L1 static-open ..
L4 dyn-complex, live-faithful everywhere) -- every current and future policy is
evaluated here. See mpc_baseline/benchtable.py for the protocol definition.
"""

import os
import sys
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from mpc_baseline import benchtable as B
from mpc_baseline.registry import POLICY_REGISTRY

BASELINES = ("mpc_grid", "mpc_vw", "mpc_grid_t", "mpc_vw_t")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policy", action="append", default=[],
                    help="registry key to (re)benchmark (repeatable; default: "
                         "the four baselines). Choices: %s"
                         % ", ".join(sorted(POLICY_REGISTRY)))
    ap.add_argument("--check", default=None, metavar="KEY",
                    help="re-run KEY and verify its stored row reproduces "
                         "exactly (exit 1 on any diff)")
    ap.add_argument("--anim", default=None, metavar="DIR",
                    help="render the seed-0 episode of every eval-set case for "
                         "the chosen policies (files ordered L1..L4) INSTEAD of "
                         "updating the table -- the table's visual companion")
    ap.add_argument("--anim-fmt", choices=("mp4", "gif"), default="mp4")
    args = ap.parse_args()

    unknown = [p for p in args.policy + ([args.check] if args.check else [])
               if p not in POLICY_REGISTRY]
    if unknown:
        print("unknown policy %s; registered: %s"
              % (", ".join(map(repr, unknown)), ", ".join(sorted(POLICY_REGISTRY))))
        sys.exit(2)

    if args.check:
        diffs = B.check(args.check)
        if diffs:
            print("MISMATCH -- the stored row does not reproduce:")
            for d in diffs:
                print("  " + d)
            sys.exit(1)
        print("OK: %s reproduces its stored row exactly (protocol v%d)"
              % (args.check, B.PROTOCOL_VERSION))
        return

    keys = args.policy or list(BASELINES)
    if args.anim:
        B.animate_set(keys, args.anim, fmt=args.anim_fmt)
        print("\nwrote animations to %s" % args.anim)
        return
    B.update(keys)
    print("\nwrote %s and %s" % (B.JSON_PATH, B.MD_PATH))


if __name__ == "__main__":
    main()
