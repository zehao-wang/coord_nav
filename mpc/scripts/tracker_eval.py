#!/usr/bin/env python3
"""Tracker eval: does the CV tracker stay silent on real static scenes and
still catch movers? The harness the 0.9.20 perception-hardening gates were
tuned on -- rerun it whenever the tracker or its gates change.

    python scripts/tracker_eval.py                 # committed fixture (56 real ticks)
    python scripts/tracker_eval.py --logs 'output/2026-08-*/run.json'   # full recordings

Two parts, both through the REAL ObstacleField (never a reimplementation):

  negatives  replay recorded all-static frames; every gated velocity is a
             phantom. Before the gates: phantoms on 95% of ticks (wall-cluster
             centroid slide + association churn). After: -97%.
  positives  inject a synthetic mover into the SAME frames with the MEASURED
             perception noise (sigma 0.02 m centres, 0.01 m radius, 28%
             dropout -- fitted from calm tracks in the recordings), so
             acquisition rate/latency are evaluated under real clutter.

Caveat recorded 2026-08-05: the recordings were made with the operator in the
room, so a residual "phantom" at walking speed may be a real mover -- the
fixture uses the two cleanest runs (1 and 0 events).
"""

import os
import sys
import glob
import json
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import numpy as np

from mpc_baseline import config as C
from mpc_baseline.obstacles import ObstacleField

FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "tests", "data", "real_frames.json")
NOISE_XY = 0.02          # measured (calm-track jitter p50/sqrt2 = 0.013, rounded up)
NOISE_R = 0.01
DROPOUT = 0.28           # measured near-track missed-update rate


def load_runs(logs_glob=None):
    if logs_glob:
        runs = []
        for path in sorted(glob.glob(logs_glob)):
            d = json.load(open(path))
            steps = [s for s in d["steps"] if s.get("circles") and s.get("pose")]
            if len(steps) < 5:
                continue
            t0 = steps[0]["t"]
            runs.append((os.path.basename(os.path.dirname(path)),
                         [(s["t"] - t0, s["circles"], s["pose"]) for s in steps]))
        return runs
    d = json.load(open(FIXTURE))
    return [(r["name"], [(s["t"], s["circles"], s["pose"]) for s in r["steps"]])
            for r in d["runs"]]


def phantom_events(frames):
    clk = {"t": 0.0}
    field = ObstacleField(C.ObstacleConfig(), lambda: clk["t"])
    n = 0
    for t, circles, pose in frames:
        clk["t"] = t
        field.update(circles, pose)
        v = field.velocities()
        if len(v):
            n += int((np.hypot(v[:, 0], v[:, 1]) > 0).sum())
    return n


def injected_acquisition(frames, speed, heading, rng):
    px0, py0, _ = frames[0][2]
    clk = {"t": 0.0}
    field = ObstacleField(C.ObstacleConfig(), lambda: clk["t"])
    first_t = None
    for t, circles, pose in frames:
        clk["t"] = t
        mx = px0 + 1.8 - speed * np.cos(heading) * (6.0 - t)
        my = py0 - speed * np.sin(heading) * (6.0 - t)
        cc = list(circles)
        if rng.random() >= DROPOUT:
            px, py, pth = pose
            dx = mx + rng.normal(0, NOISE_XY) - px
            dy = my + rng.normal(0, NOISE_XY) - py
            cth, sth = np.cos(-pth), np.sin(-pth)
            cc.append((cth * dx - sth * dy, sth * dx + cth * dy,
                       0.14 + abs(rng.normal(0, NOISE_R))))
        if first_t is None:
            first_t = t
        field.update(cc, pose)
        v, c = field.velocities(), field.circles()
        for i in range(len(v)):
            if np.hypot(c[i, 0] - mx, c[i, 1] - my) < 0.3:
                sp = np.hypot(v[i, 0], v[i, 1])
                if sp > 0 and abs(sp - speed) <= max(0.08, 0.3 * speed):
                    return t - first_t
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--logs", default=None,
                    help="glob of run.json recordings (default: the committed "
                         "fixture)")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    runs = load_runs(args.logs)
    print("== negatives: phantom gated-velocity events on all-static frames ==")
    total_t = total_p = 0
    for name, frames in runs:
        p = phantom_events(frames)
        total_t += len(frames)
        total_p += p
        print("  %-30s %3d ticks   %3d events" % (name, len(frames), p))
    print("TOTAL %d ticks, %d events" % (total_t, total_p))

    print("\n== positives: injected movers (measured noise, %d%% dropout) ==" %
          round(DROPOUT * 100))
    for speed in (0.25, 0.35, 0.45):
        # one INDEPENDENT rng per episode: a shared stream made every number
        # depend on evaluation order (adding a speed changed the others)
        outs = [injected_acquisition(frames, speed, h,
                                     np.random.default_rng(
                                         [args.seed, int(speed * 100), ri, hi]))
                for ri, (_, frames) in enumerate(runs)
                for hi, h in enumerate((np.pi / 2, np.pi, np.pi / 4))]
        got = [x for x in outs if x is not None]
        print("  %.2f m/s: acquired %d/%d  median latency %s" % (
            speed, len(got), len(outs),
            "-" if not got else "%.1f s" % float(np.median(got))))


if __name__ == "__main__":
    main()
