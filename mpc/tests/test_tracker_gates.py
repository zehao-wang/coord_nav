"""Perception-hardening gates vs REAL recorded frames (the committed fixture).

The property being pinned forever: on real all-static scenes the tracker's
gated velocities stay (near-)silent -- the failure that made the *_t variants
unsafe to deploy was 95% of ticks carrying phantom velocities. And the gates
must not have destroyed acquisition: a noise-matched injected mover at
pedestrian speed is still caught within a few seconds.
"""

import json
import os

import numpy as np
import pytest

from mpc_baseline import config as C
from mpc_baseline.obstacles import ObstacleField

FIXTURE = os.path.join(os.path.dirname(__file__), "data", "real_frames.json")


def _runs():
    d = json.load(open(FIXTURE))
    return [(r["name"], [(s["t"], s["circles"], s["pose"]) for s in r["steps"]])
            for r in d["runs"]]


def test_real_static_frames_produce_no_phantom_velocities():
    total = 0
    for _, frames in _runs():
        clk = {"t": 0.0}
        field = ObstacleField(C.ObstacleConfig(), lambda: clk["t"])
        for t, circles, pose in frames:
            clk["t"] = t
            field.update(circles, pose)
            v = field.velocities()
            if len(v):
                total += int((np.hypot(v[:, 0], v[:, 1]) > 0).sum())
    # 1 residual event on 56 ticks (possibly the operator moving in-room);
    # the pre-gate tracker produced ~600 on these frames
    assert total <= 1


def test_injected_mover_still_acquired_through_real_clutter():
    rng = np.random.default_rng(7)
    name, frames = _runs()[0]
    px0, py0, _ = frames[0][2]
    speed, heading = 0.3, np.pi / 2
    clk = {"t": 0.0}
    field = ObstacleField(C.ObstacleConfig(), lambda: clk["t"])
    lat, first_t = None, None
    for t, circles, pose in frames:
        clk["t"] = t
        mx = px0 + 1.8
        my = py0 - speed * (6.0 - t)
        cc = list(circles)
        if rng.random() >= 0.28:                      # measured dropout
            px, py, pth = pose
            dx = mx + rng.normal(0, 0.02) - px
            dy = my + rng.normal(0, 0.02) - py
            cth, sth = np.cos(-pth), np.sin(-pth)
            cc.append((cth * dx - sth * dy, sth * dx + cth * dy, 0.14))
        if first_t is None:
            first_t = t
        field.update(cc, pose)
        v, c = field.velocities(), field.circles()
        for i in range(len(v)):
            if np.hypot(c[i, 0] - mx, c[i, 1] - my) < 0.3:
                sp = np.hypot(v[i, 0], v[i, 1])
                if sp > 0 and abs(sp - speed) <= 0.09 and lat is None:
                    lat = t - first_t
    assert lat is not None and lat <= 4.0
