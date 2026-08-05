"""Perception-hardening gates vs REAL recorded frames + the jump/split
mechanisms that real clustering produces.

The properties pinned forever: on real all-static scenes the tracker's gated
velocities stay (near-)silent; observation jumps and cluster splits can reduce
or silence a velocity estimate but never LIE about it; and a noise-matched
injected mover at pedestrian speed is still caught within a few seconds. The
injection/loader logic is IMPORTED from scripts/tracker_eval.py -- the tuning
harness and the pinned tests must be the same code, or they drift apart and
validate different things.
"""

import os
import sys

import numpy as np
import pytest

from mpc_baseline import config as C
from mpc_baseline.obstacles import ObstacleField

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import tracker_eval as TE  # noqa: E402

TICK = 1.0 / 3.0


def test_real_static_frames_produce_no_phantom_velocities():
    total = sum(TE.phantom_events(frames) for _, frames in TE.load_runs())
    # 2 residual events on 56 ticks, both in the run recorded with the operator
    # in the room (possibly true motion); the pre-gate tracker produced ~600
    # on these frames. A rise here means a gate regressed.
    assert total <= 2


def test_injected_mover_still_acquired_through_real_clutter():
    _, frames = TE.load_runs()[0]
    lat = TE.injected_acquisition(frames, 0.3, np.pi / 2,
                                  np.random.default_rng([7, 30, 0, 0]))
    assert lat is not None and lat <= 4.0


def test_static_split_cluster_never_gains_velocity():
    # clustering can return ONE physical object as TWO circles ~merge_dist
    # apart. That geometry used to double-merge into a single track whose
    # "velocity" was the constant cross-centroid vector: coherence exactly
    # 1.0, all gates open, a sustained 0.45 m/s phantom on a STATIC scene.
    # The one-observation-per-track-per-frame invariant turns the split into
    # two adjacent tracks and the isolation gate silences both.
    clk = {"t": 0.0}
    field = ObstacleField(C.ObstacleConfig(), lambda: clk["t"])
    for k in range(30):
        field.update([(2.0, 0.0, 0.10), (2.0, 0.15, 0.10)], (0, 0, 0))
        v = field.velocities()
        assert not len(v) or not np.any(v)
        clk["t"] += TICK


def test_young_boost_pins_fast_mover_acquisition():
    # 0.5 m/s: per-tick displacement 0.167 m > merge_dist 0.15 -- without the
    # young-track association boost the track fragments before a velocity can
    # form. This is the committed evidence that assoc_young_boost is load-
    # bearing (deleting it should fail here, not ship silently).
    def run(boost):
        cfg = C.ObstacleConfig()
        cfg.assoc_young_boost = boost
        clk = {"t": 0.0}
        field = ObstacleField(cfg, lambda: clk["t"])
        y = -1.2
        for k in range(9):
            field.update([(2.0, y, 0.12)], (0, 0, 0))
            clk["t"] += TICK
            y += 0.5 * TICK
        v = field.velocities()
        sp = np.hypot(v[:, 0], v[:, 1]) if len(v) else np.zeros(0)
        return len(field.circles()), float(sp.max()) if len(sp) else 0.0

    n_tracks, speed = run(C.ObstacleConfig().assoc_young_boost)
    assert speed == pytest.approx(0.5, abs=0.06)      # tracked and gated
    n_off, speed_off = run(0.0)
    assert speed_off == 0.0                           # fragments without it


def test_observation_jump_closes_the_gate_instead_of_lying():
    # REAL clustering can make an obstacle's centroid JUMP (leg-split, cluster
    # merge). The required behaviour: a jump must never emit a WRONG gated
    # velocity -- worst case is a same-direction slowdown (degrades toward the
    # frozen-world planner); a direction-flipping sample closes the gate.
    clk = {"t": 0.0}
    field = ObstacleField(C.ObstacleConfig(), lambda: clk["t"])
    y = -1.0
    # steady 0.3 m/s mover, long coherent history -> gate open
    for k in range(8):
        field.update([(2.0, y, 0.12)], (0, 0, 0))
        clk["t"] += TICK
        y += 0.3 * TICK
    v = field.velocities()
    assert np.hypot(v[0, 0], v[0, 1]) == pytest.approx(0.3, abs=0.04)
    # (a) a CANCELLING jump (centroid snaps back by ~one step: raw sample ~ 0):
    # coherent, so it may stay gated -- but only SAME-DIRECTION and SLOWER
    # (max in-range sample error = merge_dist/tick = 0.45 m/s)
    y -= 0.10
    field.update([(2.0, y, 0.12)], (0, 0, 0))
    v = field.velocities()
    assert v[0, 1] >= 0.0
    assert np.hypot(v[0, 0], v[0, 1]) <= 0.3 + 0.05
    # (b) a REVERSING jump (sample direction flips): coherence collapses and
    # the gate CLOSES that tick -- zero, not a wrong velocity
    clk["t"] += TICK
    y += 0.3 * TICK - 0.14
    field.update([(2.0, y, 0.12)], (0, 0, 0))
    v = field.velocities()
    assert v[0, 0] == 0.0 and v[0, 1] == 0.0
    # (c) samples agree again -> the gate re-opens with a CORRECT estimate
    reopened = None
    for k in range(8):
        clk["t"] += TICK
        y += 0.3 * TICK
        field.update([(2.0, y, 0.12)], (0, 0, 0))
        v = field.velocities()
        sp = np.hypot(v[0, 0], v[0, 1])
        if sp > 0:
            assert sp == pytest.approx(0.3, abs=0.09)
            reopened = k
            break
    assert reopened is not None and reopened <= 5
