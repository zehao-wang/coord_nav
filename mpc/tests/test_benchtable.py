"""The fixed benchmark table: set stability, determinism, check() semantics.

Uses a shrunken set (1 seed, minimal randoms) so the suite stays fast; the
full protocol is the same code with bigger constants.
"""

import numpy as np
import pytest

from mpc_baseline import benchtable as B


def _tiny_set():
    return B.eval_set(l3_random=(1, 1003), l4_random=(1, 1004))


def test_eval_set_is_stable_and_tiered():
    a, b = B.eval_set(), B.eval_set()
    # identical on every call -- the set IS the protocol
    assert a["L3_dyn_single"][1] == b["L3_dyn_single"][1]
    assert a["L4_dyn_complex"][1] == b["L4_dyn_complex"][1]
    # tier construction: L3 = single mover, no statics; L4 = clutter
    for case in a["L3_dyn_single"][1]:
        if case.name.startswith("rand"):
            assert len(case.static) == 0 and len(case.dynamic) == 1
    for case in a["L4_dyn_complex"][1]:
        if case.name.startswith("rand"):
            assert len(case.static) >= 1 and len(case.dynamic) >= 1


def test_update_and_check_roundtrip(tmp_path):
    jp, mp = str(tmp_path / "b.json"), str(tmp_path / "b.md")
    stamp = {"commit": "test", "date": "2026-01-01"}
    t1 = B.update(["mpc_grid"], seeds=1, the_set=_tiny_set(), json_path=jp,
                  md_path=mp, stamp=stamp, log=lambda *a: None)
    row = t1["policies"]["mpc_grid"]
    assert set(B.TIERS).issubset(row["cells"])
    assert row["cells"]["overall"]["episodes"] == 4 + 6 + 5 + 2
    # deterministic: a fresh run reproduces the stored row exactly
    assert B.check("mpc_grid", seeds=1, the_set=_tiny_set(), json_path=jp,
                   log=lambda *a: None) == []
    # md regenerated with the row present and no stale marker
    md = open(mp).read()
    assert "`mpc_grid`" in md and "STALE" not in md


def test_static_tier_animation_replay_is_the_scored_episode():
    # animate_set replays static tiers through the field renderer (occlusion
    # off, no movers): the trajectory must be BYTE-IDENTICAL to the episode the
    # table scored through run_policy/KinematicSim -- the video is the episode,
    # not a lookalike
    from mpc_baseline import testfield as TF
    from mpc_baseline.eval import run_policy
    from mpc_baseline.sim import realistic_scenarios
    w = realistic_scenarios(3.0)[0]
    scored = run_policy("mpc_grid", [w], goal_dist=3.0, seed=2,
                        disturbed=True)[0]
    case = TF.Case(w.name, w.start, tuple(tuple(c) for c in w.circles), (),
                   (-0.8, -1.4, 3.8, 1.4))
    replay = TF.run_case("mpc_grid", case, seed=2, goal_dist=3.0,
                         perception=TF.PerceptionConfig(occlusion=False))
    assert np.array_equal(scored.traj, replay.traj)
    assert scored.reached == replay.reached


def test_check_flags_a_changed_row(tmp_path):
    jp, mp = str(tmp_path / "b.json"), str(tmp_path / "b.md")
    B.update(["mpc_grid"], seeds=1, the_set=_tiny_set(), json_path=jp,
             md_path=mp, stamp={"commit": "t", "date": "d"}, log=lambda *a: None)
    t = B.load(jp)
    t["policies"]["mpc_grid"]["cells"]["L1_static_open"]["success"] = 0.123
    import json
    with open(jp, "w") as fh:
        json.dump(t, fh)
    diffs = B.check("mpc_grid", seeds=1, the_set=_tiny_set(), json_path=jp,
                    log=lambda *a: None)
    assert diffs and "L1_static_open" in diffs[0]
