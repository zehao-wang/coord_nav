"""ORCA baseline: builds through the registry, avoids, yields, and runs in the
TestField -- the plug-in contract holds for an external-library policy."""

import numpy as np
import pytest

from mpc_baseline import config as C
from mpc_baseline import testfield as TF
from mpc_baseline.registry import build_policy
from mpc_baseline.sim import World
from mpc_baseline.eval import run_policy


def test_builds_through_registry_with_matching_action_space():
    policy, cfg = build_policy("orca", 35.0, 3.0)
    assert policy.action_space == "discrete"


def test_dodges_a_head_on_mover():
    # oncoming mover dead ahead: ORCA must not keep driving straight at it
    from mpc_baseline.obstacles import ObstacleField
    from carpolicy import Observation
    policy, cfg = build_policy("orca", 35.0, 3.0)
    clk = {"t": 0.0}
    field = ObstacleField(C.ObstacleConfig(), lambda: clk["t"])
    y = 0.0
    x = 1.6
    for k in range(7):                     # track it long enough to be gated
        field.update([(x, y, 0.12)], (0, 0, 0))
        clk["t"] += 1 / 3
        x -= 0.3 / 3
    obs = Observation(np.zeros(3), np.array([3.0, 0.0]),
                      [(x, y, 0.12)], field)
    act = policy.plan(obs)
    # gated velocity present and pointing at us -> the chosen hop must have a
    # lateral component (not pure forward, action 1)
    v = field.velocities()
    assert np.hypot(v[0, 0], v[0, 1]) > 0.2
    assert act.action_id != 1


def test_yields_when_boxed_in():
    # surrounded at close range: ORCA emits STOP instead of a doomed hop
    from mpc_baseline.obstacles import ObstacleField
    from carpolicy import Observation
    policy, cfg = build_policy("orca", 35.0, 3.0)
    clk = {"t": 0.0}
    field = ObstacleField(C.ObstacleConfig(), lambda: clk["t"])
    ring = [(0.32 * np.cos(a), 0.32 * np.sin(a), 0.1)
            for a in np.linspace(0, 2 * np.pi, 9)[:-1]]
    field.update(ring, (0, 0, 0))
    obs = Observation(np.zeros(3), np.array([3.0, 0.0]), ring, field)
    act = policy.plan(obs)
    assert act.action_id == 0


def test_completes_an_open_course_and_field_case():
    # static suite world with an offset box: reaches B through run_policy
    w = World([(1.4, 0.4, 0.14)], (0, 0, 0), "open")
    rs = run_policy("orca", [w], goal_dist=3.0, seed=0, disturbed=True)
    assert rs[0].reached and not rs[0].collided
    # and a TestField mover case runs end-to-end without crashing
    case = [c for c in TF.archetype_cases() if c.name == "cross_fast"][0]
    res = TF.run_case("orca", case, seed=0)
    assert res.steps > 3


def test_static_wall_head_on_slides_around_not_freezes():
    # THE static-channel pin: a wall dead on the A->B line. As zero-velocity
    # AGENTS this froze forever (one hop then 70+ zero-displacement ticks --
    # the textbook freezing robot); through RVO2's native line-obstacle
    # channel (+ the sticky Blocks.cpp perturbation) the head-on approach
    # resolves to sliding around the wall. Regressing EITHER piece (obstacle
    # channel, minimum-magnitude sticky bias) fails this.
    w = World([(0.40, 0.00, 0.11), (0.62, 0.00, 0.11)], (0, 0, 0), "wall_inline")
    r = run_policy("orca", [w], goal_dist=1.0, seed=0, disturbed=True)[0]
    assert r.reached and not r.collided


def test_orca_vw_builds_and_completes_open_course():
    # the (v,w) variant mirrors mpc_vw's action space: builds through the
    # registry with the matching space and drives an open course to B
    policy, cfg = build_policy("orca_vw", 40.0, 3.0)
    assert policy.action_space == "velocity"
    w = World([(1.4, 0.4, 0.14)], (0, 0, 0), "open")
    r = run_policy("orca_vw", [w], goal_dist=3.0, seed=0, disturbed=True)[0]
    assert r.reached and not r.collided


def test_orca_is_deterministic_under_the_episode_seed():
    # the sticky vpref perturbation must be driven by the seeded rng ONLY --
    # the fixed benchmark protocol's --check regression detection depends on
    # same-seed bit-identical trajectories
    w = World([(0.40, 0.00, 0.11), (0.62, 0.00, 0.11)], (0, 0, 0), "wall_inline")
    a = run_policy("orca", [w], goal_dist=1.0, seed=3, disturbed=True)[0]
    b = run_policy("orca", [w], goal_dist=1.0, seed=3, disturbed=True)[0]
    c = run_policy("orca", [w], goal_dist=1.0, seed=4, disturbed=True)[0]
    ta, tb, tc = (np.asarray(x.traj) for x in (a, b, c))
    assert ta.shape == tb.shape and np.allclose(ta, tb)
    assert ta.shape != tc.shape or not np.allclose(ta, tc)
