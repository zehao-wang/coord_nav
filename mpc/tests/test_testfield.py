"""TestField regression tests: obstacle motion, swept contact, occlusion,
random-case reproducibility, and the plug-in policy path.

Run from the mpc/ directory: cd mpc && python -m pytest tests/ -q
"""

import numpy as np
import pytest

from carpolicy import Policy, Action
from mpc_baseline import config as C
from mpc_baseline import testfield as TF
from mpc_baseline.registry import POLICY_REGISTRY, register
from mpc_baseline.sim import run_episode

ARENA = (-0.8, -1.4, 3.8, 1.4)


class _Ram(Policy):
    """Full speed dead ahead -- probes contact without planner behaviour."""
    action_space = "velocity"

    def plan(self, obs):
        return Action.velocity(0.3, 0.0)


class _Stop(Policy):
    """Sits still -- probes the world without the car moving."""
    action_space = "velocity"

    def plan(self, obs):
        return Action.velocity(0.0, 0.0)


def test_obstacles_move_and_bounce_elastically():
    case = TF.Case("mover", (0, 0, 0), (), ((2.0, 1.0, 0.12, 0.0, 0.40),), ARENA)
    sim = TF.DynamicSim(case)
    sim.step(np.zeros(3), 1.0 / 3.0)
    # one tick: moved up by ~v*dt toward the wall
    assert sim.circles[0, 1] == pytest.approx(1.0 + 0.40 / 3.0, abs=0.02)
    # run into the y=1.4 wall: after enough time it must have bounced back down
    for _ in range(12):
        sim.step(np.zeros(3), 1.0 / 3.0)
    assert sim.circles[0, 1] < 1.30
    # elastic: speed preserved through the bounce
    v = sim._bodies[0].velocity
    assert np.hypot(v.x, v.y) == pytest.approx(0.40, rel=1e-3)


def test_swept_contact_catches_a_midstep_graze():
    # A fast crosser passes exactly through the stationary car's spot within one
    # tick: at the tick BOUNDARY it is already past (clearance positive again),
    # so only the substep sweep can see the hit.
    case = TF.Case("graze", (0, 0, 0), (),
                   ((0.0, -0.60, 0.12, 0.0, 3.6),), ARENA)  # crosses y=0 mid-tick
    sim = TF.DynamicSim(case, substeps=10)
    sim.step(np.zeros(3), 1.0 / 3.0)
    assert sim.true_min_clearance() < sim.robot_radius     # contact seen
    inst = np.hypot(*(sim.circles[0, :2] - sim.pose[:2])) - sim.circles[0, 2]
    assert inst > sim.robot_radius                         # ...though it is already past


def test_occlusion_hides_and_reveals():
    # A static box between car and target circle: hidden; move aside: visible.
    case = TF.Case("occ", (0, 0, 0), ((1.0, 0.0, 0.15),),
                   ((2.0, 0.0, 0.12, 0.0, 0.0),), ARENA)
    sim = TF.DynamicSim(case)
    seen = [c for c in sim.sense()]
    assert len(seen) == 1                                  # only the front box
    case2 = TF.Case("occ2", (0, 0, 0), ((1.0, 0.9, 0.15),),
                    ((2.0, 0.0, 0.12, 0.0, 0.0),), ARENA)
    sim2 = TF.DynamicSim(case2)
    assert len(sim2.sense()) == 2                          # off the ray: both seen
    sim3 = TF.DynamicSim(case, occlusion=False)
    assert len(sim3.sense()) == 2                          # knob off: x-ray as before


def test_occluded_oncoming_archetype_actually_occludes():
    # the case named for the occlusion feature must start with its mover HIDDEN
    case = [c for c in TF.archetype_cases() if c.name == "occluded_oncoming"][0]
    sim = TF.DynamicSim(case)
    assert len(sim.sense()) == 1        # only the front box; the mover is shadowed


def test_zero_command_is_a_brake_not_a_random_walk():
    # the real car holds heading under a (0,0,0) brake pulse; the disturbance
    # model used to inject yaw noise anyway, spinning a waiting car ~60 deg/10 s
    case = TF.Case("still", (0, 0, 0), (), ((2.0, 1.0, 0.12, 0.1, 0.0),), ARENA)
    sim = TF.DynamicSim(case, disturbance=C.DisturbanceConfig(), seed=3)
    for _ in range(30):                 # 10 s of commanded stop
        sim.step(np.zeros(3), 1.0 / 3.0)
    assert sim.pose[2] == pytest.approx(0.0, abs=1e-9)


def test_field_rejects_short_goal_dist():
    with pytest.raises(ValueError):
        TF.random_cases(3, seed=0, goal_dist=1.0)
    with pytest.raises(ValueError):
        TF.archetype_cases(goal_dist=1.2)


def test_random_cases_reproducible_and_distinct():
    a = TF.random_cases(5, seed=3)
    b = TF.random_cases(5, seed=3)
    c = TF.random_cases(5, seed=4)
    assert a == b
    assert a != c
    for case in a:
        for (x, y, r, *_v) in case.dynamic:
            assert np.hypot(x, y) > 0.55                   # spawn clear of start


def test_field_contact_ends_episode_as_collision():
    # ram straight into an oncoming obstacle: the field must score a collision
    # at footprint contact (positive centre clearance), same ruler as the guard
    case = TF.Case("headon", (0, 0, 0), (), ((2.0, 0.0, 0.13, -0.3, 0.0),), ARENA)
    sim = TF.DynamicSim(case)
    res = run_episode(sim, _Ram(), "ram", C.ObstacleConfig(),
                      C.GoalConfig(goal_dist=3.0), plan_dt=1.0 / 3.0)
    assert res.collided
    assert res.min_clearance < sim.robot_radius


def test_plugged_in_policy_runs_and_seeds():
    # any registered Policy drops into the field through the same resolver the
    # benchmarks use, and gets seeded (rng replaced per episode)
    class _Jitter(Policy):
        action_space = "velocity"

        def __init__(self, cfg):
            self.cfg = cfg
            self.rng = np.random.default_rng(0)

        def plan(self, obs):
            return Action.velocity(0.25, float(self.rng.normal(0.0, 0.4)))

    def _build(magnitude, goal_x, goal_y=0.0, step_duration=0.5,
               allow_rotation=False):
        cfg = C.build_live_cfg("1", magnitude, goal_x, goal_y=goal_y)
        return _Jitter(cfg), cfg

    key = "_test_jitter"
    register(key, "test jitter", "velocity", _build)
    try:
        case = TF.archetype_cases()[0]
        a = TF.run_case(key, case, seed=0, disturbed=False)
        b = TF.run_case(key, case, seed=7, disturbed=False)
        assert not np.array_equal(a.traj, b.traj)          # seed reaches the policy
    finally:
        POLICY_REGISTRY.pop(key, None)
