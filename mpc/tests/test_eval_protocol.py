"""Regression tests for the eval-protocol bug classes that have already bitten
this repo at least once (see CHANGELOG 0.9.5 and 0.9.15): seeds silently
ignored, collision scored on the footprint CENTRE instead of the footprint, and
the disturbed eval executing a different plant than the policy models.

Run from the mpc/ directory (NOT the repo root -- the root's carpolicy/ project
dir shadows the installed package):

    cd mpc && python -m pytest tests/ -q
"""

import numpy as np
import pytest

from carpolicy import Policy, Action
from mpc_baseline import config as C
from mpc_baseline import eval as E
from mpc_baseline.sim import KinematicSim, World, run_episode, default_scenarios

SLALOM = [w for w in default_scenarios() if w.name == "slalom"]
CLEAR = [World([], (0, 0, 0), "clear")]


class _Ram(Policy):
    """Drives dead straight at full tilt -- for the collision-metric test."""
    action_space = "velocity"

    def plan(self, obs):
        return Action.velocity(0.22, 0.0)


def test_run_policy_seed_varies_the_policy():
    # `seed` used to reach only the SIM through run_policy (build_policy has no
    # seed parameter), so undisturbed "multi-seed" runs of any registered policy
    # were N identical copies of one episode -- the exact bug CHANGELOG 0.9.5
    # fixed for run_variant, quietly surviving on the third-party path.
    a = E.run_policy("mpc_vw", scenarios=SLALOM, seed=0)
    b = E.run_policy("mpc_vw", scenarios=SLALOM, seed=7)
    assert not np.array_equal(a[0].traj, b[0].traj)


def test_run_variant_seed_varies_the_policy():
    a = E.run_variant(1, SLALOM, seed=0)
    b = E.run_variant(1, SLALOM, seed=7)
    assert not np.array_equal(a[0].traj, b[0].traj)


def test_collision_is_footprint_contact_not_centre_penetration():
    # The sim must flag a collision when the obstacle EDGE reaches the footprint
    # (what the live guard aborts on), not once the centre is inside the circle:
    # the old `clearance < 0` test let episodes end "success" with the obstacle
    # surface 12 cm into the car.
    world = World([(0.6, 0.0, 0.12)], (0, 0, 0), "head_on")
    sim = KinematicSim(world, robot_radius=0.13)
    res = run_episode(sim, _Ram(), "ram", C.ObstacleConfig(),
                      C.GoalConfig(goal_dist=2.0), plan_dt=1.0 / 3.0)
    assert res.collided
    # ends AT contact: centre-to-edge clearance still positive, below the radius
    assert 0.0 < res.min_clearance < sim.robot_radius


def test_disturbed_discrete_policy_rolls_out_at_the_executed_tick():
    # The buffered/disturbed loop executes ONE TICK of each hop (live runner
    # semantics), so the policy must roll out at the tick too -- otherwise it
    # predicts every hop 50% longer than what runs (the model error runner.py's
    # rollout_dt exists to fix, which had survived on this offline path).
    captured = {}
    orig = E.build_policy

    def spy(*args, **kw):
        policy, cfg = orig(*args, **kw)
        captured["cfg"] = cfg
        return policy, cfg

    E.build_policy = spy
    try:
        E.run_policy("mpc_grid", scenarios=CLEAR, disturbed=True, seed=0)
    finally:
        E.build_policy = orig
    assert captured["cfg"].rollout_dt == pytest.approx(C.TickConfig().period)


def test_disturbed_v2_executes_at_the_live_tick():
    # run_variant(2, disturbed=True) is the README's "live-faithful" row: it must
    # close the loop at the car's tick period, not step_duration (0.5 s = a 2 Hz
    # cadence the car never runs, with per-tick-fitted noise mis-scaled).
    r = E.run_variant(2, CLEAR, disturbed=True, seed=0)[0]
    assert r.steps > 0
    assert r.sim_time / r.steps == pytest.approx(C.TickConfig().period)
