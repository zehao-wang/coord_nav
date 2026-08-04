"""Constant-velocity obstacle tracking + the time-aware (*_t) MPC variants.

The load-bearing guarantee: in a STATIC world the *_t variants are byte-identical
to the plain ones (velocity gates read 0 -> predictions == current circles), so
the *_t-vs-plain comparison isolates "considers obstacle motion" as the single
experimental variable.
"""

import numpy as np
import pytest

from mpc_baseline import config as C
from mpc_baseline import eval as E
from mpc_baseline.obstacles import ObstacleField
from mpc_baseline.cost import obstacle_cost
from mpc_baseline.sim import default_scenarios
from mpc_baseline import testfield as TF

TICK = 1.0 / 3.0


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def _feed(field, clock, frames):
    """frames: list of [(x, y, r), ...] base-frame circles at successive ticks,
    car pinned at the origin (base == odom)."""
    for circles in frames:
        field.update(circles, (0.0, 0.0, 0.0))
        clock.t += TICK


def test_velocity_estimate_converges_and_static_reads_zero():
    clock = _Clock()
    field = ObstacleField(C.ObstacleConfig(), clock)
    # mover: 0.3 m/s in +y; static box next to it
    frames = [[(2.0, -0.6 + 0.3 * TICK * k, 0.12), (1.0, 0.5, 0.13)]
              for k in range(8)]
    _feed(field, clock, frames)
    v = field.velocities()
    mover = np.argmax(np.abs(v[:, 1]))
    assert v[mover, 1] == pytest.approx(0.3, abs=0.03)
    assert v[mover, 0] == pytest.approx(0.0, abs=0.03)
    static = 1 - mover
    assert v[static, 0] == 0.0 and v[static, 1] == 0.0     # gated to exact zero


def test_predict_extrapolates_and_caps():
    clock = _Clock()
    cfg = C.ObstacleConfig()
    field = ObstacleField(cfg, clock)
    _feed(field, clock, [[(2.0, 0.3 * TICK * k, 0.12)] for k in range(8)])
    pred = field.predict([1.0, 10.0])                       # (2, 1, 3)
    # moving tracks extrapolate from the last RAW observation (the EMA position
    # lags a steady mover by one tick); the clock advanced one tick after the
    # final update, so the extrapolation time is (age=TICK) + dt
    y_raw = 0.3 * TICK * 7                                  # last fed observation
    assert pred[0, 0, 1] == pytest.approx(y_raw + 0.3 * (TICK + 1.0), abs=0.02)
    # total extrapolation capped at pred_cap_s, not 10 s of straight-line ghost
    assert pred[1, 0, 1] == pytest.approx(y_raw + 0.3 * cfg.pred_cap_s, abs=0.02)


def test_time_aware_cost_penalizes_future_intercept():
    clock = _Clock()
    field = ObstacleField(C.ObstacleConfig(), clock)
    # mover heading to x=1.0, y=0 (arrives in ~1.7 s): currently still 0.5 m
    # short of the intercept point, comfortably clear in the frozen world
    _feed(field, clock, [[(1.0, -1.0 + 0.3 * TICK * k, 0.12)] for k in range(6)])
    # a rollout that sits at (1.0, 0) for 9 steps -- right on the intercept point
    states = np.tile(np.array([1.0, 0.0, 0.0]), (1, 9, 1))
    rc, cc = C.RobotConfig(), C.CostConfig()
    frozen, hit_frozen = obstacle_cost(states, field, rc, cc, TICK, predict=False)
    aware, hit_aware = obstacle_cost(states, field, rc, cc, TICK, predict=True)
    assert not hit_frozen[0]        # frozen world: mover is 0.6 m away, all clear
    assert hit_aware[0]             # time-aware: it will be HERE -- collision
    assert aware[0] > frozen[0]


def test_jittered_static_box_stays_gated_to_zero():
    # the gates carry the whole static-equivalence guarantee under REAL
    # perception: 1 cm lidar jitter on a static box must never open the
    # velocity gate (this is the test that fails if the deadband or the
    # sighting count is zeroed out)
    rng = np.random.default_rng(5)
    clock = _Clock()
    field = ObstacleField(C.ObstacleConfig(), clock)
    frames = [[(1.5 + rng.normal(0, 0.01), 0.4 + rng.normal(0, 0.01), 0.13)]
              for _ in range(30)]
    _feed(field, clock, frames)
    v = field.velocities()
    assert v[0, 0] == 0.0 and v[0, 1] == 0.0


def test_fast_mover_is_tracked():
    # acquisition used to cliff at 0.30 m/s (coasting by the still-gated zero
    # velocity + EMA position lag out-ran merge_dist before the gate opened),
    # leaving the 0.40 m/s cross_fast archetype silently untracked -- the *_t
    # variants equalled plain exactly where prediction matters most
    clock = _Clock()
    field = ObstacleField(C.ObstacleConfig(), clock)
    _feed(field, clock, [[(2.0, -1.0 + 0.40 * TICK * k, 0.12)] for k in range(8)])
    assert len(field.circles()) == 1                # one track, not a fragment trail
    assert field.velocities()[0, 1] == pytest.approx(0.40, abs=0.04)


def test_pred_delay_shifts_the_scoring_timeline():
    clock = _Clock()
    field = ObstacleField(C.ObstacleConfig(), clock)
    _feed(field, clock, [[(1.0, -1.0 + 0.3 * TICK * k, 0.12)] for k in range(6)])
    # a state 0.2 m left of the intercept point, for 2 steps: without dispatch
    # delay the mover is still ~0.4-0.5 m short (clear); with a 1 s delay the
    # prediction has it arriving -- collision flagged
    states = np.tile(np.array([1.0, 0.2, 0.0]), (1, 2, 1))
    rc, cc = C.RobotConfig(), C.CostConfig()
    _, hit_near = obstacle_cost(states, field, rc, cc, TICK, predict=True)
    _, hit_far = obstacle_cost(states, field, rc, cc, TICK, predict=True,
                               pred_delay=1.0)
    assert not hit_near[0]
    assert hit_far[0]
    # and the buffered eval path actually sets the delay to one tick
    _, cfg, dt = E.resolve_policy("mpc_vw_t", disturbed=True, goal_dist=3.0)
    assert cfg.pred_extra_delay_s == pytest.approx(dt)
    _, cfg2, _ = E.resolve_policy("mpc_vw_t", disturbed=False, goal_dist=3.0)
    assert cfg2.pred_extra_delay_s == 0.0


def test_t_variants_identical_to_plain_in_static_world():
    # the ablation guarantee: static world -> gates closed -> byte-identical
    scen = [w for w in default_scenarios() if w.name == "slalom"]
    for plain, aware in (("mpc_vw", "mpc_vw_t"), ("mpc_grid", "mpc_grid_t")):
        a = E.run_policy(plain, scenarios=scen, seed=3, disturbed=True)
        b = E.run_policy(aware, scenarios=scen, seed=3, disturbed=True)
        assert np.array_equal(a[0].traj, b[0].traj), plain
        assert a[0].min_clearance == b[0].min_clearance


def test_t_variant_dodges_the_intercepting_crosser():
    # cross_slow is the case both plain baselines fail catastrophically
    # (v2 0.85 / v1 1.00 collision over 20 seeds); the time-aware variant must
    # at least handle seed 0 -- a full-margin statistical claim lives in the
    # benchmark, but a deterministic dodge here catches gross regressions.
    case = [c for c in TF.archetype_cases() if c.name == "cross_slow"][0]
    res = TF.run_case("mpc_grid_t", case, seed=0)
    assert not res.collided
