"""Run the policies over the simulator scenarios and report/compare metrics.

Metrics per episode: reached goal, collided, time-to-goal, path length, minimum
clearance to any obstacle (ground truth), final distance to B, control effort.
Aggregated per variant: success rate, collision rate, and means over the episodes
that succeeded. benchmark.py is the CLI front-end; import run_variant / compare
from here to script your own comparisons.
"""

import json

import numpy as np

from . import config as C
from .sim import KinematicSim, run_episode, default_scenarios
from .policies import Variant1Policy, Variant2Policy
from .registry import POLICY_REGISTRY, build_policy

_V1 = ("1", "vw", "v1")
_V2 = ("2", "grid", "v2")


def _cfg_for(variant, live, goal_dist):
    v = str(variant).lower()
    if v in _V1:
        cfg = C.live_config_v1() if live else C.sim_config_v1()
    elif v in _V2:
        cfg = C.live_config_v2() if live else C.sim_config_v2()
    else:
        raise ValueError(
            "unknown variant %r -- use 1 or 2, or a key registered in "
            "POLICY_REGISTRY (%s), which run_variant forwards to run_policy(). "
            "This used to fall through to variant 2 and silently benchmark the "
            "WRONG policy." % (variant, ", ".join(sorted(POLICY_REGISTRY)) or "none"))
    cfg.goal.goal_dist = goal_dist
    return cfg


def _default_plan_dt(cfg):
    """Control period the offline loop applies one planned step for.

    This is now ONE number everywhere: the global tick (TickConfig.period). The
    live runner executes exactly one planned step per tick and MPPIConfig.dt is the
    tick, so the offline loop reproduces the live cadence instead of running at a
    different period than the car (it used to close the loop at MPPIConfig.dt=0.6 s
    while the runner applied each step for 0.25 s, the period of the since-deleted
    LiveConfig.plan_rate).
    """
    mppi = getattr(cfg, "mppi", None)
    if mppi is not None:
        return mppi.dt
    return C.TickConfig().period


def _seed_policy(policy, seed):
    """Give a freshly built policy its OWN rng for this episode.

    build_policy's signature is fixed (GUI/CLI/third-party contract) and has no
    seed, so every policy it returns starts at the default seed 0 -- which made
    run_policy(seed=N) vary NOTHING undisturbed: 20 "seeds" were 20 identical
    copies of the same 6 episodes. (The same bug was already fixed once for
    run_variant -- see below -- and had quietly survived on this path.) Policies
    that expose `seed()` get it called; else a `rng` attribute is replaced; a
    policy with neither is deterministic by contract and needs no seeding.
    """
    if seed is None:
        return
    seeder = getattr(policy, "seed", None)
    if callable(seeder):
        seeder(seed)
    elif hasattr(policy, "rng"):
        policy.rng = np.random.default_rng(seed)


def resolve_policy(spec, live=False, goal_dist=1.0, goal_y=0.0, magnitude=40.0,
                   step_duration=0.5, plan_dt=None, disturbed=False, seed=0):
    """THE one place a policy spec becomes (policy, cfg, dt) for offline eval.

    `spec` is 1/2 (built-in variants; `live` picks sim vs live profile) or any
    POLICY_REGISTRY key (built through build_policy at `magnitude`, then seeded).
    Registry keys are matched FIRST and VERBATIM (see run_variant for why), and
    anything unknown raises instead of silently running variant 2.

    Shared by run_variant, run_policy and the TestField so the seed plumbing and
    the disturbed-mode dt rules cannot drift apart again: dt is the policy's own
    control period (mppi.dt / step_duration) undisturbed, and the LIVE TICK for
    every policy when disturbed -- where a discrete policy also gets rollout_dt
    set, because the buffered loop executes one tick of each hop (runner parity).
    """
    s = str(spec)
    if s in POLICY_REGISTRY:
        policy, cfg = build_policy(s, magnitude, goal_dist, goal_y=goal_y,
                                   step_duration=step_duration)
        _seed_policy(policy, seed)
        cfg.goal.goal_dist = goal_dist
        cfg.goal.goal_y = goal_y
        if disturbed:
            # runner parity for ANY registry policy: the executed step is one
            # live tick, and the policy's model step must BE that tick (the
            # runner overwrites mppi.dt/rollout_dt the same way). Without this,
            # a third-party cfg with mppi.dt != 1/3 ran the "live-faithful"
            # mode at a cadence the car never runs.
            dt = plan_dt if plan_dt is not None else C.TickConfig().period
            if getattr(policy, "action_space", None) == "discrete":
                cfg.rollout_dt = dt
            if getattr(cfg, "mppi", None) is not None:
                cfg.mppi.dt = dt
            if hasattr(cfg, "pred_extra_delay_s"):
                # buffered loop: the plan starts executing one tick after the
                # frame it was planned from -- predictions must look that much
                # further ahead (runner.py sets the same thing live)
                cfg.pred_extra_delay_s = dt
        else:
            dt = plan_dt if plan_dt is not None else _default_plan_dt(cfg)
        return policy, cfg, dt
    v = s.lower()
    if v not in _V1 and v not in _V2:
        _cfg_for(spec, live, goal_dist)          # raises with the full message
    cfg = _cfg_for(spec, live, goal_dist)
    cfg.goal.goal_y = goal_y
    is_v1 = v in _V1
    if disturbed:
        dt = plan_dt if plan_dt is not None else C.TickConfig().period
        if is_v1:
            cfg.mppi.dt = dt              # runner parity (a no-op at the shipped 1/3)
        else:
            cfg.rollout_dt = dt
        cfg.pred_extra_delay_s = dt       # buffered loop's one-tick dispatch delay
    else:
        dt = plan_dt if plan_dt is not None else (
            cfg.mppi.dt if is_v1 else cfg.step_duration)
    policy = (Variant1Policy(cfg, seed=seed) if is_v1
              else Variant2Policy(cfg, seed=seed))
    return policy, cfg, dt


def run_variant(variant, scenarios=None, live=False, goal_dist=1.0,
                obs_cfg=None, seed=0, sense_range=3.0, plan_dt=None,
                disturbed=False):
    """Run one variant over the scenarios (default suite). Fresh policy + sim per
    scenario. Returns a list of EpisodeResult.

    `variant` is 1 or 2. Any key in POLICY_REGISTRY (including your own model) is
    forwarded to run_policy(); anything else raises instead of silently running
    variant 2. `disturbed=True` is the LIVE-FAITHFUL evaluation: the measured
    execution-disturbance model (DisturbanceConfig, fit from 298 on-car ticks)
    plus the runner's buffered/compensated tick loop. Screen any smoothness or
    robustness tuning against it -- the perfect-execution default mis-ranked
    w_cont once already (looked free, went 0/2 on the car). `plan_dt` overrides
    the control period. Undisturbed, a discrete policy runs each action for its
    own duration and plan_dt does nothing for it; DISTURBED (buffered), one tick
    of each hop is what executes -- there plan_dt IS the discrete execution step,
    defaulting to the tick period exactly like the live runner.
    """
    # Registry keys are matched FIRST and VERBATIM, because register()/build_policy()
    # treat them verbatim. Lower-casing first (as this used to) meant a key that
    # collided with a built-in alias -- "grid", "vw", "v1", "2", any casing -- ran
    # the BUILT-IN instead, labelled with the caller's key: exactly the silent
    # wrong-policy failure this function was rewritten to stop. Mixed-case keys were
    # also unreachable by any spelling.
    v_exact = str(variant)
    if v_exact in POLICY_REGISTRY:
        return run_policy(v_exact, scenarios=scenarios, goal_dist=goal_dist,
                          obs_cfg=obs_cfg, seed=seed, sense_range=sense_range,
                          plan_dt=plan_dt, disturbed=disturbed)
    v = v_exact.lower()
    if v not in _V1 and v not in _V2:
        _cfg_for(variant, live, goal_dist)        # raises with the full message

    scenarios = scenarios if scenarios is not None else default_scenarios(goal_dist)
    obs_cfg = obs_cfg or C.ObstacleConfig()
    dist = C.DisturbanceConfig() if disturbed else None
    results = []
    for world in scenarios:
        # resolve_policy per scenario = fresh policy AND fresh cfg each episode,
        # with the seed and disturbed-dt rules applied in the one shared place.
        policy, cfg, dt = resolve_policy(variant, live=live, goal_dist=goal_dist,
                                         plan_dt=plan_dt, disturbed=disturbed,
                                         seed=seed)
        sim = KinematicSim(world, sense_range=sense_range,
                           robot_radius=cfg.robot.robot_radius, seed=seed,
                           disturbance=dist)
        results.append(run_episode(sim, policy, variant, obs_cfg, cfg.goal,
                                   plan_dt=dt, robot_cfg=cfg.robot,
                                   buffered=disturbed))
    return results


def run_policy(policy_key, scenarios=None, goal_dist=1.0, goal_y=0.0,
               magnitude=40.0, obs_cfg=None, seed=0, sense_range=3.0,
               plan_dt=None, step_duration=0.5, disturbed=False):
    """Run ANY registered policy -- including your own model -- over the scenarios.

    Fresh policy + sim per scenario, constructed through registry.build_policy so
    the action_space / build-signature checks apply here too, then seeded (see
    _seed_policy -- build_policy itself cannot take a seed). `magnitude` defaults
    to 40: the value the five on-car 5 m runs used AND the shipped live default
    (LiveConfig.magnitude); below ~17.4 the car cannot steer at all, see
    calibration/README.md. Returns a list of EpisodeResult, so print_table / summarize
    / to_json work on it exactly like a built-in variant.

    NOTE the registry always builds a LIVE config (registry build() calls
    config.build_live_cfg), so run_policy("mpc_vw") is the live profile while
    run_variant(1) defaults to the SIM profile -- their numbers differ. Use
    run_variant(1, live=True) to compare like with like.
    """
    if str(policy_key) not in POLICY_REGISTRY:
        raise KeyError("unknown policy %r; registered: %s"
                       % (policy_key, ", ".join(sorted(POLICY_REGISTRY)) or "(none)"))
    scenarios = scenarios if scenarios is not None else default_scenarios(goal_dist)
    obs_cfg = obs_cfg or C.ObstacleConfig()

    results = []
    for world in scenarios:
        policy, cfg, dt = resolve_policy(policy_key, goal_dist=goal_dist,
                                         goal_y=goal_y, magnitude=magnitude,
                                         step_duration=step_duration,
                                         plan_dt=plan_dt, disturbed=disturbed,
                                         seed=seed)
        sim = KinematicSim(world, sense_range=sense_range,
                           robot_radius=cfg.robot.robot_radius, seed=seed,
                           disturbance=(C.DisturbanceConfig() if disturbed else None))
        results.append(run_episode(sim, policy, policy_key, obs_cfg, cfg.goal,
                                   plan_dt=dt, robot_cfg=cfg.robot,
                                   buffered=disturbed))
    return results


def summarize(results):
    """Aggregate a list of EpisodeResult into a dict of headline metrics."""
    n = len(results)
    succ = [r for r in results if r.reached and not r.collided]
    coll = [r for r in results if r.collided]
    # clearance only meaningful where obstacles exist (finite clearance)
    clr = [r.min_clearance for r in results if np.isfinite(r.min_clearance)]
    return {
        "episodes": n,
        "success_rate": len(succ) / n if n else 0.0,
        "collision_rate": len(coll) / n if n else 0.0,
        "mean_time_s": float(np.mean([r.sim_time for r in succ])) if succ else None,
        "mean_path_len": float(np.mean([r.path_length for r in succ])) if succ else None,
        "mean_min_clearance": float(np.mean(clr)) if clr else None,
        "mean_effort": float(np.mean([r.control_effort for r in succ])) if succ else None,
    }


def _fmt(x, nd=3):
    return "  -  " if x is None else ("%.*f" % (nd, x))


def print_table(results_by_variant, scenario_rows=True, aggregate=True):
    """results_by_variant: {label: [EpisodeResult]}. Prints per-scenario status
    and an aggregate row per variant. benchmark.py's multi-seed mode splits the
    two sections: scenario rows from seed 0 only (a 120-row listing would be
    noise), the aggregate over all seeds (a seed-0 aggregate next to it would
    just re-print the misleading single-seed numbers)."""
    labels = list(results_by_variant)

    if scenario_rows:
        scen = [r.name for r in results_by_variant[labels[0]]]
        print("\nPer-scenario (R=reached, C=collided, x=failed):")
        header = "  %-16s" % "scenario" + "".join("  %-10s" % l for l in labels)
        print(header)
        for i, name in enumerate(scen):
            row = "  %-16s" % name
            for l in labels:
                r = results_by_variant[l][i]
                tag = "C-hit" if r.collided else ("R %4.1fs" % r.sim_time if r.reached else "x-fail")
                row += "  %-10s" % tag
            print(row)

    summ = {l: summarize(results_by_variant[l]) for l in labels}
    if aggregate:
        print("\nAggregate:")
        keys = ["success_rate", "collision_rate", "mean_time_s", "mean_path_len",
                "mean_min_clearance", "mean_effort"]
        print("  %-20s" % "metric" + "".join("  %-10s" % l for l in labels))
        for k in keys:
            row = "  %-20s" % k
            for l in labels:
                row += "  %-10s" % _fmt(summ[l][k])
            print(row)
    return summ


def to_json(results_by_variant, path):
    obj = {}
    for l, rs in results_by_variant.items():
        obj[l] = {"summary": summarize(rs),
                  "episodes": [{k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                for k, v in r._asdict().items() if k != "traj"}
                               for r in rs]}
    with open(path, "w") as fh:
        json.dump(obj, fh, indent=2)
    return path
