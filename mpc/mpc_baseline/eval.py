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
    while the runner applied each step for 1/plan_rate=0.25 s).
    """
    mppi = getattr(cfg, "mppi", None)
    if mppi is not None:
        return mppi.dt
    return C.TickConfig().period


def run_variant(variant, scenarios=None, live=False, goal_dist=1.0,
                obs_cfg=None, seed=0, sense_range=3.0, plan_dt=None):
    """Run one variant over the scenarios (default suite). Fresh policy + sim per
    scenario. Returns a list of EpisodeResult.

    `variant` is 1 or 2. Any key in POLICY_REGISTRY (including your own model) is
    forwarded to run_policy(); anything else raises instead of silently running
    variant 2. `plan_dt` overrides the control period -- VELOCITY policies only;
    a discrete policy runs each action for its own duration (see run_episode), so
    plan_dt does nothing for it.
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
                          plan_dt=plan_dt)
    v = v_exact.lower()
    if v not in _V1 and v not in _V2:
        _cfg_for(variant, live, goal_dist)        # raises with the full message

    scenarios = scenarios if scenarios is not None else default_scenarios(goal_dist)
    obs_cfg = obs_cfg or C.ObstacleConfig()
    cfg = _cfg_for(variant, live, goal_dist)
    is_v1 = v in _V1
    dt = plan_dt if plan_dt is not None else (cfg.mppi.dt if is_v1 else cfg.step_duration)

    results = []
    for world in scenarios:
        sim = KinematicSim(world, sense_range=sense_range,
                           robot_radius=cfg.robot.robot_radius, seed=seed)
        policy = (Variant1Policy(cfg) if is_v1 else Variant2Policy(cfg, seed=seed))
        results.append(run_episode(sim, policy, variant, obs_cfg, cfg.goal,
                                   plan_dt=dt, robot_cfg=cfg.robot))
    return results


def run_policy(policy_key, scenarios=None, goal_dist=1.0, goal_y=0.0,
               magnitude=40.0, obs_cfg=None, seed=0, sense_range=3.0,
               plan_dt=None, step_duration=0.5):
    """Run ANY registered policy -- including your own model -- over the scenarios.

    Fresh policy + sim per scenario, constructed through registry.build_policy so
    the action_space / build-signature checks apply here too. `magnitude` defaults
    to 40 (the GUI default and the value the car was validated at; 20 cannot steer,
    see mpc/README.md). Returns a list of EpisodeResult, so print_table / summarize
    / to_json work on it exactly like a built-in variant.

    NOTE the registry always builds a LIVE config (registry build() calls
    config.build_live_cfg), so run_policy("mpc_vw") is the live profile while
    run_variant(1) defaults to the SIM profile -- their numbers differ. Use
    run_variant(1, live=True) to compare like with like.
    """
    scenarios = scenarios if scenarios is not None else default_scenarios(goal_dist)
    obs_cfg = obs_cfg or C.ObstacleConfig()

    results = []
    for world in scenarios:
        policy, cfg = build_policy(policy_key, magnitude, goal_dist, goal_y=goal_y,
                                   step_duration=step_duration)
        cfg.goal.goal_dist = goal_dist
        cfg.goal.goal_y = goal_y
        dt = plan_dt if plan_dt is not None else _default_plan_dt(cfg)
        sim = KinematicSim(world, sense_range=sense_range,
                           robot_radius=cfg.robot.robot_radius, seed=seed)
        results.append(run_episode(sim, policy, policy_key, obs_cfg, cfg.goal,
                                   plan_dt=dt, robot_cfg=cfg.robot))
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


def print_table(results_by_variant):
    """results_by_variant: {label: [EpisodeResult]}. Prints per-scenario status
    and an aggregate row per variant."""
    labels = list(results_by_variant)
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

    print("\nAggregate:")
    keys = ["success_rate", "collision_rate", "mean_time_s", "mean_path_len",
            "mean_min_clearance", "mean_effort"]
    print("  %-20s" % "metric" + "".join("  %-10s" % l for l in labels))
    summ = {l: summarize(results_by_variant[l]) for l in labels}
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
