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


def _cfg_for(variant, live, goal_dist):
    if variant == 1:
        cfg = C.live_config_v1() if live else C.sim_config_v1()
    else:
        cfg = C.live_config_v2() if live else C.sim_config_v2()
    cfg.goal.goal_dist = goal_dist
    return cfg


def run_variant(variant, scenarios=None, live=False, goal_dist=1.0,
                obs_cfg=None, seed=0, sense_range=3.0):
    """Run one variant over the scenarios (default suite). Fresh policy + sim per
    scenario. Returns a list of EpisodeResult."""
    scenarios = scenarios if scenarios is not None else default_scenarios(goal_dist)
    obs_cfg = obs_cfg or C.ObstacleConfig()
    cfg = _cfg_for(variant, live, goal_dist)
    plan_dt = cfg.mppi.dt if variant == 1 else cfg.step_duration

    results = []
    for world in scenarios:
        sim = KinematicSim(world, sense_range=sense_range,
                           robot_radius=cfg.robot.robot_radius, seed=seed)
        policy = (Variant1Policy(cfg) if variant == 1
                  else Variant2Policy(cfg, seed=seed))
        results.append(run_episode(sim, policy, variant, obs_cfg, cfg.goal,
                                   plan_dt=plan_dt))
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
