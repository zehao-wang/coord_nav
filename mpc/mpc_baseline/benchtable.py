"""The FIXED eval protocol + eval set: one committed, same-seed, difficulty-
tiered benchmark that every policy -- current baselines and future models -- is
evaluated on.

Problem this solves: numbers used to live in hand-edited README cells and
scratch logs, comparable only if you trusted that whoever produced them ran the
same seeds, suites and profiles. This module pins ONE protocol and ONE set, and
records every policy's results in `mpc/benchmarks.json` (source of truth) and
`mpc/BENCHMARKS.md` (the generated fixed table).

THE PROTOCOL (bump PROTOCOL_VERSION whenever ANY of this changes):
  * every policy is built through the registry (`build_policy`) at the LIVE
    profile, magnitude 40 -- the exact plug-in contract a third-party model
    gets, so rows are apples-to-apples (NOTE: this differs from the README's
    historical variant-1 rows, which used the slower SIM profile);
  * every episode runs the LIVE-FAITHFUL loop (buffered tick + measured
    execution disturbance): the perfect-execution world has twice mis-ranked
    controllers (w_cont 0.9.3, v2's deterministic limit cycles) and is NOT part
    of the protocol;
  * seeds 0..19 per case; collision = footprint contact (the 0.9.15 metric,
    same test the live guard aborts on);
  * the SET is difficulty-tiered by CONSTRUCTION (not by measured hardness,
    which would drift as policies improve):

      L1 static-open   realistic_scenarios(3.0)    4 static worlds, B=3 m
      L2 static-tight  default_scenarios(1.0)      6 static worlds, B=1 m,
                                                   partly beyond the physical
                                                   turn radius (stress)
      L3 dyn-single    4 mover archetypes (cross_slow/cross_fast/oncoming/
                       diagonal) + 6 random single-mover cases (gen seed 1003)
      L4 dyn-complex   occluded_oncoming + 9 random cases with 1-2 static
                       obstacles AND 1-2 movers (gen seed 1004): occlusion,
                       interception and clutter together

Everything is deterministic given the code version, so `check()` re-runs a
policy and compares against its stored row exactly: the table doubles as a
regression detector for the whole planning/eval stack.
"""

import json
import os
import subprocess
from datetime import date

import numpy as np

from . import testfield as TF
from .eval import run_policy, summarize
from .registry import POLICY_REGISTRY
from .sim import default_scenarios, realistic_scenarios

PROTOCOL_VERSION = 1
SEEDS = 20
MAGNITUDE = 40.0
L3_RANDOM = (6, 1003)                 # (count, generator seed)
L4_RANDOM = (9, 1004)

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JSON_PATH = os.path.join(_ROOT, "benchmarks.json")
MD_PATH = os.path.join(_ROOT, "BENCHMARKS.md")

TIERS = ("L1_static_open", "L2_static_tight", "L3_dyn_single", "L4_dyn_complex")


def eval_set(l3_random=L3_RANDOM, l4_random=L4_RANDOM):
    """The fixed set, tier -> ("static", worlds, goal_dist) | ("field", cases).
    Case composition is part of the protocol version."""
    arch = {c.name: c for c in TF.archetype_cases(3.0)}
    l3 = [arch[n] for n in ("cross_slow", "cross_fast", "oncoming", "diagonal")]
    l3 += TF.random_cases(l3_random[0], seed=l3_random[1], goal_dist=3.0,
                          n_static=(0, 0), n_dynamic=(1, 1))
    l4 = [arch["occluded_oncoming"]]
    l4 += TF.random_cases(l4_random[0], seed=l4_random[1], goal_dist=3.0,
                          n_static=(1, 2), n_dynamic=(1, 2))
    return {
        "L1_static_open": ("static", realistic_scenarios(3.0), 3.0),
        "L2_static_tight": ("static", default_scenarios(1.0), 1.0),
        "L3_dyn_single": ("field", l3),
        "L4_dyn_complex": ("field", l4),
    }


def _turn_stats(rs):
    """Mean turn angle (deg) between consecutive displacement directions --
    the smoothness metric the 0.9.18 screening used."""
    angs = []
    for r in rs:
        d = np.diff(r.traj[:, :2], axis=0)
        n = np.hypot(d[:, 0], d[:, 1])
        d, n = d[n > 0.02], n[n > 0.02]
        if len(d) < 2:
            continue
        u = d / n[:, None]
        dots = np.clip((u[:-1] * u[1:]).sum(axis=1), -1.0, 1.0)
        angs.append(np.degrees(np.arccos(dots)))
    return float(np.concatenate(angs).mean()) if angs else None


def _tail_wander(rs):
    """Mean |cross-track| over each episode's last third (protocol worlds run
    A=(0,0)->B=(gd,0), so cross-track = |y|) -- the return-to-line metric."""
    t = [float(np.abs(r.traj[:, 1])[-max(1, len(r.traj) // 3):].mean()) for r in rs]
    return float(np.mean(t)) if t else None


def _cell(rs, extra_metrics=False):
    s = summarize(rs)
    out = {"episodes": s["episodes"],
           "success": round(s["success_rate"], 4),
           "collision": round(s["collision_rate"], 4),
           "mean_time_s": None if s["mean_time_s"] is None else round(s["mean_time_s"], 2),
           "mean_min_clearance": None if s["mean_min_clearance"] is None
           else round(s["mean_min_clearance"], 3)}
    if extra_metrics:
        t, w = _turn_stats(rs), _tail_wander(rs)
        out["mean_turn_deg"] = None if t is None else round(t, 1)
        out["tail_wander_m"] = None if w is None else round(w, 3)
    return out


def run_protocol(key, seeds=SEEDS, the_set=None, log=print):
    """Run the full fixed protocol for one registry policy -> row dict."""
    if key not in POLICY_REGISTRY:
        raise KeyError("unknown policy %r; registered: %s"
                       % (key, ", ".join(sorted(POLICY_REGISTRY))))
    the_set = the_set or eval_set()
    row = {"protocol": PROTOCOL_VERSION, "seeds": seeds, "cells": {}}
    all_rs = []
    for tier in TIERS:
        spec = the_set[tier]
        rs = []
        if spec[0] == "static":
            _, worlds, gd = spec
            for s in range(seeds):
                rs += run_policy(key, worlds, goal_dist=gd, magnitude=MAGNITUDE,
                                 seed=s, disturbed=True)
        else:
            rs = TF.run_field([key], spec[1], seeds=seeds,
                              magnitude=MAGNITUDE)[key]
        row["cells"][tier] = _cell(rs, extra_metrics=tier.startswith(("L3", "L4")))
        all_rs += rs
        log("  %-16s %-16s %.3f/%.3f" % (key, tier,
                                         row["cells"][tier]["success"],
                                         row["cells"][tier]["collision"]))
    row["cells"]["overall"] = _cell(all_rs)
    return row


def _stamp():
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=_ROOT,
            stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        commit = "unknown"
    return {"commit": commit, "date": date.today().isoformat()}


def load(path=JSON_PATH):
    if os.path.exists(path):
        with open(path) as fh:
            return json.load(fh)
    return {"protocol_version": PROTOCOL_VERSION, "policies": {}}


def update(keys, seeds=SEEDS, the_set=None, json_path=JSON_PATH, md_path=MD_PATH,
           stamp=None, log=print):
    """Run the protocol for `keys` and update their rows; regenerate the md.
    `stamp` overrides the commit/date metadata (tests pass a fixed one)."""
    table = load(json_path)
    table["protocol_version"] = PROTOCOL_VERSION
    for key in keys:
        log("benchmarking %s ..." % key)
        row = run_protocol(key, seeds=seeds, the_set=the_set, log=log)
        row.update(stamp or _stamp())
        table["policies"][key] = row
    with open(json_path, "w") as fh:
        json.dump(table, fh, indent=1, sort_keys=True)
    write_md(table, md_path)
    return table


def check(key, seeds=SEEDS, the_set=None, json_path=JSON_PATH, log=print):
    """Re-run `key` and compare against its stored row. Returns [] if the
    stored numbers reproduce exactly (the protocol is deterministic), else a
    list of diffs -- nonempty means either the planning stack changed behaviour
    or the stored row is stale."""
    stored = load(json_path)["policies"].get(key)
    if stored is None:
        return ["no stored row for %r" % key]
    if stored.get("protocol") != PROTOCOL_VERSION:
        return ["stored row is protocol v%s, current is v%s"
                % (stored.get("protocol"), PROTOCOL_VERSION)]
    fresh = run_protocol(key, seeds=seeds, the_set=the_set, log=log)
    diffs = []
    for tier in TIERS + ("overall",):
        a, b = stored["cells"].get(tier), fresh["cells"].get(tier)
        if a != b:
            diffs.append("%s: stored %s != fresh %s" % (tier, a, b))
    return diffs


def write_md(table, md_path=MD_PATH):
    lines = [
        "# MPC benchmark -- fixed eval protocol + tiered set, v%d"
        % table["protocol_version"],
        "",
        "**Do not edit numbers by hand** -- update with "
        "`python scripts/benchmark_table.py [--policy KEY]`, verify a row with "
        "`--check KEY`. Source of truth: `benchmarks.json`. Every current and "
        "FUTURE policy is evaluated here, same seeds, same set.",
        "",
        "Protocol: registry-built policies (LIVE profile, magnitude %.0f -- the "
        "plug-in contract), live-faithful execution everywhere (buffered tick + "
        "measured disturbance; the perfect-execution world has mis-ranked "
        "controllers twice and is not part of the protocol), %d seeds per case, "
        "collision = footprint contact. Deterministic per code version: a "
        "`--check` mismatch means behaviour changed." % (MAGNITUDE, SEEDS),
        "",
        "Difficulty tiers (graded by construction, fixed generator seeds):",
        "",
        "| tier | contents | eps/policy |",
        "|---|---|---|",
        "| **L1 static-open** | realistic suite: 4 static worlds, B=3 m | %d |" % (4 * SEEDS),
        "| **L2 static-tight** | tight suite: 6 static worlds, B=1 m, partly beyond the turn radius | %d |" % (6 * SEEDS),
        "| **L3 dyn-single** | cross_slow/cross_fast/oncoming/diagonal + 6 random single-mover cases (seed %d) | %d |" % (L3_RANDOM[1], 10 * SEEDS),
        "| **L4 dyn-complex** | occluded_oncoming + 9 random clutter cases: 1-2 statics AND 1-2 movers (seed %d) | %d |" % (L4_RANDOM[1], 10 * SEEDS),
        "",
        "## Results (success / collision)",
        "",
        "| policy | L1 | L2 | L3 | L4 | overall | L3+L4 turn(deg) | L3+L4 tail(m) | commit | date |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]

    def fmt(c):
        return "%.3f / %.3f" % (c["success"], c["collision"])

    for key in sorted(table["policies"]):
        row = table["policies"][key]
        c = row["cells"]
        stale = "" if row.get("protocol") == table["protocol_version"] else " **(STALE)**"
        turns = [c[t].get("mean_turn_deg") for t in ("L3_dyn_single", "L4_dyn_complex")
                 if c.get(t, {}).get("mean_turn_deg") is not None]
        tails = [c[t].get("tail_wander_m") for t in ("L3_dyn_single", "L4_dyn_complex")
                 if c.get(t, {}).get("tail_wander_m") is not None]
        lines.append("| `%s`%s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            key, stale,
            fmt(c["L1_static_open"]), fmt(c["L2_static_tight"]),
            fmt(c["L3_dyn_single"]), fmt(c["L4_dyn_complex"]), fmt(c["overall"]),
            "-" if not turns else "%.1f" % float(np.mean(turns)),
            "-" if not tails else "%.3f" % float(np.mean(tails)),
            row.get("commit", "?"), row.get("date", "?")))
    lines += ["",
              "`turn` = mean direction change between trajectory steps "
              "(smoothness, dynamic tiers); `tail` = mean |cross-track| over "
              "each episode's last third (return-to-line, dynamic tiers)."]
    with open(md_path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    return md_path
