# MPC benchmark -- fixed eval protocol + tiered set, v2

**Do not edit numbers by hand** -- update with `python scripts/benchmark_table.py [--policy KEY]`, verify a row with `--check KEY`. Source of truth: `benchmarks.json`. Every current and FUTURE policy is evaluated here, same seeds, same set.

Protocol: registry-built policies (LIVE profile, magnitude 40 -- the plug-in contract), live-faithful execution everywhere (buffered tick + measured disturbance; the perfect-execution world has mis-ranked controllers twice and is not part of the protocol), 20 seeds per case, collision = footprint contact. Deterministic per code version: a `--check` mismatch means behaviour changed.

Difficulty tiers (graded by construction, fixed generator seeds):

| tier | contents | eps/policy |
|---|---|---|
| **L1 static-open** | realistic suite: 4 static worlds, B=3 m | 80 |
| **L2 static-tight** | tight suite: 6 static worlds, B=1 m, partly beyond the turn radius | 120 |
| **L3 dyn-single** | cross_slow/cross_fast/oncoming/diagonal + 6 random single-mover cases (seed 1003) | 200 |
| **L4 dyn-complex** | occluded_oncoming + 9 random clutter cases: 1-2 statics AND 1-2 movers (seed 1004) | 200 |

## Results (success / collision)

| policy | L1 | L2 | L3 | L4 | overall | L3+L4 turn(deg) | L3+L4 tail(m) | commit | date |
|---|---|---|---|---|---|---|---|---|---|
| `mpc_grid` | 1.000 / 0.000 | 1.000 / 0.000 | 0.755 / 0.245 | 0.835 / 0.165 | 0.863 / 0.137 | 21.7 | 0.184 | cd7942a | 2026-08-05 |
| `mpc_grid_t` | 1.000 / 0.000 | 1.000 / 0.000 | 1.000 / 0.000 | 0.970 / 0.030 | 0.990 / 0.010 | 22.4 | 0.160 | cd7942a | 2026-08-05 |
| `mpc_vw` | 0.850 / 0.113 | 0.350 / 0.600 | 0.725 / 0.270 | 0.485 / 0.450 | 0.587 / 0.375 | 6.3 | 0.157 | cd7942a | 2026-08-05 |
| `mpc_vw_t` | 0.850 / 0.113 | 0.350 / 0.600 | 0.915 / 0.030 | 0.770 / 0.185 | 0.745 / 0.207 | 6.4 | 0.122 | cd7942a | 2026-08-05 |

`turn` = mean direction change between trajectory steps (smoothness, dynamic tiers); `tail` = mean |cross-track| over each episode's last third (return-to-line, dynamic tiers).
