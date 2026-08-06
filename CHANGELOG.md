# Changelog

Findings and notable changes. README is for day-to-day usage; this file records
*why* things are the way they are (hard-won during bring-up).

## 0.9.23 - 2026-08-06 - ORCA/RVO2 joins the board (the literature's reactive crowd baseline)

Context: the user asked about Human2Nav (ICRA 2026, crowd navigation from
human videos via feasibility-guided flow matching) as a baseline. NOT open
source as of today: only a project page exists (iunone.github.io/human2nav);
its "Code" button is an unedited template placeholder pointing at the
unrelated LeCAR-Lab/ASAP repo; no arXiv, no weights. Recorded for periodic
re-checking. Its classical baselines, however, are open: ORCA is now plugged
in; SARL (CrowdNav, MIT) fits the interface too but ships no weights -- it
needs a training run before it means anything (backlog).

mpc_baseline/orca_policy.py + registry key `orca`: one fresh RVO2 sim per
tick; robot = agent aiming at B, every remembered circle = an agent carrying
the tracker's gated velocity (statics/walls are zero-velocity agents) -- the
tracker work is precisely the sensing layer ORCA always assumed someone else
would provide. The collision-free holonomic output snaps to the nearest of
the 8 mecanum translation actions (same table the actuator runs), STOP when
ORCA yields. Deterministic; seeding is a no-op by design.

Dependency: Python-RVO2 built from source (cython<3, and cmake>=4 needs
CMAKE_POLICY_VERSION_MINIMUM=3.5 -- both bites documented here so the next
install takes minutes, not archaeology). Not in pyproject (no sdist on PyPI);
the registry imports it lazily so everything else works without it.

Honest expectations, borne out by the board: reactive-only with reciprocity
assumptions -- strong against movers, weak in static pockets/narrow passages.
The tiered table exists exactly to show such profiles honestly.

## 0.9.22 - 2026-08-05 - LIVE SESSION: v2 goes long-horizon sampled, the radius ratchet, and the dynamic A/B lands

A full on-car day. Every failure yielded a committed root cause; the session
ended with the designed result: a person crossing at ~0.3 m/s makes frozen
mpc_grid wander 7.54 m into a timeout while mpc_grid_t tracks them live
(23/41 ticks, 0.13-0.28 m/s estimates) and reaches B in 13.89 s / 3.97 m.
Recorded run.jsons render to top-down videos via NEW smoke/render_run.py
(tracker arrows + prediction ghosts included).

The chase, in order:
- **Deployed perception validated live**: 0 phantom velocity events in a 60 s
  no-human capture and a 47-frame operator-present capture (after adding the
  vel_trust_range=2.2 m gate: ALL residual phantoms sat at 2.2-3.0 m where
  far-lidar centroids jitter; bookkeeping continues at range so an entering
  mover is usable immediately).
- **Static A/B passed** (after course surgery): grid 9.93 s and 12.29 s,
  grid_t 11.94 s, threading a 0.35 m channel; a 26-tick live run showed mean
  3.8 cm trajectory deviation between the variants -- static byte-equivalence
  holds on the real car.
- **v2 rebuilt as a long-horizon warm-started SAMPLING MPC** (user call: "why
  exhaustive? aren't we MPC?"). Horizon 4 exhaustive -> 12 sampled
  (run-structured candidates -- the discrete analogue of v1's AR(1) noise --
  plus nominal mutations, 2048 samples x 2 refine passes). 10 ms solves (the
  h5-exhaustive attempt cost 270 ms) thanks to exact query-box culling in
  ObstacleField; offline tight-default jumped 0.833 -> 1.000 (wall_inline's
  beyond-horizon dead-end cured). mem_radius 3->4.5, sim sense_range 3->6
  (real-lidar parity; culling made far context free).
- **The RADIUS RATCHET** (live-only, invisible offline): field memory kept
  max(old, new) radius per track; live radii jitter, so a continuously-seen
  obstacle ratchets ~ +0.1 m over 100 ticks. Two ratcheted boxes shrank a
  0.35 m passable channel to ~0.15 m in memory and the car dithered at a gap
  it could physically thread -- twice, until the mechanism was found. Radius
  is now EMA'd like position. Next run threaded the channel and reached.
- Live wisdom logged: the ch341 serial wedge correlates with SUSTAINED
  DIAGONAL drive (max current; two wedges, both mid-diagonal; magnitude 35 +
  cable reseat = no recurrence); the wedge estop worked both times. v2 skims
  its inflated boundary by design and one hop of execution error can cross
  the contact guard -- margin is the razor (extra_margin 0.15 closed the
  room's corridors instead; policy_run grew an --extra-margin flag).
  Square boxes protrude past fitted circles: one graze at predicted 0.142 m
  clearance. Real rooms have WALL POCKETS and CLUTTERED GOALS the offline
  suites never model -- goal-blocked detection and walled scenarios are now
  protocol-v3 backlog.
- PROTOCOL_VERSION -> 2 (sense_range + the new v2); full 4-row table rebuilt.

## 0.9.21 - 2026-08-05 - hardening review round: split-cluster phantom, extraction fixes at the source, live tracker overlay

Adversarial review of the 0.9.19/0.9.20 layers (12 agents, 9 confirmed, 0
refuted) plus a user question that turned out to be the sharpest one of the
session: "can obstacle JUMPS make the velocity prediction wrong?" Answer
before this round: yes, one way -- a clustering SPLIT could synthesize a
sustained, perfectly coherent, fully gated 0.45 m/s phantom on a static
object (same-frame double-merge overwrote the raw-obs anchor; the constant
cross-centroid vector then read as motion). Fixed structurally: every track
absorbs AT MOST ONE observation per frame -- a split now spawns an adjacent
track and the isolation gate silences both. Jump semantics are now pinned by
tests: in-range jumps can only produce a same-direction SLOWDOWN (degrades
toward frozen-world), direction-flipping samples close the gate that tick,
out-of-range jumps spawn silent new tracks. Wrong-direction velocity cannot
persist.

Also in this round:
- association picks the nearest ELIGIBLE track (the young boost was shadowed
  by marginally-nearer mature tracks); displacement gate rate-normalises by
  actual anchor age; 18 memory columns got named constants; variant alias
  tables unified in config.V1_SPECS/V2_SPECS (three drifted copies).
- benchmark integrity: protocol params pinned at call sites, _stamp() marks
  dirty trees, static-tier benchmark animations replay with substeps=1
  (metrics byte-identical to the scored episode); tracker_eval uses
  independent per-episode RNG and the tests import ITS harness (constants
  had already diverged between the two copies). Full 4-row table rebuilt at
  one clean commit (the review caught rows mixed across tracker versions --
  exactly the trust failure the table exists to prevent).
- GUI: the top-down view now draws the tracker overlay LIVE -- purple
  velocity arrows with m/s labels on anything the tracker gates as moving,
  plus dashed +1s/+2s constant-velocity ghosts when a *_t policy is driving
  (the operator sees what the planner believes). The same data lands in
  run.json for post-run analysis.
- **perception extraction fixed at the source** (obstacle_perception; needs
  an on-car rebuild): pure logic split into ROS-free clustering.h with a
  workstation g++ test. Odom-anchored wall chunking kills the dominant
  phantom source (interior chunk slide 0.090 -> 0.015 m/tick; 0.090/tick =
  0.27 m/s apparent velocity, matching the recorded phantom p90 0.245);
  Kasa arc fit recovers a compact object's TRUE centre (approach drift
  0.05-0.10 m -> 0.0000); merge_circles rejoins split objects; temporalFilter
  holds flickering circles 2 frames (the measured 28% missed-update rate was
  flicker deletion). The tracker gates stay as defence-in-depth.

## 0.9.20 - 2026-08-05 - perception hardening: the _t tracker now survives REAL sensor data

The gap: replaying 220 recorded on-car ticks (all-static scenes, the 0.9.14
sessions) through the tracker showed the sim-tuned gates leaking PHANTOM
velocities on 95% of ticks -- p50 0.107 / p90 0.245 / max 0.572 m/s within
1.5 m, overlapping pedestrian speeds. Root causes measured, not guessed: wall
clusters arrive as chains whose centroids SLIDE as the viewpoint moves, and
adjacent clusters cross-associate (churn). NOT timestamp skew (a zero-yaw run
was equally affected). Deploying *_t like that would have meant dodging ghost
walls. Method: an offline harness (real frames = labeled negatives; movers
injected into the SAME frames with MEASURED noise -- centre sigma 0.02 m,
radius 0.01 m, 28% dropout, fitted from calm tracks -- as labeled positives),
feature-separation analysis, then gate selection by grid search.

- **Four mechanism-targeted gates** in ObstacleField._vel (config knobs with
  the tuning data cited): COHERENCE >= 0.90 (|sum of raw velocity samples| /
  sum|samples|, decayed -- churn flips direction, movers do not), ISOLATION
  >= 0.35 m (walls come in chains; a mover crosses open floor), NET
  DISPLACEMENT >= 0.15 m within a [0.75, 1.5] s ping-pong-anchored window
  (slide wanders; a mover goes somewhere), sightings 3 -> 5. Plus a young-track
  association boost (+0.07 m for <3 sightings): a 0.45 m/s mover's first
  re-sighting lands beyond merge_dist and used to fragment forever.
- **Result on the recordings** (REAL class end-to-end): phantom events 2685 ->
  80 (-97%), and the two clean A/B runs read 1 and 0; injected movers at
  0.25/0.35/0.45 m/s acquire 67-80% of episodes in ~2-3.7 s median. Honest
  residue: the recordings had the OPERATOR in the room -- walking-speed
  residuals in the later sessions may be true movers; a controlled no-human
  recording is part of the live session.
- **Committed evidence**: tests/data/real_frames.json (56 real ticks from the
  0.9.14 A/B runs) + tests asserting <=1 phantom event on them forever and
  injected-mover acquisition <= 4 s; scripts/tracker_eval.py re-runs the whole
  harness (fixture by default, --logs for full recordings) whenever the
  tracker changes. Measured noise knobs recorded in the script constants for
  the eventual TestField perception fit (protocol still runs noise at 0).
- Benchmark rows for mpc_grid_t / mpc_vw_t refreshed (the gates buy real-world
  safety with ~1.7 s acquisition latency in sim too -- see BENCHMARKS.md for
  the updated cells; plain rows untouched, gates only affect velocities()).
- Cost: none for the frozen-world variants (association boost aside, gates act
  only on the gated velocity output), none for static worlds (velocity reads
  0 exactly -- byte-equivalence tests still pass). 31 tests.

## 0.9.19 - 2026-08-04 - the fixed benchmark: one eval protocol, one tiered set, one committed table

`BENCHMARKS.md` + `benchmarks.json` + `scripts/benchmark_table.py` +
`mpc_baseline/benchtable.py`. Every current and future policy is evaluated on
the SAME protocol and SAME set, and the results live in one committed table --
no more hand-edited README cells or scratch-log numbers that are comparable
only on trust.

Protocol v1 (versioned; any change bumps it and marks old rows STALE):
registry-built policies at the live profile (magnitude 40 -- the plug-in
contract), live-faithful execution EVERYWHERE (the perfect-execution world has
mis-ranked controllers twice: w_cont 0.9.3, v2's deterministic limit cycles --
it is not part of the protocol), 20 seeds, footprint-contact collisions, and a
DIFFICULTY-TIERED set graded by construction (not by measured hardness, which
would drift as policies improve):

    L1 static-open    realistic suite, 4 worlds, B=3 m           ( 80 eps)
    L2 static-tight   tight suite, 6 worlds, partly beyond the
                      turn radius                                 (120 eps)
    L3 dyn-single     4 mover archetypes + 6 random single-mover
                      cases (gen seed 1003)                       (200 eps)
    L4 dyn-complex    occluded_oncoming + 9 random clutter cases,
                      1-2 statics AND 1-2 movers (gen seed 1004)  (200 eps)

The whole thing is deterministic per code version, so
`benchmark_table.py --check KEY` re-runs a row and byte-compares it: the table
doubles as a full-stack regression detector.

First edition (commit efbd2b5 rows):

    mpc_grid     1.000/0.000  1.000/0.000  0.820/0.180  0.865/0.135  overall 0.895/0.105
    mpc_grid_t   1.000/0.000  1.000/0.000  1.000/0.000  1.000/0.000  overall 1.000/0.000
    mpc_vw       0.850/0.113  0.350/0.600  0.725/0.270  0.485/0.450  overall 0.587/0.375
    mpc_vw_t     0.850/0.113  0.350/0.600  0.905/0.040  0.855/0.110  overall 0.770/0.185

Reading the table: the vw rows' L1/L2 cells are IDENTICAL between plain and _t
-- the static byte-equivalence guarantee, now visible in published numbers.
The difficulty gradient orders correctly for the differential-drive variants
(L1 > L3 > L4, L2 hardest = physical limits). Two caveats recorded: the
static cells run the LIVE profile, so they differ from the README's historical
sim-profile rows by design; and mpc_grid_t sits at the CEILING (1.000 across
all 400 dynamic episodes -- the new tier generator seeds happen not to produce
the wall-bounce cases that caught it in the 0.9.17 battery), so a future
policy that beats it needs protocol v2 with a harder L4 (faster movers,
mid-course bounces, perception noise once fitted). 28 tests (set stability,
tier construction, update/check round-trip, tamper detection).

## 0.9.18 - 2026-08-04 - smoothness screening: L1 cross-track shipped, direction penalty rejected-with-data

Two behaviour reports from watching the field animations: vw meanders beside
the A->B line after a mover recedes (slow return to path), and grid sometimes
double-backs a hop before continuing. Both got a mechanism and a 20-seed
screening sweep (README rule: smoothness tuning is screened against the
disturbed/live-faithful loops -- the sweeps that killed w_cont in 0.9.3 now run
in minutes instead of on the car).

**Shipped: `CostConfig.w_track_l1 = 1.0`** (linear cross-track, vw variants).
Diagnosis: the quadratic w_track's gradient vanishes near the line, so the
last few cm of return cost almost nothing -- hence the meander. The L1 term
has CONSTANT pull near the line while growing slower than the quadratic far
out, so it does not re-create the fight-the-wide-detour failure that capped
w_track at 4.17. Sweep (w_l1 = 0 / 0.5 / 1.0 / 2.0): field tail-wander
0.192->0.159 (vw) and 0.133->0.106 (vw_t) monotonically; field
success/collision neutral-to-better; realistic 1.000 unchanged; tight stress
suite pays ~1pp. Applied to BOTH vw variants so the _t ablation stays
single-variable. Republished numbers: tight v1 0.717/0.283 default,
0.442/0.533 disturbed; realistic disturbed 0.938/0.037; field vw 0.553/0.410,
vw_t 0.873/0.110 (grid rows untouched).

**Rejected with data: hop-direction smoothness for the FROZEN grid.** The
mechanism (cost.direction_cost, (1-cos) of the turn between hops and vs the
executing action -- small corrections nearly free, a double-back 2x a 90-deg
turn) is in the tree, but every sequence-term weight failed screening
(w=0.1: field success 0.740->0.667 at 21.8->15.5 deg mean turn), and the
history-only term still costs plain grid 6pp at w=0.05. The finding worth
recording: **frozen-world grid's double-backs are not noise -- they are
emergency corrections to a world model that jumps every frame**, and taxing
them taxes error recovery. The time-aware grid_t has no such excuse and gets
a genuine free lunch: `w_dir_hist = 0.1` -> mean turn 22.9->16.0 deg at
0.980/0.020 (better than its 0.977/0.023 baseline), tight-disturbed
1.000/0.000 intact. DEFAULTS stay 0 to keep the *_t-vs-plain ablation clean
and the static byte-equivalence tests meaningful; 0.1 is the documented
deployment option for mpc_grid_t (config.py/README). The real fix for grid's
unsmoothness is mpc_grid_t itself: reversal rate is 0.8% of transitions
either way, and the visible flapping largely disappears once the planner's
world stops jumping. 25 tests (direction-cost math incl. rotation exemption,
L1 near-line gradient).

## 0.9.17 - 2026-08-04 - time-aware baselines (mpc_grid_t / mpc_vw_t): constant-velocity obstacle prediction

The user asked for baselines that DO and DON'T consider obstacle motion over
time. There is no drop-in mature library for this stack (the mature
dynamic-obstacle planners -- teb_local_planner, Nav2's MPPI controller -- are
whole ROS navigation stacks in C++); the mature METHOD, and the standard
baseline in the dynamic-obstacle MPC literature, is constant-velocity
tracking + prediction. That is what shipped:

- **Tracker in the shared ObstacleField** (obstacles.py), so the live runner
  and every sim run identical code: memory rows carry [EMA pos, r, t, v, n,
  raw obs pos]; velocity = EMA of RAW-observation differences (differencing
  the EMA position overestimates a steady mover by exactly 2x), capped at
  vel_cap=1.0 m/s; association matches fresh circles against RAW-obs
  positions coasted by the UNGATED velocity (fixes the measured 0.30 m/s
  acquisition cliff -- the 0.40 m/s cross_fast archetype used to fragment
  into 1-sighting tracks and was silently untrackable). Planning only sees a
  velocity once a track clears vel_min_sightings=3 AND vel_deadband=0.04 m/s,
  which is what makes a STATIC world byte-identical between *_t and plain
  (regression-tested, including under 1 cm obs jitter): the *_t-vs-plain
  comparison isolates "considers motion" as the single variable.
- **Time-indexed obstacle cost** (cost.py): rollout step h scores against
  predict(t + (h+1)*dt + pred_extra_delay_s), capped at pred_cap_s=2.5 s of
  total extrapolation (beyond that a CV guess is fiction). pred_extra_delay_s
  is the loop's dispatch buffering: the live runner and the buffered
  (disturbed) sim start executing a plan ONE TICK after the frame it was
  planned from -- the adversarial review measured the uncorrected timeline at
  2 ticks (~0.15-0.23 m) of error in the CUT-IN-FRONT direction (one tick
  dispatch + one tick EMA-position lag; predictions now extrapolate from the
  raw observation, and runner.py/resolve_policy set the delay). These two
  timeline fixes took the field's archetype collisions from 3/18 to 0/18 in
  the review's A/B and are most of the final margin below.
- Registry keys `mpc_grid_t` / `mpc_vw_t` (GUI dropdown picks them up);
  `smoke/policy_run.py --variant` now accepts any registry key (1/2 map to
  mpc_vw/mpc_grid), so the *_t variants are drivable on the car.

TestField battery, 15 cases x 20 seeds, live-faithful (success / collision):

    mpc_grid   0.740 / 0.260        mpc_grid_t  0.977 / 0.023
    mpc_vw     0.537 / 0.440        mpc_vw_t    0.867 / 0.100

Per-case: grid_t has ZERO collisions on 13/15 cases (cross_slow 0.85 -> 0.00,
diagonal 0.65 -> 0.00, occluded_oncoming 0.20 -> 0.00 -- the coasted
prediction tracks a mover THROUGH occlusion); the residue (rand03 0.20,
rand07 0.15) is movers that bounce off a wall mid-approach, where constant
velocity is the wrong model -- the honest boundary of a CV baseline. With
tracking, obstacle memory flips back from liability to asset: *_t with
--mem 0 degenerates to plain (no cross-frame association -> no velocities;
grid_t collision 0.023 with memory vs 0.163 without). Known effects on the
PLAIN variants: coasted association slightly changes their dynamic-world
memory (the old stale-trail "phantom wall" made them accidentally
conservative near occluders -- plain numbers re-measured in the same battery;
static suites are unchanged, verified 120/120 episode pairs byte-identical).
Review: 11 agents; the timeline and acquisition-cliff findings confirmed by
instrumented reproduction, both fixed; 23 tests total pin the tracker math,
the gates, the delay threading and the static-equivalence guarantee.

## 0.9.16 - 2026-08-04 - TestField: dynamic obstacles (pymunk) + occlusion, live-parity plug-in arena

`mpc_baseline/testfield.py` + `scripts/run_field.py` + tests. The gap it closes:
every prior number -- both suites, all 4 table rows -- was a STATIC world, and the
sim was X-RAY (saw obstacles through obstacles). Any policy registered in the
registry plugs into the field by key; its inputs match the live PolicyRunner
(same Observation contract and ObstacleField code, 3 Hz tick, dispatch-next-tick
+ dead-time compensation, measured execution disturbance at the 1/3 s tick), plus
occlusion: a circle whose centre ray is blocked by another obstacle is not
sensed. Dynamic obstacles are pymunk rigid bodies (elastic, frictionless,
zero-gravity; bounce off walls, statics and each other), so seeded random cases
stay physically consistent; the car stays on the calibrated kinematics, and
contact is scored with the same footprint ruler as the guard, SWEPT through 10
sub-steps per tick so a crosser cannot pass through the footprint between ticks.
Battery: 5 archetypes (crossers, oncoming, occluded-oncoming behind the box) +
10 seeded random cases, 20 seeds each.

Findings (300 episodes per policy per config):

    default (live-faithful):  v2 0.757 / 0.243     v1 0.530 / 0.437
    --mem 0 (no memory):      v2 0.837 / 0.163     v1 0.580 / 0.380
    --perfect-exec:           v2 0.867 / 0.133     v1 0.557 / 0.443

- **Neither baseline is robust to intercepting movers** (cross_slow: v2 0.85
  collision, v1 1.00). The rollouts assume a frozen world; nothing estimates
  obstacle velocity. This is the real capability gap the static suites hid --
  a dynamic-capable policy must extrapolate motion (frames are there to diff).
- **The 1.5 s obstacle memory is a LIABILITY in dynamic worlds**: --mem 0 is
  safer for both variants -- a mover drags a trail of stale remembered circles
  and the soft wall pins to where the obstacle no longer is. Quantified at
  0.243->0.163 (v2) and 0.437->0.380 (v1) collision.
- **v1's dynamic collisions are mostly NOT execution disturbance** (0.443 under
  --perfect-exec): planning against a one-frame-old static world is the cause.

Correctness fixes that fell out of the field's adversarial review (9 agents, 6
confirmed, 0 refuted):

- **A zero command now BRAKES in the disturbed sim** (sim.py). The yaw-lag state
  used to take its additive noise on (0,0,0) commands too, so a stopped car
  random-walked its heading ~60 deg/10 s -- the fit is from DRIVING ticks, and
  the real runner's hold path clamps the wheels. Stop-and-wait policies were
  being penalized for a phantom. Nudged the disturbed rows: tight v1
  0.492/0.500 -> 0.475/0.517, realistic v1 0.950 -> 0.925 (README updated).
- **Disturbed mode forces the live tick for EVERY policy** (eval.resolve_policy,
  now the single construction path shared by run_variant/run_policy/TestField):
  a third-party velocity cfg with mppi.dt != 1/3 used to run the "live-faithful"
  mode at its own cadence; now mppi.dt/rollout_dt are overwritten to the tick,
  exactly like runner.py does on the car.
- occluded archetype redesigned (the original crosser left the shadow almost
  immediately -- the case named for occlusion never exercised it; now an
  oncoming mover hides dead behind the box for seconds), --mem now rides along
  to --anim (the gifs used to depict a DIFFERENT episode than the table),
  goal_dist < 2 m is rejected with a clear error (used to crash or silently
  launch movers outside the arena), --policy de-duplicates, --anim degrades
  gracefully without matplotlib, run_case built-in specs default to the live
  profile. pymunk added to mpc dependencies; 15 tests total.

## 0.9.15 - 2026-08-04 - eval harness audit: honest collision metric, runnable protocol, guard defaults

A 13-agent adversarial audit (4 review dimensions, every finding independently
verified by execution, 8/8 confirmed, 0 refuted) found the PLANNER correct -- the
mecanum mixing, plant inversion, AR(1) sampler and dt-integrated costs all check
out numerically, and every published number reproduces exactly -- but the
EVALUATION HARNESS was quietly overstating variant 1 and could not be re-run by
anyone else. All fixes below; regression tests in `mpc/tests/` pin each one.

- **Collision = footprint contact, not centre penetration** (`sim.py`). The sim
  flagged a collision only when the footprint CENTRE entered the obstacle circle,
  so an episode could end "success" with the obstacle surface 12 cm inside the
  car -- states the live guard estops on. Under the honest metric (edge reaches
  `robot_radius`, exactly the live guard's test) the v1 tight-suite numbers move
  0.992/0.008 -> 0.725/0.275 (default) and 0.767/0.167 -> 0.492/0.500
  (disturbed); same trajectories, honest ruler. v2 is 0.000 contact under BOTH
  metrics across all 400 episodes -- the "v2 is the robust baseline" ranking not
  only survives, it strengthens. README table republished from fresh 20-seed runs.
- **The published protocol is now runnable**: `benchmark.py --seeds` (default 20)
  reproduces the README table as-is -- it used to run seed 0 only, printing
  exactly the single-seed numbers the README warns against, and the 120-episode
  table required a hand-written loop that was never committed. The
  realistic-regime suite is now IN THE REPO (`sim.realistic_scenarios`,
  `--suite realistic`): the old "realistic 1.000/0" row came from ad-hoc worlds
  nobody could re-run. Re-measured: 1.000/0.000 default (both variants),
  v1 0.950/0.025 disturbed, v2 1.000/0.000 -- the claim was honest, and now it
  is checkable. Plots also stop drawing goal B at a hardcoded 1 m.
- **`run_policy` seeds the policy** (`eval._seed_policy`). The documented
  third-party comparison path (`benchmark.py --policy X`, `E.run_policy`) never
  passed a seed to the policy -- `build_policy`'s signature is a fixed contract
  without one -- so undisturbed multi-seed evaluation of ANY registered model was
  N byte-identical copies presented as N trials. This is the same bug class
  0.9.5 fixed for `run_variant`; it had recurred one path over. Now
  `seed()`/`rng` on the policy is seeded per episode.
- **Disturbed eval runs discrete policies at the real tick** (`eval.py`). The
  "live-faithful" v2 row closed the loop at step_duration=0.5 s -- a 2 Hz cadence
  the car never runs, with noise constants fitted per 1/3 s tick injected per
  0.5 s step -- and the offline `--policy mpc_grid --disturbed` screening path
  planned 0.5 s hops while executing 1/3 s ones (the exact 50% over-prediction
  the runner's `rollout_dt` was introduced to fix, surviving offline). Both now
  execute AND roll out at `TickConfig.period`, like the runner. v2 disturbed
  stays 1.000/0.000 at the true cadence, so no prior conclusion flips.
- **`policy_run.py` collision guard defaults ON** (`--no-guard` to disable). The
  README-recommended live command still shipped the pre-incident guard-off
  default with the already-repudiated "redundant, false-tripping" comment -- the
  GUI default was flipped after the 2026-07-25 penetration incident, this CLI
  was missed.
- **Perception outage no longer traps `run()`** (`runner.py`). The NOFRAME
  branch `continue`d before the abort/timeout/link checks, so with perception
  dead the loop held forever: GUI Stop was ignored, `timeout_s` unenforced, and
  the link-loss estop unreachable during the exact outage it exists for (the car
  itself did brake via the keep-alive; the workstation loop was immortal). The
  exits now run inside the NOFRAME branch too.
- Known remainders, deliberately unfixed: `Policy.reset()` still has no caller
  (fresh-instance-per-run remains the contract), perception noise
  (`noise_xy`/`dropout`) still unexercised in both eval modes, the collision
  guard still fires on current-frame circles only, and v2's undisturbed rows are
  deterministic (effective n=6 -- now documented in the README instead of
  implied to be 120 samples).

## 0.9.14 - 2026-08-04 - live A/B go-around: both variants clean; regime claims hold

Operator-run A/B with a real obstacle (~1 m ahead, B = 3 m, magnitude 40, guard on),
variant 1 then -- after turning the car around, so the REVERSE course -- variant 2.
`output/2026-08-04_17-43-38-523` (v1) and `_17-44-09-371` (v2).

    v1: reached 9.56 s, min clearance 0.322 m, fidelity xy 1.015 / yaw 0.686
    v2: reached 9.58 s, min clearance 0.168 m, fidelity xy 0.844
    both: 0 skipped frames, 0 guard trips, dt ~0.330

- **The disturbance model's realistic-regime claim is confirmed on the car**: both
  variants go around cleanly at realistic spacing, as the disturbed sim predicts.
- **xy fidelity 1.015 at 10.8 V** -- the battery sat almost exactly at the 10.5 V
  calibration point and the plant model tracked to 1.5 %. Yaw 0.686 is the best
  recorded (the reseated ch341 cable may be helping the whole serial path).
- The tight-regime prediction (v1 collides 0.167 under yaw lag) remains UNTESTED --
  this course was not tight. The v1-vs-v2 clearance difference (0.322 vs 0.168,
  opposite of the sim's ordering) is CONFOUNDED: v2 ran the reverse course after a
  manual turn-around, n=1 each. No conclusion drawn.

## 0.9.13 - 2026-08-04 - execution-disturbance model: offline tuning can now be trusted

The gap this closes: the undisturbed sim mis-ranks controllers. w_cont=0.6 looked
FREE offline (1.000 success, less jitter) and went 0/2 on the car -- in a
perfect-execution world, a controller that cannot correct never pays for it. Every
smoothness/robustness tuning question was blocked on this.

**Model, FIT from 298 clean on-car ticks** (six 2026-07-25 runs, wedge-contaminated
segments excluded; command-vs-measured pairs from the tick logs):

    v: multiplicative per-tick noise    N(0.946, 0.268)
    w: first-order lag + noise          tau = 0.48 s (~1.4 ticks), R^2 0.885
                                        (zero-lag model: 0.700), noise 0.171 rad/s

The 0.48 s yaw lag IS the quantified "command flips sign every 1-2 ticks and the
chassis cannot follow" -- the mechanism behind yaw fidelity 0.6.

**Implementation:** `DisturbanceConfig` (config.py) + execution filtering in
`KinematicSim.step`; `run_episode(buffered=True)` reproduces the live runner's loop
shape (one-tick dispatch buffer + dead-time compensation); `eval.run_variant/
run_policy(disturbed=True)` and `benchmark.py --disturbed` switch the whole stack to
the live-faithful evaluation.

**Acceptance (reproduce the w_cont incident):** in the era-faithful open-corridor
setup, w_cont=0.6 costs 15 % timeouts even UNDISTURBED once the loop is buffered
(1.000 -> 0.850) and stays degraded disturbed -- the original "looked free" verdict
was HALF the missing buffer, half the missing disturbance. The car's exact 0/2 is
n=2 and was not chased (10-15 %% simulated timeout rates make 2-for-2 plausible;
over-fitting a two-sample target would be worse than honest under-shoot).

**New 20-seed baseline, tight suite:**

                       v1              v2
    undisturbed    0.992 / 0.008   0.833 / 0.000
    DISTURBED      0.767 / 0.167   1.000 / 0.000

Two findings with face validity: variant 1 pays 0.167 collisions for the yaw lag in
tight scenarios -- the measurable target for the yaw-tracking work -- and variant 2,
which steers by holonomic translation and never needs the lagged axis, is the more
robust baseline under realistic execution, matching on-car intuition.

## 0.9.12 - 2026-08-04 - third calibration point; ch341 root cause; car_base respawn

(Entry written after the fact -- the work landed in 8db0e63/49d665c with full commit
messages but no changelog section.) 11.1 V calibration validated the voltage model:
arm 0.194/0.198/0.194 across three voltages, speed extrapolation within 2.5 %% at
magnitude 40; shipped constants unchanged. Root cause of the MCU wedges found in
dmesg: the ch341 USB-serial adapter spontaneously drops off the bus (marginal
contact, vibration-correlated); car_base now has respawn="true" (deployed), the
per-tick imu_frozen() monitor covers the freeze-without-death case, and
calib_model discards samples contaminated mid-measurement. The user reseating the
cable stabilised the link (6+ min without a drop, previously minutes apart);
lesson recorded: after a drop while IDLE, car_base holds a stale fd without dying
-- restart car-ros; respawn only fires on the write-error death path.

## 0.9.11 - 2026-08-04 - per-tick MCU wedge detection

Closes the top item left open on 2026-07-25: both MCU wedges struck MID-RUN, after
the startup `sensors_live()` gate had passed -- the second one left the car driving
blind for 44 ticks (14.7 s) with `real/cmd x0.00` while the planner kept commanding.

- **`CarClient.imu_frozen(window_s=1.2, min_msgs=8)`** -- non-blocking, backed by a
  persistent lightweight IMU tap (~3 KB/s at 10 Hz; nothing like the 43 KB/s /scan
  we dropped). True when the last 1.2 s of gyro-z samples are BIT-IDENTICAL: a real
  gyro dithers even parked (7-10 distinct values per 30 samples, measured), so 8+
  identical readings only happen when the serial read path is stuck republishing
  one value -- the exact signature of both wedges. Not-enough-data and
  stale-messages both return False: startup must not false-trip, and "no messages"
  is link loss, which link_ok()/stale checks own.
- **The runner checks it EVERY TICK** (under the existing `check_sensors` switch),
  before anything reaches the wheels. A hit emits a `WEDGE` tick-log line and hard
  estops -- not a hold, because a hold trusts the next frame and there will not be
  one. The estop fast path (raw-PWM zeros on /wheel_cmd) works precisely in this
  failure mode: the wedge kills the READ path while writes still reach the motors.
- Detection latency, verified with realistic 10 Hz timestamps: **fires 1.2 s after
  freeze onset = 3.6 ticks** (~0.43 m at v_max). Against 44 ticks of blind driving
  last time. Verified against the recorded wedge signature (fires), the healthy
  stationary capture (does not), startup with 4 samples (does not), and a stalled
  ring (does not -- that is link loss).

Offline suite unchanged: v1 1.000, v2 0.833.

## 0.9.10 - 2026-07-25 - on-car validation: 5 m reached, model fidelity 0.91

The run that closes the day. Variant 1, magnitude 40, B = 5 m ahead, via the GUI,
battery 9.9 V. `output/2026-07-25_22-05-36-246/`.

    reached True   final distance 0.143 m   16.57 s   51 ticks
    flags  REACHED=1  WPIN=4        (no ZERO, no NEAR, no STALE, no NOFRAME)
    gd     5.000 -> 0.143, strictly monotone

**Model fidelity -- what the whole calibration was for:**

    cumulative measured/commanded   translation 0.910   yaw 0.599
    per-tick translation ratio      median 0.86  (p10 0.69, p90 1.14)

0.910 is the number to keep. The pack was at 9.9 V while the shipped constants were
measured at 10.5 V, and the voltage model predicts about +10 % of over-prediction
there -- which is what came out. **The affine plant model and the yaw feedforward
hold up on the real car**, and the offline figures now have a physical counterpart.

Yaw fidelity 0.599 is the known, non-blocking one: the command changes sign on 31 %
of ticks and the chassis cannot follow that. It does not stop the car reaching B.
Fixing it needs a disturbance model in the sim first -- see 0.9.3, where the obvious
fix looked free offline and went 0/2 on the car.

**Tick health:** dt 0.331 mean (0.225-0.384), **0 skipped frames**, planning 78 ms
median / 93 ms peak against the 333 ms budget. No MCU wedge across all 51 ticks.

The run's header shows `collision_abort False`: the guard default became True in
0.9.6 and the GUI instance that flew this run carried that change, so it was
unchecked in the UI.

### Still open

- **Sensor liveness is only checked at startup.** Both of today's MCU wedges appeared
  mid-run (0.9.9); the check has to run every tick and estop on a frozen feed.
- **Yaw tracking 0.599** -- needs the sim disturbance model first.
- `ZERO` still appears transiently near the goal on some runs (0.9.5).
- `pwm_per_mps` has never been measured above 10.5 V. Per 0.9.7 that does not matter
  at magnitude >= 30, so this is low priority.

## 0.9.9 - 2026-07-25 - the MCU wedge recurs mid-run; estop measured and made verifiable

### The wedge is not a one-off, and it happens DURING a run

A verification run at magnitude 40 after a power cycle: the sensor gate PASSED at
startup (imu dithering, gyro bias 0.0009 rad/s, odom yaw drift -0.03 deg/3 s), the
first 18 ticks were healthy -- `real/cmd` 0.74-1.08, `gd` falling 2.500 -> 0.941,
1.58 m travelled -- and then at t=5.88 s the car stopped dead. The remaining 44
ticks show the pose frozen at (+1.69, +0.21) and `real/cmd x0.00` while the planner
kept commanding v=0.361. Checked afterwards: the MCU read path was wedged again
(imu frozen at 0.171509228, odom yaw drifting -19.6 deg / 2 s).

That is twice in under an hour, both times around 10 V. **The startup gate is not
enough -- the check has to run every tick**, and the `real/cmd` column is what
caught it: a run of `x0.00` is "commands sent, car not moving" and nothing else.

### The e-stop, measured

I claimed the estop had not worked because `car-ros` was still `active` when I
checked. **That was wrong -- I measured too early.** The path takes 5.1 s end to
end, and I checked after ~1 s. Timed properly:

    bare SSH handshake                      0.85 s
    first zero actually reaching the motors 1.23 s   <- the number that matters
    whole script (3 s of zero-blasting)     4.2 s

The order inside `estop.sh` is forced: `/dev/myserial` is exclusive and `car_base`
holds it, so the service must stop before the port can be opened at all.

Fixed:
- **`CarClient.estop()` blasts raw-PWM zeros on `/wheel_cmd` first** (milliseconds,
  down the link car_base already has open) and fires the SSH teardown as the
  backstop. `/wheel_cmd` maps to `set_motor`; the 2026-07-22 runaway that velocity
  commands could not halt was `/cmd_vel`, which goes through the firmware's loop.
- **It now confirms.** It was `Popen` with stdout AND stderr discarded, returning
  None whether it succeeded, the key was missing, or the car was unreachable -- on
  the last line of defence. It now waits (default 8 s), checks for "motors are OFF"
  and returns True/False, printing PULL THE POWER on failure.
- **`estop.sh` is now in the repo** (`car_ros/estop.sh`) instead of existing only
  on the car, with a backup of the car's copy taken before deploying. Dropped its
  probe of the retired `scan_bridge` port 9870, which always failed and cost a
  python3 startup; 5.1 s -> 4.2 s total.

### The sensor gate's first live use found a bug in the gate

It failed a healthy car. Encoders and `/battery_v` were flagged FROZEN because they
held one value for 3 s -- which a stationary car's encoders and a quantised voltage
reading are supposed to do. The IMU is the only MCU feed that must dither when the
car is still, and it is what actually caught the real fault. The distinct-value test
is now IMU-only; encoders and battery are only required to be publishing. Verified
both ways: healthy parked car PASSES, the recorded wedged signature still FAILS.

## 0.9.8 - 2026-07-25 - INCIDENT: wedged MCU read path; sensor-liveness gate added

**What happened.** An attempted re-calibration produced impossible numbers -- the
in-place yaw rate came out at 1.55 rad/s for *every* PWM from 22 to 70, PWM 40 and
60 agreeing to three decimals. Diagnosis: the MCU serial READ path had wedged while
the WRITE path kept working. Every MCU-sourced topic was frozen at a single value
over a 6 s sample:

    imu   60 msgs, 1 distinct value   stuck at -1.0360 rad/s
    enc   60 msgs, 1 distinct value   stuck at [-1049, -1343, 1395, 1273]
    batt  15 msgs, 1 distinct value   stuck at 10.00 V

Consequences, all of which I initially misread:

- odom yaw ramped **+178 deg in 3.0 s with the car standing still**, integrating the
  stuck gyro. The "1.55 rad/s at every PWM" was that ramp, not the car turning.
- the encoders never moved, so I concluded the wheels were not turning -- **they
  were**: the operator saw the car spinning in place the whole time.
- `link_ok()` reported healthy throughout, because it only asks whether the voltage
  is plausible and 10.00 V is plausible. A frozen reading passes it.

The car was hard-estopped (`estop.sh`, new serial, writes 0 directly and kills
car-ros) and confirmed stopped.

**My error was continuing to command motion while the data contradicted itself.**
PWM 40 and 60 producing identical yaw, encoders not counting, and voltage showing
zero sag under a four-motor load are each individually enough to stop and diagnose.
Sensors first, motors second.

**The gate.** `CarClient.sensors_live(secs=3.0, gyro_still_max=0.05)` -- call with
the car stationary; it subscribes temporarily, leaves nothing running, and returns
`(ok, report)`. Two independent tests, because either alone can be fooled:

- every MCU topic must produce **more than one distinct value**; a wedged feed is
  bit-identical while real sensors always dither;
- the gyro must read ~0 while the car is still.

`PolicyRunner` now refuses to start when it fails (`check_sensors=True`), before any
motion and before the link-loss interlock, and `calibration/calib_model.py` refuses
to drive. Both say to power-cycle: restarting car-ros may not clear a wedged serial.

**`LiveConfig.magnitude` 30 -> 40**, the value the car is actually validated at (five
5 m runs, final distance 0.05-0.11 m) and the only magnitude the yaw feedforward was
measured at end to end. v_max 0.361 m/s, full yaw to w_max 1.2, turn radius 0.30 m.
Tight suite over 20 seeds is 0.975 at 40 against 0.992 at 30 -- within noise, and 40
carries the on-car evidence.

Re-calibration is still OUTSTANDING: the pack was at 10.00 V, below the 10.5 V the
shipped constants came from, so it would not have pinned `pwm_per_mps` at a working
voltage even had the link been healthy.

## 0.9.7 - 2026-07-25 - default magnitude 30; repo-wide stale-claim sweep

**`LiveConfig.magnitude` 20 -> 30.** The old 20 was chosen when the proportional
model claimed it meant 0.10 m/s. Under the calibrated plant it is 0.083 m/s with
only 0.176 rad/s of yaw (turn radius 0.47 m), barely above the magnitude-17.4 floor
where yaw becomes exactly zero. Tight suite over 20 seeds:

    mag 20   v_max 0.083   yaw <= 0.176   radius 0.47 m   success 0.333   collide 0.000
    mag 25   v_max 0.153   yaw <= 0.520   radius 0.29 m   success 0.842   collide 0.000
    mag 30   v_max 0.222   yaw <= 0.863   radius 0.26 m   success 0.992   collide 0.008
    mag 40   v_max 0.361   yaw <= 1.200   radius 0.30 m   success 0.975   collide 0.025

Failures at 20 are timeouts, not collisions -- the car is simply too slow to steer.
The GUI spinbox now reads the same constant instead of carrying its own default, so
there is one number. 40 remains the value the five on-car 5 m runs used.

**Stale-claim sweep.** Six agents checked every quantitative claim in the code
comments and docs against the current code; 38 validated corrections applied, each
with its `old` text verified unique before replacement. The session re-measured the
robot and rewrote the controller repeatedly, and every round left numbers behind.
Representative:

- `config.py` module docstring named `sim_config()`/`live_config()`, which have
  never existed (the factories are `sim_config_v1/_v2`, `live_config_v1/_v2`).
- The calibration block labelled the 10.5 V row "shipped" including its arm 0.198,
  but the shipped arm is 0.196, the midpoint of the two sessions.
- "the arm is a CONSTANT (0.216/0.196/0.184)" contradicted its own numbers: that is
  a monotone -15 % drift with PWM. It is now stated as "nearly stops drifting",
  against the -43 % the proportional model produced.
- `noise_tau = 0.8425` was documented as "the shipped beta=0.7 at the default tick";
  `exp(-(1/3)/0.8425)` is 0.673. 0.9346 would give 0.7. Comment corrected, value kept.
- `kinematics.py` still quoted the superseded 9.8 V fit `(PWM-17.0)/73.4` with
  +-0.003 residuals; shipped is `(PWM-14.0)/72.1` with +-0.001.
- "car_base uses the identical mix (its WZ_ARM == arm)" -- car_base_node still has
  WZ_ARM 0.5 against our 0.196; the FORM matches, the value has not since 0.9.0.
- `obstacles.raw_min_distance`'s docstring named two consumers that no longer call it.
- The tick-log legend documented a **`HOLD` flag that no code path emits** (the real
  one is `STALE`), and the example log lines were missing two columns the log emits.
- Several CLI docstrings still advertised `--deadzone-pwm` (deleted) and `plan_dt
  0.25` (the period of the deleted `LiveConfig.plan_rate`).

A mechanical check now confirms the tick-log legend and the flags the runner emits
are the same set in both directions.

## 0.9.6 - 2026-07-25 - safety: the guard shipped OFF, and the tick delay was uncompensated

The review's synthesis, after re-measuring everything against 0.9.5. Three of these
are consequences of changes made earlier in this same session.

### SAFETY: the GUI shipped the imminent-collision guard unchecked

`gui/car_console.py` set the checkbox to False, overriding `LiveConfig.collision_abort
= True`. The operator's run `output/2026-07-25_20-01-44` is the evidence that it
matters -- measured from its own tick log against `robot_radius` 0.13 m:

    tick 30   nearest 0.118   dmem 0.104   footprint  26 mm inside   v=0.361 w=-0.539
    tick 31   nearest 0.089   dmem 0.026   footprint 104 mm inside   v=0.361 w=-1.158
    tick 32   nearest 0.097   dmem 0.004   footprint 126 mm inside   v=0.361 w=+0.293

126 of 130 mm of the footprint inside an obstacle circle, at `v_max`, for three
consecutive ticks (1 s), with nothing intervening. The guard threshold is
`robot_radius + collision_margin` = 0.130 and would have fired on all three. Now
checked by default; `collision_estop=False` already makes it a soft stop, so a false
trip costs a stop, not a killed `car-ros`.

### The one-tick dispatch delay was compensated by nobody

0.7.0 introduced command buffering (plan at tick N, dispatch at N+1) but neither
policy advanced its start state by the command already in flight, and
`sim.run_episode` applies a plan in the tick it was planned -- so the offline suite
never modelled the delay it had introduced. At the live caps that dead time is
**0.12 m of travel and 23 deg of heading per tick**, against `robot_radius` 0.13 +
`extra_margin` 0.10. Reproduced over 120 episodes at the live config:

    no buffer (what the sim modelled)   success 0.975   collide 0.025
    buffered, UNcompensated (the car)   success 0.892   collide 0.033
    buffered + compensated (fixed)      success 0.975   collide 0.025

The runner now plans from `rollout_body(pose, cmd_in_flight, period)` rather than the
measured pose. The field update still uses the MEASURED pose -- that is where the scan
was taken. Offline and live now agree numerically again, which is why the suite is
left unbuffered rather than duplicating the compensation in two places.

### Below magnitude 17.4 the achievable-yaw clip is identically zero

0.9.5's fix clips the policy to `yaw_gain*max(0, mix_limit - yaw_deadband)`, which is
0 whenever `v <= yaw_deadband*arm/(1-min_inner_frac)` = 0.048 m/s, i.e. magnitude
<= 17.4. The planner would drive dead straight with no steering channel at all, and
the GUI's magnitude spinbox stepped by 5 from 0, putting that one click away.
`build_live_cfg` now refuses with the reason and the minimum usable magnitude, and the
GUI spinbox floor is 20.

### Two smaller ones

- **A discrete hop's life on the car was whatever the policy asked for.**
  `--step-duration 3.0` was accepted and equals `drive_action_node`'s own
  `max_duration_s`, so a runner killed between ticks left 3 s -- nine ticks -- of
  open-loop motion. The runner now hands the car `tick.action_duration` (0.5 s, 1.5
  ticks) regardless; `step_duration` stays a planning quantity.
- **`WPIN` could not fire at the shipped magnitude.** It tested `|w| >= w_max`, but
  after 0.9.5 the policy cannot emit more than 0.176 at magnitude 20 or 0.863 at 30,
  so the flag that means "cannot turn enough" was dead exactly where the car cannot
  turn. It now compares against the yaw actually available at that speed.

Offline unchanged: v1 1.000, v2 0.833 on the shipped config.

**Still open** (from the review's own list, and from the operator's runs): `ZERO`
appearing transiently near the goal; ~45 lower-severity findings, mostly stale numbers
in comments left by this session's repeated re-measurement.

## 0.9.5 - 2026-07-25 - review round: the yaw feedforward was a no-op below magnitude 40

A 5-lens adversarial review of `77b1c96..HEAD` returned 53 findings. The important
ones, fixed here.

### CRITICAL: the yaw feedforward cancelled itself except at magnitude 40

`Variant1Policy._perturb` clipped `|w|` to the MIX limit `(1-min_inner_frac)*v/arm`
(inner wheel stays forward). `yaw_feedforward` then adds `yaw_deadband` on top to
break roller scrub -- and is capped by that *same* mix limit. So at the limit the
compensation was clipped straight back to where it started:

    magnitude 20   policy asks 0.382   ->  car delivers 0.176   =  46 %
    magnitude 30   policy asks 1.019   ->  car delivers 0.863   =  85 %
    magnitude 40   policy asks 1.200   ->  car delivers 1.200   = 100 %

**The on-car validation used magnitude 40 -- the one value where it happens to
work.** Fixed by clipping the policy to the yaw the car can actually DELIVER,
`yaw_gain * max(0, mix_limit - yaw_deadband)`, so commanded == realised at every
magnitude (verified 100 % at 20/30/40/60). Magnitude 40 is unchanged, so the
five 5 m runs recorded in `output/2026-07-25_20-0*` remain valid evidence.

### Other fixes

- **Variant 2 rolled out at 0.5 s while the car executes one 0.333 s tick** -- the
  same model-vs-execution mismatch already fixed for variant 1, missed for the
  discrete path. `step_duration` now means only the hop's LIFE on the car (it must
  outlive a tick so the next tick supersedes it instead of it expiring into a
  brake); the new `rollout_dt`, set by the runner to the tick, is the model step.
- **The tick rate did not reach the model.** The GUI's "tick Hz (global)" spinbox
  and `--tick-hz` reached the runner but not `MPPIConfig.dt`, so changing the rate
  left the planner integrating at the old one. The runner now syncs both.
- **`eval.run_variant` never passed its seed to `Variant1Policy`** -- the repo's own
  multi-seed protocol was a no-op through the shipped API (our published numbers
  came from ad-hoc scripts that passed it explicitly). Both variants now get it.
- **A hold poisoned the divergence metric**: `_hold()` left the last dispatched
  command paired with the next tick's motion, fabricating a ratio and permanently
  skewing the cumulative figure quoted as calibration evidence. It now clears the
  pairing. `run()` also resets it per run, and `_moved` divides by the MEASURED
  interval rather than the nominal one.
- **A `real/cmd` ratio of exactly 0.00 rendered as "-"** (no data) because the code
  tested truthiness. 0.00 means THE CAR DID NOT MOVE, which is the most important
  thing that column can say.

### Offline after the fixes, 20 seeds (policy seed now genuinely varies)

    v1 tight suite   0.992 success / 0.008 collide / 0.000 frozen
    v1 realistic     1.000 / 0.000 / 0.000
    v1 live mag 40   1.000 / 0.000 / 0.000
    v2 tight suite   0.833 / 0.000 / 0.000

### From the operator's five 5 m runs (`output/2026-07-25_20-0*`)

All five reached B, final distance 0.05-0.11 m. Two things the tick log caught that
are still OPEN:

- **`NEAR` x3 in one run**: the nearest obstacle edge reached **0.09 m** and the
  remembered one **0.00 m**, against `robot_radius` 0.13 -- the footprint was
  overlapping a circle. The collision guard is OFF by default in the GUI, so
  nothing intervened.
- **`ZERO` x2** at `gd` 0.35 and 0.29, well short of the 0.15 m tolerance: the
  "car freezes" mode still appears transiently near the goal. It recovered on its
  own the next tick.
- `WPIN` on 13-36 % of ticks at magnitude 40, i.e. the policy often wants more yaw
  than `w_max=1.2` allows.

Not fixed: ~45 lower-severity findings, mostly stale numbers in comments and docs
from the several rounds of re-measurement this session.

## 0.9.4 - 2026-07-25 - the obstacle barrier saturated: one clip caused 41% of collisions

`obstacle_cost` computed `encroach = clip(obs_buffer - clearance, 0, obs_buffer)`.
The upper clip made the term a CONSTANT once the state was inside inflation.
Measured: `dC/d(clearance)` was exactly **0.00 /m** at every depth from -0.05 m to
-0.30 m, against +8.33 /m just outside. Nothing pushed the car back out.

It compounded with `collided` being one boolean per rollout: when EVERY sample
collides, the 1e6 is a common offset that cancels in the argmin, so the winner was
chosen on goal cost alone -- drive straight at the obstacle. Reproduced: with the
car inside inflation and B beyond the obstacle, the policy commanded
`v=0.220 w=+0.000`, i.e. dead ahead.

Fix is one character of intent: `np.maximum(buf - clr, 0.0)`, no upper clip, so the
penalty keeps growing with depth and the least-bad rollout is the shallowest.
Gradient now rises monotonically inside inflation (8.33 -> 28.33 /m). The same
constructed case now commands `v=0.220 w=-1.010` -- turning away.

**Offline, 20 seeds:**

| | before | after |
|---|---|---|
| v1 tight suite success | 0.583 | **0.983** |
| v1 tight suite collisions | **0.408** | **0.000** |
| v1 realistic (B=2.5 m) | 1.000 | 1.000 / 0 coll |
| v1 live cfg, mag 40 | - | 1.000 / 0 coll |
| frozen (never moved) | - | **0.000** everywhere |

That single clip was responsible for essentially all of variant 1's collisions, and
the "car freezes" failure went with it: standing still had been the only rollout
that escaped the flat penalty, and now escaping actually pays.

**On the car:** 3/3 reached, 5.3 / 4.9 / 4.9 s, 1.48 / 1.44 / 1.44 m against a
1.5 m goal.

Also fixed two bugs in the tick log's own `real/cmd` divergence metric, found while
reading these runs:
- it compared the pose delta with the command dispatched in the SAME tick, but that
  delta was produced by the PREVIOUS tick's command -- off by one. The measurement
  now happens before dispatch.
- it used the chord between consecutive poses against a commanded path length, which
  understates whenever the car turns. Now converts chord to arc.
Together these moved a run's cumulative translation ratio 0.811 -> 0.877 and yaw
0.535 -> 0.706. The residual gap is genuine transient response: the steady-state
sweep tracks 98-104 %, but per tick the command changes and the car is still
accelerating into it.

## 0.9.3 - 2026-07-25 - cross-tick control continuity: implemented, measured, left OFF

The yaw command reversed sign tick to tick faster than the chassis could follow.
Cause: `w_smooth` couples steps WITHIN one horizon, but nothing coupled consecutive
ticks, so each tick's argmin was free to jump anywhere. The standard fix is a term
penalising the first step's distance from the control already executing.

Implemented as `CostConfig.w_cont` + `Variant1Policy._u_prev`, in the same rate form
as `w_smooth` so it means the same thing at any tick rate.

**It works at what it was built for, and it is still shipped OFF (`w_cont = 0.0`).**

Offline it looked free -- 10 seeds x 4 realistic scenarios: w_cont 0.6 kept success
at 1.000 while cutting sign reversals 0.362 -> 0.292 per tick and mean |dw|
0.450 -> 0.335, and it even lifted the tight suite 0.583 -> 0.708. On the car it
failed:

    w_cont=0.0   reached 3/3   5.3 s    rev/tick 0.26   |dw| 0.37   yaw pinned 16 %
    w_cont=0.6   reached 0/2   14.3 s   rev/tick 0.06   |dw| 0.15   yaw pinned 42 %

The smoothing objective was met exactly as designed. The car simply stopped
reaching B: sluggish to correct, saturating at the yaw cap, both runs timing out.

**Why offline was misleading:** `KinematicSim` has no disturbances (`noise_xy` and
`dropout` are never set by `eval.run_variant`), so a controller that is slow to
correct pays nothing there. On the car it must correct against real perturbations
every tick, and this term is precisely what stops it. That is a sharper statement
of the sim/real gap already noted in `mpc/README.md`, and it is the concrete reason
to model disturbance in the sim before tuning anything else on smoothness.

The mechanism is kept, defaulted off, for that work.

## 0.9.2 - 2026-07-25 - no v-w coupling (my error); yaw DEADBAND found and compensated

**Correction first: there is no v-w coupling.** 0.9.1 reported that forward speed
fell to 53 % at maximum yaw. That was a measurement artefact of mine: `test_arc`
projected the displacement onto the START heading, so a car driving a curve at full
speed looks slow. Measuring along the ARC (chord and turn angle -> arc length), a
sweep of |w| at fixed v gives 95 / 101 / 101 / 104 / 107 % -- flat. The fitted
coupling coefficient is -0.047, i.e. noise with the wrong sign.
`rollout_unicycle`'s assumption that v is independent of w is FINE. `test_arc` now
reports arc speed.

**What that sweep did find is a yaw DEADBAND.** Commanded -> realised yaw at
v=0.361: 0.30 -> 38 %, 0.60 -> 67 %, 0.90 -> 75 %, 1.20 -> 92 %, fitting
`w_real = 1.079 * (w_cmd - 0.219)`. A percentage that RISES with magnitude is a
deadband; a time lag would cost the same fraction at every magnitude. Physically:
yawing a mecanum chassis scrubs the rollers sideways, which costs a threshold
torque of its own -- the per-wheel friction offset cannot cover it, because that
only makes each wheel's own speed correct.

- New `RobotConfig.yaw_deadband` (0.219 rad/s) and `yaw_gain` (1.079), applied by
  `kinematics.yaw_feedforward` inside `velocity_to_wheel_pwm`. Re-measured on the
  car with it active: **106 / 102 / 100 / 111 %** (was 38 / 67 / 75 / 92).
- The feedforward is capped so the inner wheel never reverses. At low v the extra
  yaw would flip it backwards, and a differential-drive car that pivots one wheel
  is not what the policy planned; at those speeds the car genuinely cannot deliver
  the full yaw and the cap makes that honest instead of commanding the impossible.
- On the car, variant 1 reached B again (4.29 s, 1.09 m for a 1.2 m goal, yaw
  -10 deg). Cumulative measured/commanded improved from 0.810 -> **0.961** on
  translation and 0.451 -> **0.608** on yaw.

**The remaining yaw gap is a different effect and is still open.** The steady-state
sweep now tracks ~100 %, but a real run only reaches 0.608 because the command
reverses sign tick to tick (+0.68 / +0.50 / -0.40 / -0.53) faster than the chassis
can follow. That is rotational lag, not deadband -- the next thing to fix.

**Offline, 20 seeds x 6 scenarios, at the shipped 10.5 V constants:** v2 0.833 /
0.000, v1 0.583 / 0.408 on the tight built-in suite. v2 was 1.000 at the previous
(9.8 V) constants: the offset moving 17 -> 14 makes a PWM-40 hop 15 % longer, and
the tight suite is sensitive to that. The plant genuinely is voltage-dependent and
one constant set cannot be exact across the pack's range.

## 0.9.1 - 2026-07-25 - calibration reproduced; variant 1 no longer spirals

Re-ran the calibration at a different battery voltage and finished the on-car
tests that were still open.

**Reproducibility (PWM 30/40/60 per axis, fit residuals +-0.003 m/s):**

    9.8 V : pwm_per_mps 73.4, pwm_offset 17.0, arm 0.194
    10.5 V: pwm_per_mps 72.1, pwm_offset 14.0, arm 0.198   <- now shipped

The slope moved 1.8 % and the arm 2 %; the friction threshold moved most
(17 -> 14 PWM, since more torque breaks stiction at a lower PWM). **My earlier
warning that pwm_per_mps would be "WRONG for a charged pack" was overstated** --
it is barely voltage-sensitive. The arm is a ratio of the two axes measured at the
same voltage, which is why both runs agree on it; that is the number to trust.

**Variant 1 reaches the goal on the car.** Before calibration the same policy
spiralled: yaw -145 deg, 1.40 m travelled for a 0.6 m goal, timeout, and w pinned
at -1.200 for 12 consecutive ticks. After: reached in 3.96 s, 1.16 m travelled for
a 1.2 m goal, yaw -9 deg, w never above 0.68, and the tick log shows
`flags REACHED=1` -- no WPIN, no AWAY, no ZERO -- with gd falling monotonically
1.200 -> 0.098.

**Two unmodelled effects the new instrumentation measured, both still open:**

- **v-w coupling.** The `arc` test (end-to-end, deadzone included) at v=0.361:
  straight tracks 99 %, but at w=0.6 forward speed is 90 % and at w=1.2 only
  **53 %**. Skid-steering scrubs forward speed; `rollout_unicycle` assumes v is
  independent of w. A first cut is v_eff = v*(1 - k*w^2) with k ~ 0.30 from these
  three points.
- **Yaw lag.** Cumulative measured/commanded over the successful run was 0.810 on
  translation (consistent with the coupling above plus the standing start) and
  **0.451 on yaw**. The command oscillated +0.68 / +0.50 / -0.40 / -0.53 tick to
  tick and the car cannot follow that; this is the previously qualitative "omega
  command jitter" now quantified. Per-tick ratios are noisy (0.33-1.20); the
  cumulative figure is the signal.

Also fixed a bug in `smoke/calib_model.py`: the `arc` test computed its test speed
as `mag / pwm_per_mps`, the old proportional form, so it drove at 0.555 m/s
instead of the intended v_max 0.361. It now uses the affine inverse.

## 0.9.0 - 2026-07-25 - plant model CALIBRATED; tick log completed

### The plant is affine, not proportional

`smoke/calib_model.py` (new) measures commanded -> actual on the car. Three PWM
levels per axis, /odom as reference (encoder xy, gyro yaw):

    LINEAR  v = (PWM - 17.0) / 73.4        residuals +-0.003 m/s over PWM 30/40/60
    YAW     offset from the yaw axis independently = 18.4 PWM  (linear axis: 17.0)

The motors do not turn until PWM clears a friction threshold. With the offset
accounted for the yaw arm becomes a constant **0.194 m** (0.199 / 0.195 / 0.186);
under the old proportional model it "drifted" 0.168 -> 0.125 -> 0.095 with PWM.
**A friction offset was masquerading as wrong geometry.** If your measured arm
changes with PWM, you are missing an offset, not a length.

Model error at the magnitude the car is driven (PWM 40): speed **1.55x** too slow
in the model, yaw **0.80x** too fast -> real turn radius ~1.9x the planned one.
That is what made variant 1 command max yaw tick after tick and spiral.

- `RobotConfig`: `pwm_per_mps` 200 -> **73.4**, new **`pwm_offset` 17.0**,
  `wz_arm` 0.5 -> **0.194**. `Variant1Config.steer_arm` 0.10 -> **0.194**.
  `v_max` is now the affine inverse of the magnitude, not `magnitude/pwm_per_mps`.
- **`deadzone_pwm` is gone.** It clamped each wheel UP to a fixed value
  independently, which flattened the left/right difference -- and that difference
  IS the steering signal. Replaced by the affine inverse
  `PWM = v*k + offset*sign(v)`, one constant added to every wheel, so the
  differences survive. Commanded -> realised yaw went from 0%/0%/13%/17% at
  magnitude 20 and 100%/100%/87%/78% at 40, to **100% at both**.
- Offline over 20 seeds x 6 scenarios: variant 2 **0.833 -> 1.000** (its hop
  distance had been over-estimated ~55%). Variant 1 0.750 -> 0.592 on the built-in
  TIGHT suite -- not a regression: restoring the fictitious 0.10 arm reproduces the
  old 0.750, i.e. the sim had been granting turns the car cannot make. In the
  regime the car is actually driven (B=3 m, obstacles 1.2-1.5 m) the calibrated
  model is **1.000 success / 0 collisions**, identical to before.
- CAVEAT: measured at **9.8 V** (healthy is 11-13 V). `pwm_per_mps` moves with
  voltage -- re-run `--test linear` on a charged pack. `pwm_offset` (a threshold)
  and `wz_arm` (a ratio) are far less sensitive.
- Measurement trap found the hard way: **yaw must be integrated during the run**,
  not differenced end-to-end. /odom yaw wraps to (-pi, pi], so the first PWM-60 run
  turned a full revolution and read -3.4 deg, putting the arm at -7.165 m.
- Procedure documented in `mpc/README.md`.

### Tick log completed

A design pass over the log (four debugging personas) found the biggest hole:
**`tlog.tick()` was only reached at the end of the loop**, so every `continue` /
`return` / `break` path -- no frame, stale data, link loss, collision, abort,
timeout, goal reached -- produced NO line. Each failure tick was a gap, and a
reader seeing 0.67 s of silence would conclude the loop hung when it had actually
run, stopped the car and cleared its buffer. Now **every tick emits exactly one
line**, tagged `NOFRAME` / `STALE` / `COLLIDE` / `LINKLOST` / `ABORT` / `TIMEOUT` /
`REACHED`, so a gap in the numbering can only mean the loop stopped.

Also added, all from the same pass:
- **pose in the RUN-START body frame**, not raw odom. A run starting at odom
  (1.96, -0.94) used to print as if it began 2 m off course.
- **`obs/mem` shows two clearances**: what the collision guard sees (this frame's
  circles only) and what the policy plans against (the odom rolling memory).
  `n--` with a small `m` means the guard cannot fire on an obstacle the policy can
  see -- it separates "perception lost it" from "the guard had nothing to act on".
- **`real/cmd`**: measured / commanded motion per tick, the model-vs-plant
  divergence live. Suppressed under `pose_source=dead_reckon`, where the pose IS
  the integrated command and the ratio would be a tautology.
- `src_fid`: which observation frame each dispatched command was planned from, so
  the one-tick buffering is verifiable rather than assumed.

## 0.8.0 — 2026-07-25 — dedicated per-tick debug log

One file per experiment, named by the experiment start time, so someone who did NOT
write the policy can open it and see what happened on every tick without re-running.

- **`mpc_baseline/ticklog.py`**: `output/<start>/tick_<start>.log` (fixed-column text,
  for reading) plus `tick_<start>.jsonl` (same records, for plotting/grep). The GUI
  passes its existing per-Execute directory so the tick log lands beside
  `observation.mp4`/`run.json`; the CLIs get an equivalent directory automatically, so
  all three entry points behave the same.
- **Header** records what the run WAS: policy, action space, tick rate and period,
  action duration, goal B, magnitude, pose source, safety-guard state, robot
  constants, the command box (`v` range and `|w| <= w_max`, so a pinned command is
  recognisable), and the model-vs-plant caveat. **Footer** gives the outcome plus tick
  count, wall time and a count of every flag raised.
- **One line per tick** shows the tick's structure directly: `DISPATCH` (what was SENT
  this tick, decided one tick earlier) next to `PLAN->next` (what this observation
  produced, to be sent next tick) — the one-tick command buffering is visible rather
  than inferred. Plus frame_id, pose, distance to B, obstacle count and nearest edge,
  and a timing breakdown `wA pB WC` (ms waiting for the frame / planning / total work).
- **A `FLAGS` column names anomalies** so a 500-line file is skimmed, not read: `SKIP`
  (frames missed), `OVERRUN` (work exceeded the tick), `HOLD` (stale data, car
  stopped), `NODISP`, `NOPTS` (points did not pair), `ZERO` (commanded a full stop —
  the "car froze" failure), `WPIN` (yaw pinned at the cap — cannot turn enough),
  `AWAY` (distance to B grew), `NEAR`, `SAFETY`. Healthy ticks show `.`.
  Validated by replaying the real spiral run recorded earlier: healthy ticks read `.`
  and the failure onset is unmistakable from tick 9 — `WPIN AWAY` repeated to the end.
  A `v`-at-`v_max` flag was tried and dropped: a goal-seeking policy sits there nearly
  every tick, which buried the flags that mean something.
- The runner is instrumented for this (`self._t`): per-tick index, wait/plan/work
  times, actual period, frames skipped, and whether the tick re-planned or replayed a
  cached step. Each line is flushed immediately, so an E-STOP, Ctrl-C or crash still
  leaves a complete file — those are the runs worth reading. Logging failures are
  swallowed: a log must never take the car down.

Verified live: two runs on the car, 6 ticks, `flags (none)`, dt 0.332–0.340.

## 0.7.0 — 2026-07-25 — one global tick; command buffering; dt-invariant cost

The stack had three unsynchronised clocks: perception 3 Hz, the variant-1 loop
4 Hz (free-running, never waiting for a new frame), and the variant-2 loop ~1.61 Hz
(paced by `time.sleep(step_duration + SETTLE_S)` inside the actuator). Consequences,
by arithmetic: variant 1 re-planned on an observation frame it had already seen on
about 1 cycle in 4; variant 2 never looked at about half the frames at all.

Now everything runs on ONE tick, game-engine style. `TickConfig.rate_hz` (3.0 = the
car's perception rate) is the single number; `LiveConfig.plan_rate` is gone.

- **A tick is one observation frame.** New `carclient.wait_frame(after, timeout)`
  blocks on a condition variable signalled by `_on_obs`, so a tick fires exactly
  once per new `frame_id` — no repeats, no skips (verified: 9 frames -> 9 ticks,
  0 repeats, 0 skips). It matches on "different id", not "greater id", so the
  car-ros restart that resets `frame_counter_` to 1 does not hang it.
- **Commands are buffered one tick.** Inside a tick: wait for the frame, run the
  safety checks on that FRESH frame, DISPATCH what the previous tick decided
  (superseding whatever the car is still running), then plan this frame into the
  buffer for the next tick. Emit nothing and the car keeps running its current
  command; let it expire and the car holds (the car-side node brakes at
  `end_time`). Deterministic one-tick latency instead of jitter.
- **`DriveActionActuator.step()` no longer blocks.** It slept for the whole hop,
  which is what paced variant 2 off the hop instead of off perception. The car-side
  handler already ends a running move as "superseded" and starts the new one
  immediately, so back-to-back hops stay continuous. The runner now rejects
  `step_duration < tick period` (the hop would expire before the next tick replaced
  it and the car would brake in the gap).
- **`_hold()` now actually stops a discrete policy.** It was a no-op on the premise
  that discrete policies are "between hops" — true only while the runner slept
  through each hop. With non-blocking dispatch a hop can be running when a tick
  decides to hold.
- **The GUI runs on the same tick**: it polls at 4x and edge-detects `frame_id`, so
  observation, render and recording happen exactly once per tick. "refresh Hz" is
  now "tick Hz (global)" and is passed into the runner.

**Model timing aligned, and the cost made dt-invariant.** `MPPIConfig.dt` is now the
tick (1/3 s, was 0.6 s against a 0.25 s execution period), with `horizon` 16 -> 29 to
keep the same 9.7 s lookahead; solve time 6.9 ms mean / 9.2 ms max against a 333 ms
budget. That retune initially turned 0 collisions into 2, which exposed a real latent
bug: **every running cost term was a raw sum over H**, so changing the horizon
silently re-weighted running-vs-terminal and obstacle-vs-goal. Running terms are now
integrals (`x dt`) and smoothness is a rate (`du^2/dt`); weights are the
car-validated values divided by the old dt=0.6 (`w_smooth` x0.6), so the old timing
reproduces its old numbers exactly. `noise_beta` likewise became `noise_tau`, an
AR(1) time constant in seconds, so a sampled manoeuvre stays the same manoeuvre when
the tick changes.

**Honest result:** measured over 20 seeds x 6 scenarios (n=120), not the single seed
the headline suite uses — old timing 0.742 success / 0.242 collision / 0.017 frozen,
new tick timing 0.750 / 0.250 / 0.000. Statistically indistinguishable, which is the
correct outcome for a timing change. Note the ~24 % collision rate is PRE-EXISTING on
the old timing too; the 1-seed suite reports 0.833/0.000 and simply cannot see it.
Variant 2 is unchanged at 0.833 / 0.000 / 0.000.

**Not yet verified on the car** — it was powered down and charging when this landed.

## 0.6.4 — 2026-07-25 — adversarial review of 0.6.2/aa04122: one real bug, four false doc claims

A 14-agent adversarial review of the preceding commits. Everything below reproduced.

- **BUG `eval.py`: `run_variant` lower-cased the key BEFORE testing the registry**,
  while `register()`/`build_policy()` treat keys verbatim. So a registered key that
  collided with a built-in alias — `grid`, `vw`, `v1`, `v2`, `1`, `2`, any casing —
  **ran the built-in instead, labelled with the caller's key**, and any mixed-case
  key was unreachable by every spelling (the error even printed `MyModel` inside the
  list of keys it claimed were valid). That is precisely the silent wrong-policy
  failure `aa04122` was written to eliminate, reintroduced one line above the fix.
  Reproduced: a Spy policy registered as `grid` had `plan()` called 0 times and
  `run_variant('grid')` was byte-identical to `run_variant(2)`. Registry keys are
  now matched first and verbatim (Spy now gets 108 `plan()` calls under both `grid`
  and `MyModel`).
- **`config.build_live_cfg` still had the same fall-through**: `if v1: ... else: v2`,
  so `"V1"`, `"velocity"` or any typo silently returned a discrete `Variant2Config`
  with no `.v_max`/`.mppi` — and `mpc/README.md` tells model authors to call this
  function by hand. `build_policy`'s action_space check cannot catch it (it compares
  the registry entry against the POLICY, never against the cfg). Now explicit on both
  sides with a raise, matching `policies.make_policy`.
- **`--plan-dt` is a no-op for discrete policies** (`sim.py` uses `act.duration`), but
  the CLI help said "every policy" and `mpc/README.md` offered it to custom-model
  authors as the way to compare "按实车节奏". Verified: `run_variant(2, plan_dt=0.01)`
  and `plan_dt=5.0` are identical; velocity does change. Help text corrected.
- **"compare like with like" was false.** `--variant --live-profile` takes magnitude
  from `LiveConfig.magnitude` (20 → v_max 0.10) while `--policy` builds through the
  registry at `--magnitude` (default 40 → v_max 0.20) — exactly 2.00x. Documented,
  with the flags that actually match them.
- **"the EXACT points the circles were clustered from" overstated it** in 5 places.
  `publishPoints(pts)` publishes the full range-filtered scan, but circles come from
  `cluster_pts` (DBSCAN label >= 0) and are then EMA-smoothed by `temporalFilter`, so
  a few percent of published points are noise lying inside no circle. Publishing
  everything is right for a viewer; the wording was wrong, and it contradicted the
  0.6.2 claim that "every point cluster sits inside its own circle" (corrected in
  place). It is the same SAMPLE, not a per-point correspondence.

## 0.6.3 — 2026-07-25 — review of 0.6.2: frame-id reset bug, dead /scan subscription

Self-review of the 0.6.2 change set found one real bug and one real redundancy.

- **BUG: the point cache died permanently after a car-ros restart.** `_opts` was
  pruned by keeping the numerically largest frame_ids
  (`sorted(self._opts)[:-POINT_FRAMES]`). The car's `frame_counter_` restarts at 1
  whenever the `obstacle_circles` node does, so every post-restart frame (1, 2, 3 …)
  sorted below the retained pre-restart ids (101–104) and was deleted the instant
  it arrived. `observation().points` then returned None **forever**, silently, until
  the client process was restarted. Reproduced against the real callbacks: 7/7
  post-restart frames unpaired, cache frozen at `[101,102,103,104]`. Fixed by
  evicting in ARRIVAL order (`OrderedDict` + `move_to_end` / `popitem(last=False)`);
  `obstacle_points(None)` likewise returns the newest *arrived* frame rather than
  `max(id)`. After the fix all 7 post-restart frames pair and the stale ids flush
  within POINT_FRAMES frames.
  **Reachability, corrected 0.6.5 after live testing:** a full `systemctl restart
  car-ros` restarts roscore, which drops the client's subscriber registrations
  entirely (verified live: the pre-existing client timed out, a fresh one paired
  10/10) — so that path masks the bug behind a reconnect the GUI already handles by
  self-relaunching. The bug is reachable when the perception node alone restarts
  while the client keeps running. The fix is right either way; the original claim
  that the GUI's "Restart car-ros" button triggers it was wrong.
- **REDUNDANCY: carclient still subscribed to `/scan` for nobody.** Moving the GUI
  to `/obstacle_points` left `scan_points()` with zero callers in the repo, but the
  subscription stayed unconditional in every CarClient — including inside the 4 Hz
  MPC loop. Measured on the car: **43.3 KB/s** (7.6 Hz × 720 beams), ~3× what
  `/obstacle_points` costs (15.2 KB/s) and ~17× `/obstacles` (2.6 KB/s). Link
  latency on this car has already been a root cause of closed-loop failure, so the
  subscription is now opt-in: `CarClient(subscribe_scan=True)`. `scan_points()`
  raises a message pointing at `observation()` when it is off. `LidarOdometry` has
  its own `/scan` subscriber, so `pose_source="lidar"` is unaffected.
  (The CPU cost of the dead handler turned out to be negligible — 0.10 ms/scan =
  0.08 % of a core — so bandwidth, not CPU, was the reason to change it.)
- **GUI double-gating removed**: `_poll_tick` gated the points on the checkbox AND
  `paintEvent` gated on `show_points`. The data gate also blanked the cloud for one
  poll period each time the box was ticked. Points are now always handed to the
  view and `show_points` alone decides whether to draw.

## 0.6.2 — 2026-07-25 — point cloud is now frame-synced with the obstacle circles

The GUI drew the point cloud from `/scan` and the circles from `/obstacles`, two
independently cached topics with nothing pairing them. They were never the same
sample, on three counts:

1. `obstacle_circles_node` runs on its own `rate_hz` timer off the *latest* scan,
   so at 3 Hz against the lidar's ~7.7 Hz roughly 60 % of scans never produce
   circles — the client's newest `/scan` is usually not the one behind the
   circles it holds.
2. Each cache ages independently: measured skew between the circles' own source
   points and the client's freshest `/scan` was −0.041 s to +0.221 s over four
   consecutive samples. At 0.2 m/s and ~0.7 rad/s that is centimetres of shift
   and ~10° of rotation of the whole cloud against the circles.
3. The circles are not even a single scan: `temporalFilter` EMAs each circle
   across frames (`filter_alpha=0.5`) with ego-motion compensation.

Fix — publish the points the circles actually came from, tagged with the same id:

- **New topic `/obstacle_points`** (`std_msgs/Float32MultiArray`,
  `[frame_id, x,y, x,y, ...]`, base frame): the exact post-range-filter point set
  `process()` clustered this cycle, published in the same call with the same
  `frame_counter_` as `/obstacles`. Subscriber-gated like `/obstacles_viz`, so it
  costs nothing when unused. Params `publish_points` (true), `points_stride` (1).
  Measured on the car: 646 pts/frame = 5.0 KB/frame = 15.1 KB/s at 3 Hz.
- **`carclient.observation()` → `Frame(frame_id, circles, points, age)`**: circles
  and points guaranteed to share a frame_id. `points` is None when that frame has
  no points yet — it is never back-filled with another frame's points, which is
  the whole point. Points are kept in a small frame_id-keyed cache (POINT_FRAMES=4)
  because ROS gives no cross-topic ordering guarantee. `scan_points()` (raw
  `/scan`, unsynchronised) stays for callers that want raw lidar.
- **GUI** draws the pair atomically via `render_frame(..., points=...)` and shows
  `pts N @frame N` in the overlay, or `-- (no /obstacle_points)` — so a missing
  pair is visible instead of silently drawing a stale cloud. Previously `sp.age`
  was never checked at all, so a wedged lidar would have left the last cloud on
  screen indefinitely.

Verified on the car after `deploy_to_car.sh`: 12/12 consecutive `observation()`
samples paired (frames 118–129, ~73 circles / ~645 points each), and an offscreen
render shows the clusters sitting inside their circles. (Correction, 0.6.4: the
published set is the INPUT to clustering, so a few percent are DBSCAN noise points
that lie inside no circle — same sample, not a per-point correspondence.)

## 0.6.1 — 2026-07-25 — docs: install command was broken; stale/wrong doc claims fixed

Documentation-only pass (no behaviour change). Every item below was reproduced.

- **The install command in README never worked**: `pip install -e carclient carpolicy mpc`.
  pip's `-e` takes ONE path — the other two were treated as PyPI package names, so the
  command installed only `carclient` and then died with
  `ERROR: Could not find a version that satisfies the requirement carpolicy`.
  Consequence: **`mpc-baseline` was never installed** in the `ros1` env (`carclient`
  and `carpolicy` had been installed separately at some point). Nothing broke in
  practice only because every entry point does its own `sys.path.insert(...)`
  (`gui/car_console.py:29`, `mpc/scripts/*.py`, `smoke/policy_run.py`). Fixed to
  `pip install -e carclient -e carpolicy -e mpc`.
- **Repo-root package shadowing documented properly.** The root dirs `carclient/` and
  `carpolicy/` shadow the installed packages as empty namespace packages whenever cwd
  is on `sys.path` (`python -c`, `python -m`, REPL) → `carpolicy.__file__ is None` and
  `ImportError: cannot import name 'Policy' from 'carpolicy' (unknown location)`.
  Running a script as a file is unaffected (repo root is not on `sys.path` then). The
  old troubleshooting row mentioned only `carclient` and described it imprecisely.
- **`reset()` is documented as "called before each run" but has ZERO callers** —
  `PolicyRunner.run()` does not call it, nor does the GUI or any CLI. It works today
  only because a fresh policy is built per run via `registry.build()`. Docstrings in
  `carpolicy/__init__.py` and the "add your own model" guide in `mpc/README.md` now say
  what is actually true, and flag it as a pending code fix.
- **magnitude 20 cannot steer** — added to `mpc/README.md`. `velocity_to_wheel_pwm`
  bumps each wheel INDEPENDENTLY up to `deadzone_pwm=30`, which flattens the steering
  differential. Measured commanded-vs-realised yaw rate at `v_max`: mag 20 gives 0% of
  commanded w for |w| <= 0.45 (all four wheels pinned at 30 PWM → pure straight line,
  and vx inflates 0.10 → 0.15 m/s) and 17% at w=0.90; mag 40 gives 78–100%. The planner
  models none of this. This is why the one validated real-car go-around was at mag 40.
- **Benchmark numbers were absent from `mpc/README.md` and stale in this file.** Current
  measured baseline is **5/6 for BOTH variants** (v1 fails `wall_inline`, v2 fails
  `slalom`), not the "100 % reached, 0 collisions for both variants" recorded under
  0.3.0 (left as-is there — it is a dated historical entry). Also documented the two
  ways the offline suite differs from the car: `eval.py` closes the loop at
  `plan_dt = MPPIConfig.dt = 0.6 s` while the live runner uses `1/plan_rate = 0.25 s`
  (re-running at the live cadence with the live config drops v1 to 4/6), and the sim's
  "ground truth" is the planner's own integrator with `noise_xy`/`dropout` never set.
- Lookahead/turn-radius rows now state their magnitude dependence: lookahead
  `H*dt*v` is ≈1.9 m at mag 40 but ≈0.96 m at mag 20; min turn radius `v/w` is 0.17 m
  at mag 40 and 0.11 m at mag 20 (where the differential constraint caps w at 0.9 first).
- `README.md` file table: dropped `car_ros/rplidar_watchdog.py` (that file is **not** in
  this repo — it lives only on the car) and added the missing `carpolicy/`, `mpc/`,
  `car_ros/viz.launch`, `output/` rows.

## 0.6.0 — 2026-07-24 — variant 1 → sampling MPC; gyro odom; WiFi latency; GUI recording

- **Variant 1 refactored from CasADi NLP → continuous (v,ω) SAMPLING MPC (DWA)**
  (user: "经典简单问题不该越设计越冗长"). Same core as variant 2, continuous action
  space: sample K (v,ω) sequences (AR(1)-smoothed noise) → rollout → score vs goal +
  ALL obstacle circles + the A→B line → argmin → warm-start. `nlp_mpc.py` and the
  casadi dependency DELETED; multi-start seeds / via-points / hysteresis gone. ~90
  lines, ~6–20 ms/solve (was 80–160 ms). Both variants now one sampling kernel.
- **odom yaw now from the IMU GYRO, not the wheel encoders** (`car_base_node.py`,
  `yaw_source=gyro`, `gyro_sign=-1`, auto bias). The encoder differential badly
  UNDER-reports yaw in a forward-arc turn (car turned 45° while odom read 7.6° → the
  closed loop thought it was straight and spiralled off). lidar ICP is the WORST yaw
  source (loses yaw on rotation) — never use `pose_source=lidar`.
- **WiFi 3 s latency root-caused + fixed as a car system default**: NetworkManager
  power-save (`wifi.powersave=2`); power-save made the radio sleep → 3+ s RTT →
  starved the 4 Hz closed loop of fresh /odom. This (not the planner) caused the
  early "spiralling / drove into obstacles". `carclient` `_dump_fh` init-order race
  also fixed (was a startup callback error).
- **Config** (variant 1): `MPPIConfig.horizon=16, dt=0.6` (~1.9–2.1 m lookahead;
  can't be as short as variant 2's 4 — differential steering needs to see wide/near
  obstacles earlier), `w_max=1.2` (turn radius ~0.17 m; user relaxed 45°→60°),
  `CostConfig.w_track=2.5` (was 6.0 — too strong, fought the wide detour around a
  wall/2nd obstacle ON the line; 2.5 fixed sequential-obstacle cases & lifted the
  eval 0.67→0.83), `extra_margin=0.10`.
- **GUI**: default policy = velocity (variant 1), B=3 m, collision guard OFF;
  **each Execute records `observation.mp4` (3 Hz) + `trajectory.png` + `run.json`**
  to `output/<ts>/`. Plus bug fixes: manual-pad lockout during an MPC run,
  start-failure UI recovery, Restart-car-ros aborts the running policy first.
- **Docs rewritten** (`mpc/README.md`, top-level `README.md`) incl. an "add your own
  model" guide (implement `carpolicy.Policy` → register in `registry.py` → appears
  in the GUI). Multi-agent audit: 37 findings fixed (mostly stale NLP/CasADi refs +
  dead config `lam`/`keepalive_rate` + doc value errors + 3 small bugs).

## 0.5.0 — 2026-07-24 — live bring-up: rplidar fix, differential variant 1, velocity executor

- **rplidar "restart twice" bug fixed** (see 0.3.0 section) -- STOP command before
  launch; /scan now up on a single `car-ros` restart.
- **Variant 1 is now DIFFERENTIAL DRIVE** (user spec): all wheels forward, steer by
  the L/R speed difference, bounded turn. NLP inequality `v - |w|*steer_arm >= 0`
  (inner wheel >= 0, ~45 deg cap); `steer_arm` (0.15) is synced to `robot.wz_arm`
  (the actuation mix arm) in the variant-1 config factories so the real inner wheel
  never reverses. Smoother, car-like. Verified reaching B on the real car.
- **Variant-1 actuation = car-side velocity executor** (replaces the workstation
  /wheel_cmd stream, which arrived bursty over lossy WiFi and wedged the MCU
  serial). New `/drive_wheels [FL,RL,FR,RR,duration]` topic handled by
  drive_action_node's `on_wheels` (local 20 Hz keep-alive + sustained brake);
  workstation sends ONE pulse per plan cycle via `carclient.drive_wheels()` /
  `VelocityPulseActuator`. Verified: 4 Hz sustained pulses drive continuously with
  no serial wedge.
- **GUI**: B range ±10 m; collision guard fires on contact only (`collision_margin=0`,
  the perception circles already carry margin); +x/+y axis hints in the top-down view.
- **Variant-1 smoothing** (user: turns were abrupt / a 90-deg reactive turn, inner
  wheel stopped): `w_smooth=0.6` (gradual steering), `w_max=0.5`, `min_inner_frac=0.1`
  + live `deadzone_pwm=30` (inner wheel keeps spinning through a turn), `dt=0.4`
  horizon 14 (~1.1 m lookahead so it plans the whole arc, not a reactive turn). Use
  pose_source `odom` (motor, accurate yaw) for variant 1; lidar ICP drifts over long
  runs. Live results: reaches B=(3,0), clean go-around, smooth.
- **Cross-track cost (variant 1)**: `CostConfig.w_track=6` penalises deviation from the
  straight A->B line (the policy remembers A = the run's first pose), so the car stays on
  the direct path, deviates only the minimum to clear an obstacle, and returns to the line
  ASAP -- no more going far out before turning back. Verified live: tight go-around, dips
  ~0.75 m to clear then rejoins the line, path 3.6 m (was 5.4 m). Horizon back to H=14
  dt=0.35 (the cross-track cost, not a long horizon, keeps it tight; ~37 ms/solve).
- **`smoke/policy_run.py`** -- the live experiment/visualization tool: drives a policy
  to B, records the trajectory, and writes a start-frame plot (near obstacles only) +
  JSON to `smoke/results/` (gitignored). `python smoke/policy_run.py --variant 1 --bx 3 --by 0 --pose odom`.
- **GUI "Restart car-ros" button** (`carclient.restart_ros()`, SSH `systemctl restart
  car-ros`): recover the stack after an E-STOP without leaving the GUI (single restart now).
- Root causes documented (not all fixed): control-not-executing = Rosmaster lib
  swallows serial write errors + MCU serial on a flaky VIA USB hub; "goal drift" =
  odom forward over-read diverging from the lidar obstacles (use the lidar pose
  source to align them), not a goal-code bug.

## 0.4.0 — 2026-07-24 — Policy abstraction (`carpolicy`) + GUI policy panel

- **`carpolicy/`** (new pip package, sibling of `carclient`): the GENERAL policy
  interface, deliberately NOT under `mpc_baseline`. `Policy` (ABC) with
  `plan(Observation) -> Action`; `Observation(pose, goal, circles, field)`;
  `Action.velocity(v,w)` / `Action.discrete(id,mag,dur)` (+ optional `traj`).
  MPC is one implementation; learning-based policies inherit the same base. I/O is
  documented at the top of the module. `pip install -e carpolicy`.
- **MPC variants now `class …Policy(carpolicy.Policy)`** overriding `plan`; each
  declares `action_space` ("discrete"/"velocity").
- **`mpc_baseline.registry.POLICY_REGISTRY`**: name → {label, action_space, build}.
  Add a backend with one entry; the GUI/CLI pick it up automatically.
- **`MPCRunner` → `PolicyRunner`** (alias kept): drives ANY Policy, dispatching on
  `action_space`, not a hardcoded variant. First arg is a ready Policy or a
  variant key.
- **GUI policy panel** (`gui/car_console.py`): pick a policy backend (registry-
  driven dropdown), set **B as coordinates** — forward x, left y (m) relative to
  the start pose — pick the **pose source** (motor odom [default] / lidar / dead-
  reckon), toggle the collision guard, **Execute / Stop**. Runs the MPC in a
  background thread; draws goal B (green) and the **predicted path (yellow)** in
  the top-down view. Manual steering + Execute gate on MCU-link health.
- Goal B generalised to `(goal_dist forward, goal_y left)` in the start body frame.

## 0.3.0 — 2026-07-24 — /odom fixed (encoder-based) + lidar odometry

Live bring-up found **/odom was broken on the forward/back axis**: the firmware's
`get_motion_data()` reported ~10% of true forward motion (a 0.28 m drive read as
0.025 m), while strafe and yaw came back ~90% correct. Root cause: the firmware
computes odometry from a mis-mapped wheel frame (same wheel/port swaps as the
`_set_wheels` command path), so the forward component cancels. Motors are fine —
all 8 directions physically move ~20 cm (verified with lidar).

### Fixed (`car_ros/car_base_node.py`, deployed to the car)
- **/odom now rebuilt from RAW per-motor encoders** (`get_motor_encoder`) with the
  correct wheel mapping (M1=FL, M2=−RR, M3=FR, M4=−RL, same as `_set_wheels`) and
  scales calibrated against lidar: `k_lin_x=0.000164`, `k_lin_y=0.000176` m/count,
  `k_ang=0.001056` rad/count (all ROS params; `~odom_source=firmware` reverts).
  Verified: forward recovered, strafe ratio 0.99, yaw ratio 0.99. Forward still
  over-reads ~7% straight / more when it veers (wheel slip + motor imbalance) — a
  physical issue the MPC's per-cycle re-aim absorbs.
- New topic **`/wheel_encoders`** (`std_msgs/Float32MultiArray [m1,m2,m3,m4]`).
- **Lidar ICP odometry** (`mpc_baseline/lidar_odom.py`): accumulates ego-motion by
  scan-matching /scan; most accurate on the forward axis. Used to calibrate the
  encoder odom; also selectable as the MPC pose source (`pose_source="lidar"`).
- Runner tracks pose via `/odom` by default, with `lidar` and `dead_reckon` options.

### rplidar "restart twice" bug — ROOT-CAUSED & FIXED
- Root cause (from `~/.ros/log/<session>/rosout.log`): a rplidarNode killed
  mid-scan leaves the lidar STREAMING scan packets (motor keeps spinning). The
  next node sends GET_DEVICE_INFO but reads the leftover stream instead of the
  reply, so it STALLS right after "RPLIDAR running" (never reaches "Firmware Ver"
  / "current scan mode") -> no /scan until a second restart stops the stream.
- **Fix** (`car_ros/ros_stack_start.sh`, deployed): send the RPLIDAR STOP command
  (`0xA5 0x25`) to /dev/rplidar before `roslaunch`, so the node always opens a
  quiet device. Verified: /scan now comes up on a SINGLE `sudo systemctl restart
  car-ros` (twice, deterministic). No more double restarts. Backup
  `~/ros_stack_start.sh.prempc.bak` on the car.

## 0.2.0 — 2026-07-24 — MPC baseline policy (`mpc/`)

Workstation-side **MPC baseline** to take the car A→B (B = the pose ~5 s of
forward driving reaches) **around** lidar obstacle circles. Two control variants,
both **continuous / never stop before B**:

### Built (`mpc/`, package `mpc_baseline`)
- **Variant 2 (default baseline, discrete grid-hop)** — sampling/enumeration MPC
  over the 8 mecanum translation actions (STOP excluded so it never stops early),
  picks the lowest-cost sequence's first action, actuates via `/drive_action`.
- **Variant 1 (continuous v,ω)** — CasADi/IPOPT nonlinear MPC (installed casadi);
  parametric NLP built once, warm-started, **multi-start** with pursuit seeds to
  break the "stop dead in front of a symmetric obstacle" local minimum. Actuates
  via `/wheel_cmd`. No fallback planner.
- Pure-numpy brain (kinematics/obstacles/cost/sampling) + **offline simulator +
  eval/benchmark**: the exact planning code validated with no car — 6 scenarios,
  100 % reached, 0 collisions for both variants.
- **carclient** extended: `.pose()` (/odom), `.wheels()` (/wheel_cmd).
- Live safety: streamed-wheel keep-alive; refuses to drive unless the MCU link is
  healthy (so the link-loss estop is armed); imminent-collision estop fires before
  the footprint touches; stale odom/obstacles hold; any abnormal exit hard-estops.
- 8-finding adversarial multi-agent review; all fixed. Kinematics + NLP packing
  reviewed clean.

### Findings
- **/cmd_vel rotation is (by analysis) broken on the car**: `car_base_node._set_motors`
  maps the rear wheels as `set_motor(FL,-RL,FR,-RR)` but the correct port order
  (per `_set_wheels`) is `set_motor(FL,-RR,FR,-RL)` — rear-left/right swapped. For a
  `wz` command the front and rear axles then yaw opposite ways and cancel (matches
  the old "wz is banned at the bridge" note). NOT empirically re-tested (car was
  offline). The MPC sidesteps it by driving `/wheel_cmd` (correct `_set_wheels`
  path); only fix `_set_motors` if you need `/cmd_vel`.
- Goal B near an obstacle: keep `obs_buffer`/`extra_margin` small enough that B
  isn't swallowed by the inflation, or the soft wall repels the car short of B.
- Real-car runs start at **magnitude 20** (near the ~30 PWM motor deadzone; if it
  doesn't move, raise `--magnitude`, or variant 1 `--deadzone-pwm 30`).

## 0.1.0 — 2026-07-23 — initial bring-up

Brought the Jetson Nano mecanum car under **pure-ROS** control from the
workstation, with a PySide6 console and a `carclient` Python API.

### Built
- **Network**: workstation WiFi hotspot `coord_nav` (TP-Link RTL8811AU, DKMS
  `8821au`); car fixed at `10.42.0.187`; `roscar` env; `smoke/` self-tests.
- **Perception** (`obstacle_perception/`, C++): `/scan` → DBSCAN → enclosing
  circles → redundancy drop → `/obstacles` (`[frame_id, x,y,r, ...]`, metres,
  base frame). ~3.4 ms/frame (≈20× the old Python `perception_server.py`), 3 Hz.
- **Control** (`car_ros/drive_action_node.py`): discrete mecanum actions
  `/drive_action [id, magnitude, duration_s]` → timed `/wheel_cmd` pulses →
  `/drive_result`. Per-wheel `/wheel_cmd` retained for bring-up.
- **carclient/** (pip package): `CarClient` — obstacles / drive / result / estop
  / MCU-link health; bounded 100-frame in-memory history + on-disk dump.
- **gui/** (PySide6): obstacle top-down view, steering wheel, colour log.
- Car runs **headless** (`multi-user.target`, gdm disabled, packagekit masked);
  `coord_nav` made **system-wide** so a headless boot still connects.

### Findings (the important gotchas)
- **Rear motor ports M2/M4 are swapped in hardware** (rear-left ↔ rear-right)
  and the rear plugs are polarity-reversed. Fixed in car_base's `/wheel_cmd`
  handler: `rl→M4, rr→M2`, both negated. Verified wheel-by-wheel.
- **The mecanum wheels were mounted wrong** (side-paired: both left wheels one
  roller handedness, both right the other), not the standard DIAGONAL-paired X
  (FL & RR same, FR & RL same). Forward worked but strafe just rotated. Fixed by
  physically **swapping the two rear wheels** → standard config, standard
  kinematics table. Diagonal moves (2-wheel drive) reach only half a straight
  move's per-axis distance, and strafe drags, so client-side multipliers
  (`diag_mult` ~1.6, `strafe_mult` ~1.2, both GUI-tunable, apply-based) scale
  their magnitude to keep grid steps even. The GUI base-magnitude ceiling is
  80/max(mult) so `base x mult` never exceeds the car's 80 cap.
- **Cross-distro message md5**: `sensor_msgs/BatteryState` differs between the
  car's Melodic and the workstation's Noetic, so the client CANNOT subscribe to
  `/battery` (connection dropped). Worked around by republishing voltage as
  `std_msgs/Float32` on `/battery_v`. Prefer std_msgs types for new car→client
  topics. (`LaserScan`/`Odometry`/`Float32MultiArray`/`String` are compatible.)
- **The MCU firmware has NO motor timeout.** Both `set_motor` (raw PWM) and
  `set_car_motion` (velocity) latch the last command forever — verified by
  sending one command and watching it run for 3 s. So a single lost "stop" =
  permanent runaway. The RC remote is safe only because it streams commands
  continuously (it never goes silent); the Nano can (it hangs).
- **Runaway root cause**: commanding max magnitude drives peak current past what
  the source can supply → brownout, and/or the Nano CPU saturates → the stop
  loop stalls → the MCU latches → the car keeps moving. **WiFi is onboard (not
  USB)**, so the "network drop" during a runaway is CPU saturation, not a
  power/USB glitch. When the Nano fully hangs, only a **physical power-cut**
  stops the car — no host-side software can.
- **The frequent lidar "disconnected" was self-inflicted.** An `rplidar-watchdog`
  service (added, then **removed**) misfired: its own node wasn't receiving
  `/scan`, so it judged the lidar wedged and `pkill`'d a perfectly healthy lidar
  every ~27 s — each kill a "disconnect". `dmesg` showed NO USB disconnects; the
  lidar hardware/connector is fine. With it gone, `/scan` is rock-solid.
- **Never USB unbind/rebind on the car** — it ripples the USB bus and disturbs
  things (and risks the network). Recover a genuinely wedged lidar with
  `sudo systemctl restart car-ros` (+ wait ~15 s) or `sudo reboot`.
- `rostopic hz`/`echo` often hang ignoring `timeout`; use a bounded python
  `wait_for_message`. `pkill -f <x>` matches the shell running it if the command
  text contains `<x>` — use `pkill -x <name>` or kill by PID.

### Safety layers (host-side; a **physical motor E-STOP is still recommended**)
- **Magnitude hard-capped at 80** (car-side, final applied magnitude). The
  earlier 50 cap came from a mis-diagnosed "brownout" — the real instability was
  the removed rplidar-watchdog. The GUI's base-magnitude ceiling is 80/diag_mult
  so `base x diag_mult` can never exceed 80.
- **Sustained brake**: drive_action holds `[0,0,0,0]` for ~1.5 s after every move
  (~30 messages), so a few dropped stops can't latch.
- Conservative defaults (magnitude 40, duration 0.8 s); hard duration cap 3 s.
- **GUI**: MCU-link banner (green/red), steering disabled on link loss, spacebar
  = E-STOP, obstacle disconnect logged as ERR.
- **Overload audit**: rates capped ~3 Hz (car_base 20→10 Hz, `/battery_v` 20→3 Hz,
  `/obstacles` 3 Hz, GUI refresh 3 Hz); queues bounded (client history
  `deque(maxlen=100)`, GUI display 200 lines); GUI log file rotates at 5 MB.
- **E-STOP** = SSH `sh ~/estop.sh` (ROS-independent; brings car-ros down;
  recover with `sudo systemctl start car-ros`).
