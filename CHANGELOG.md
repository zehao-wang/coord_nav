# Changelog

Findings and notable changes. README is for day-to-day usage; this file records
*why* things are the way they are (hard-won during bring-up).

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
