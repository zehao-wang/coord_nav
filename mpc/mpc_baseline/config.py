"""Tunable parameters for the MPC baseline, as plain dataclasses.

Everything the planner, simulator and live runner need is here so a run is fully
described by one config object. Per-variant factory helpers give sensible
starting points: `sim_config_v1/_v2()` (nominal speeds, for offline development)
and `live_config_v1/_v2()` (the validated magnitude-40 profile for the real car).

Frames & units throughout the package:
  * Planning happens in the ODOM frame: state = [x, y, theta], metres/radians,
    x forward of the car's start pose, theta CCW.
  * Obstacles arrive from /obstacles in the BASE frame (x fwd, y left) and are
    transformed into odom via the current /odom pose before planning.
  * Body velocity is (vx, vy, wz): vx forward, vy left, wz yaw-rate (CCW+).
  * Wheel order is [FL, RL, FR, RR], forward-positive PWM, matching /wheel_cmd.
"""

from dataclasses import dataclass, field
from typing import Tuple


# --- physical / kinematic ------------------------------------------------
@dataclass
class RobotConfig:
    """Chassis geometry and the PWM<->velocity mapping.

    MEASURED on the car with calibration/calib_model.py (2026-07-25 and 2026-08-04),
    not guessed. The
    plant is AFFINE, not proportional -- the motors do not turn until PWM clears a
    friction threshold:

        wheel m/s = (PWM - pwm_offset) / pwm_per_mps

    Measured at three battery voltages, PWM 30/40/60 per axis (fit residuals
    +-0.003 m/s each):

        9.8 V : pwm_per_mps 73.4, pwm_offset 17.0, arm 0.194
        10.5 V: pwm_per_mps 72.1, pwm_offset 14.0, arm 0.198   <- shipped k/offset
        11.1 V: pwm_per_mps 68.4, pwm_offset 13.2, arm 0.194
                (wz_arm/steer_arm ship at 0.196, mid of the three arms; the third
                point validated the voltage extrapolation to 2.5 % at magnitude 40,
                so the 10.5 V k/offset stay -- the closed loop absorbs the rest)

    Voltage sensitivity is therefore SMALL for the slope (1.8 %) and for the arm
    (2 %), and larger for the friction threshold (17 -> 14 PWM: more torque is
    available, so a lower PWM breaks stiction). The arm is a geometry ratio, so
    measuring both axes at the SAME voltage cancels the voltage out -- that is the
    number to trust, and it is why all three sessions agree on it.

    Once the offset is accounted for the arm nearly stops drifting with PWM
    (0.216/0.196/0.184 at PWM 30/40/60, -15 % end to end, mean 0.198); under the
    old proportional model it slid 0.168 -> 0.095, a -43 % drift, i.e.
    a friction offset was masquerading as wrong geometry. See mpc/README.md.
    """
    robot_radius: float = 0.13        # half-footprint of the mecanum car (m)
    pwm_per_mps: float = 72.1         # MEASURED slope (was a nominal 200)
    pwm_offset: float = 14.0          # MEASURED friction threshold, PWM
    wz_arm: float = 0.196             # MEASURED yaw arm (m); was a nominal 0.5
    # Yawing scrubs the mecanum rollers sideways, which costs its own threshold
    # torque on top of the per-wheel friction. MEASURED w_real = 1.079*(w_cmd-0.219)
    # at v=0.361 (38/67/75/92 % of command at |w| 0.30/0.60/0.90/1.20 -- a rising
    # percentage is a deadband, a lag would cost the same fraction everywhere).
    yaw_deadband: float = 0.219       # rad/s that buys no yaw at all
    yaw_gain: float = 1.079           # slope above the deadband
    wheel_pwm_cap: float = 80.0       # clamp on any wheel PWM (car caps at 100)
    diag_mult: float = 1.6            # diagonal magnitude boost (matches carclient)
    strafe_mult: float = 1.2          # strafe magnitude boost (matches carclient)


# --- sampling-based MPC (MPPI) core --------------------------------------
@dataclass
class MPPIConfig:
    horizon: int = 29                 # H rollout steps; H*dt ~= 9.7 s ~= 1.9-2.1 m lookahead.
                                      # Longer than variant 2's 4: differential steering must
                                      # see a wide/near obstacle early to turn out in time
    samples: int = 400                # K sampled control sequences
    dt: float = 1.0 / 3.0             # step time (s) == TickConfig.period. The FIRST planned
                                      # step is executed for exactly one tick, so the model's
                                      # step must BE a tick or the rollout predicts a motion
                                      # the car never performs. (Was 0.6 s against a 0.25 s
                                      # execution period -- H was 16 to keep the same 9.6 s.)
    noise_v: float = 0.10             # exploration std, m/s
    noise_w: float = 0.7              # exploration std, rad/s
    noise_tau: float = 0.8425         # AR(1) smoothing TIME CONSTANT (s): beta = exp(-dt/tau).
                                      # In SECONDS, not per-step, so a sampled manoeuvre stays
                                      # the same MANOEUVRE if the tick rate changes (a per-step
                                      # beta silently halves the smoothing horizon when dt does).
                                      # 0.8425 s == beta 0.673 at the default 1/3 s tick (the
                                      # per-step beta it replaced was 0.7).
                                      # Larger = longer sustained turns.
    n_iters: int = 3                  # sample -> argmin -> resample refinement passes / cycle


@dataclass
class CostConfig:
    """Weights for the shared cost. Distances are in metres."""
    # Running terms are INTEGRALS (weight x sum x dt), so they no longer change
    # meaning when the horizon or the step time changes. Values below are the
    # car-validated raw-sum tuning divided by the old dt=0.6 (x0.6 for w_smooth,
    # which is a rate), so the numbers are identical to what flew before.
    w_goal_run: float = 1.6667        # per-second distance-to-goal (was 1.0 per STEP)
    w_goal_term: float = 12.0         # terminal distance-to-goal (single term, no dt)
    w_track: float = 4.1667           # cross-track: pull back onto the straight A->B line after
                                      # a detour (variant 1). Higher = tighter, but too high
                                      # fights the wide detour needed around a wall on the line
    w_obs: float = 100.0              # soft barrier weight inside the buffer
    obs_buffer: float = 0.15          # start pushing away this far outside inflation (m)
    extra_margin: float = 0.10        # added to (circle.r + robot_radius) inflation (clearance)
    w_ctrl_v: float = 0.0333          # penalise speed lightly (prefer efficient paths)
    w_ctrl_w: float = 0.25            # penalise yaw-rate (prefer going straight)
    w_smooth: float = 0.24            # penalise the control RATE of change WITHIN the horizon
    w_cont: float = 0.0               # OFF. Penalises the first step's distance from the
                                      # control the car is ALREADY executing (w_smooth only
                                      # couples steps within one horizon, nothing couples
                                      # consecutive ticks). It does smooth the command --
                                      # measured on the car, sign reversals 0.26 -> 0.06 per
                                      # tick and mean |dw| 0.37 -> 0.15 at w_cont=0.6 -- but
                                      # the car then STOPPED REACHING B: 0/2 runs, both timing
                                      # out at 14.3 s with the yaw pinned at the cap 42 % of
                                      # ticks, against 3/3 in 5.3 s with it off. Offline it
                                      # looked free (1.000 success, less jitter) because the
                                      # sim has no disturbances, so being sluggish costs
                                      # nothing there. Keep at 0 until the sim models
                                      # disturbance; the mechanism is kept for that work.
    collision_cost: float = 1.0e6     # hard penalty for a rollout that hits inflation


@dataclass
class GoalConfig:
    """Goal B, in the car's START body frame: goal_dist forward (x), goal_y left
    (y). B = start_xy + R(start_yaw) @ (goal_dist, goal_y). Coordinates are
    relative to where the car started."""
    goal_dist: float = 1.0            # metres forward of the start pose
    goal_y: float = 0.0               # metres left of the start pose (0 = straight ahead)
    goal_tol: float = 0.15            # reached when within this (m)
    timeout_s: float = 25.0           # abort the episode after this long


# --- continuous variant 1: (v, omega) ------------------------------------
@dataclass
class Variant1Config:
    v_min: float = 0.0                # m/s -- no reverse/stop: keep moving to B
    v_max: float = 0.22               # m/s forward cap for OFFLINE work; the live factories
                                      # derive it from the magnitude via the affine plant
                                      # ((PWM-offset)/k), e.g. mag 40 -> 0.361 m/s
    w_max: float = 1.2                # yaw-rate cap => min turn radius v/w_max ~0.30 m at mag 40.
                                      # Only a
                                      # CAP; the cost keeps turns gentle unless an obstacle needs
                                      # the sharper turn. Inner wheel still rolls forward at it.
    # Differential drive: all wheels forward, steer by L/R speed difference; the constraint
    # keeps the inner wheel >= min_inner_frac * v so it never pivots (smoothness is w_smooth).
    steer_arm: float = 0.196          # MEASURED yaw arm (m) so commanded w == actual yaw
                                      # rate; also the inner-wheel-constraint arm. The old
                                      # 0.10 let the planner ask for turns twice as tight as
                                      # the car can do, which is why it spiralled.
    min_inner_frac: float = 0.1       # inner wheel speed >= this fraction of v (keeps it rolling)
    predict_obstacles: bool = False   # True = the TIME-AWARE variant (mpc_vw_t): the
                                      # obstacle cost evaluates rollout step h against the
                                      # tracker's constant-velocity prediction at t+h*dt
                                      # instead of the frozen current circles
    pred_extra_delay_s: float = 0.0   # added to every prediction time: the loop's
                                      # dispatch buffering. The live runner and the
                                      # buffered (disturbed) sim start executing a plan
                                      # ONE TICK after the frame it was planned from, so
                                      # they set this to the tick period (runner.py /
                                      # resolve_policy); the unbuffered offline loop
                                      # executes immediately and leaves it 0
    robot: RobotConfig = field(default_factory=RobotConfig)
    mppi: MPPIConfig = field(default_factory=MPPIConfig)
    cost: CostConfig = field(default_factory=CostConfig)
    goal: GoalConfig = field(default_factory=GoalConfig)


# --- discrete variant 2: grid-hop over the mecanum action set ------------
@dataclass
class Variant2Config:
    # Which discrete action ids the planner may pick (see carclient.Action).
    # Default: the 8 translation directions, NO STOP and no in-place rotation --
    # the car hops every cycle and never stops before B (continuous control). Add
    # 0 to allow STOP, or 9/10 to allow rotation.
    actions: Tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7, 8)
    step_magnitude: float = 40.0      # PWM magnitude per hop (sim); live overrides
    step_duration: float = 0.5        # the hop's LIFE on the car (s). Must outlive one tick
                                      # so the next tick SUPERSEDES it rather than it expiring
                                      # into a brake. NOT the rollout step -- see rollout_dt.
    rollout_dt: float = None          # model step for the rollout; the runner sets it to the
                                      # tick period, because ONE tick of the hop is what
                                      # actually gets executed before the next one replaces it.
                                      # None = fall back to step_duration (offline default).
    horizon: int = 4                  # hops looked ahead
    samples: int = 1024               # sampled action sequences (if not exhaustive)
    exhaustive_cap: int = 20000       # enumerate all seqs when |A|^H <= this
    predict_obstacles: bool = False   # True = the TIME-AWARE variant (mpc_grid_t);
                                      # see Variant1Config.predict_obstacles
    pred_extra_delay_s: float = 0.0   # dispatch buffering, see Variant1Config
    robot: RobotConfig = field(default_factory=RobotConfig)
    cost: CostConfig = field(default_factory=CostConfig)
    goal: GoalConfig = field(default_factory=GoalConfig)


# --- execution disturbance (for the offline sim) --------------------------
@dataclass
class DisturbanceConfig:
    """How the real car deviates from the commanded body velocity, FIT from 298
    clean on-car ticks (six 2026-07-25 runs, wedge-contaminated segments removed;
    see calibration/results/ and CHANGELOG 0.9.12).

    Exists because the undisturbed sim mis-ranks controllers: w_cont=0.6 looked
    FREE offline (1.000 success, less jitter) and went 0/2 on the car -- in a
    perfect-execution world a sluggish controller never pays for being unable to
    correct. Enable this to make offline smoothness/robustness tuning transfer.

        v: multiplicative per-tick noise      v_exec = v_cmd * N(v_gain, v_std)
        w: first-order lag + additive noise   w_exec' = a*w_exec + b*w_cmd + N(0,w_noise)
           with a = exp(-dt / w_tau); b fixed by w_b at the fitted dt=1/3 s.

    The w fit: a=0.498, b=0.390, residual 0.171 rad/s, R^2 0.885 (vs 0.700 for
    zero-lag) -- a lag constant of 0.48 s, ~1.4 ticks, which is exactly why
    commands that flip sign every tick deliver only ~0.6-0.7 of their yaw.
    """
    v_gain: float = 0.946             # mean of measured/commanded arc speed per tick
    v_std: float = 0.268              # its std (includes odom sampling jitter, which
                                      # the real loop also sees, so it belongs here)
    w_tau: float = 0.48               # yaw lag time constant (s); a = exp(-dt/w_tau)
    w_b: float = 0.390                # input gain at the FITTED dt (1/3 s); scaled
                                      # with dt so steady-state gain stays put
    w_noise: float = 0.171            # additive yaw-rate noise per tick (rad/s)


# --- obstacle handling (base->odom, rolling memory) ----------------------
@dataclass
class ObstacleConfig:
    mem_time_s: float = 1.5           # keep obstacles seen this recently (0 = current frame only)
    mem_radius: float = 3.0           # drop remembered obstacles farther than this from the car
    merge_dist: float = 0.15          # fuse remembered circles closer than this (m)
    max_age_stale: float = 1.0        # ignore an /obstacles frame older than this (s)
    # Constant-velocity tracking (used by the *_t time-aware policies; pure
    # bookkeeping for everything else). Gates are what keep a STATIC world
    # byte-identical to the frozen-world planner: a track's velocity reads 0
    # until it has been sighted vel_min_sightings times AND its EMA speed clears
    # vel_deadband.
    vel_ema: float = 0.5              # EMA weight on each new raw velocity sample
    vel_cap: float = 1.0              # m/s clamp: a bad association can't invent a sprinter
    vel_min_sightings: int = 3        # frames a track must be seen before its v counts
    vel_deadband: float = 0.04        # m/s below which a track is "static" (perception
                                      # jitter on a real box measures well under this)
    pred_cap_s: float = 2.5           # cap on TOTAL extrapolation (age + horizon step):
                                      # beyond ~2.5 s a constant-velocity guess is fiction
                                      # (bounces, stops), so the prediction holds there.
                                      # v1's 9.7 s horizon tail plans against the held
                                      # position, not a 9.7 s straight-line ghost


# --- the global tick -----------------------------------------------------
@dataclass
class TickConfig:
    """THE one rate the whole stack runs on -- game-engine style.

    A tick is one observation frame. Inside a tick: read the observation, run the
    safety checks, DISPATCH the action decided last tick (it overrides whatever the
    car is still running), then plan the action for the next tick. Emit nothing and
    the car simply keeps executing its current command; let that command expire and
    the car holds its state (the car-side node brakes at end_time).

    rate_hz must equal the car's perception rate (`rate_hz` of obstacle_circles in
    viz.launch) -- that is the real clock, everything else follows it. Planning
    faster only re-plans on frames already seen; slower drops frames outright.
    """
    rate_hz: float = 3.0              # == obstacle_circles rate_hz on the car
    action_ticks: float = 1.5         # a dispatched action is given this many ticks of
                                      # life on the car. >1 so ONE dropped command does
                                      # not stutter the wheels; <2 so two in a row let it
                                      # expire and the car brakes instead of running on.
    wait_ticks: float = 2.0           # give up waiting for a new frame after this many
                                      # ticks and run the tick anyway (safety still runs)

    @property
    def period(self):
        """Seconds per tick."""
        return 1.0 / self.rate_hz

    @property
    def action_duration(self):
        """Duration (s) handed to the car with each dispatched action."""
        return self.action_ticks / self.rate_hz


# --- live runner (real car) ----------------------------------------------
@dataclass
class LiveConfig:
    tick: TickConfig = field(default_factory=TickConfig)
    execute_steps: int = 1            # planned steps to apply before re-planning (1 = tight
                                      # closed loop; >1 = N steps open-loop). Safety runs each step
    magnitude: float = 40.0           # PWM. The value the car is actually VALIDATED at: five
                                      # 5 m runs reached B (final distance 0.05-0.11 m), and it
                                      # is the only magnitude the yaw feedforward was measured
                                      # at end to end. Gives v_max (40-14)/72.1 = 0.361 m/s,
                                      # full yaw to w_max 1.2, turn radius 0.30 m.
                                      # Tight suite over 20 seeds: 0.333 at magnitude 20 (all
                                      # timeouts -- too slow to steer, and only 0.176 rad/s of
                                      # yaw), 0.992 at 30, 0.975 at 40 (sweep taken under the
                                      # pre-0.9.15 centre-based collision metric: fine for the
                                      # ranking, not as safety rates). Below magnitude 17.4 the
                                      # achievable yaw is exactly zero and build_live_cfg
                                      # refuses. "Start small" was 20, chosen when the old
                                      # proportional model claimed it meant 0.10 m/s; measured,
                                      # it is 0.083 m/s.
    estop_on_link_loss: bool = True   # estop (not soft stop) if MCU link drops
    link_wait_s: float = 3.0          # refuse to drive if the MCU link isn't healthy within this
    collision_abort: bool = True      # stop if an obstacle surface reaches the footprint
    collision_margin: float = 0.0     # guard fires this far INSIDE robot_radius; 0 = on contact
                                      # (positive would false-trip on a legit graze, stop short of B)
    # An abnormal exit / Ctrl-C ALWAYS hard-estops (MCU has no motor timeout -> a dropped
    # soft stop would latch the wheels).


# The motors' friction threshold rises as the pack drains: measured 14.0 PWM at
# 10.5 V and 17.0 at 9.8 V, about -4.3 PWM per volt. Safety checks that must hold on
# a TIRED battery add this much to the shipped offset. (Speed accuracy does not need
# it -- the closed loop absorbs the whole 9.5-12 V spread; see calibration/README.md.)
PWM_OFFSET_LOW_BATT_MARGIN = 4.5


def _v_of_pwm(pwm, robot):
    """Speed ceiling for a PWM ceiling, through the affine plant."""
    return max(0.0, (pwm - robot.pwm_offset) / robot.pwm_per_mps)


def sim_config_v1() -> Variant1Config:
    c = Variant1Config()
    c.robot.wz_arm = c.steer_arm      # actuation mix arm == steer constraint arm
    return c


def sim_config_v2() -> Variant2Config:
    return Variant2Config()


def live_config_v1() -> Variant1Config:
    """Conservative variant-1 profile for the real car (LiveConfig.magnitude)."""
    c = Variant1Config()
    # magnitude 30 -> PWM 30 -> ~0.22 m/s ceiling; keep yaw gentle too.
    c.v_max = _v_of_pwm(LiveConfig.magnitude, c.robot)
    c.v_min = 0.0
    c.robot.wz_arm = c.steer_arm      # actuation mix arm == steer constraint arm
    c.robot.wheel_pwm_cap = 60.0      # hard ceiling on any wheel PWM live
    return c


def live_config_v2() -> Variant2Config:
    """Conservative variant-2 profile for the real car (LiveConfig.magnitude)."""
    c = Variant2Config()
    c.step_magnitude = LiveConfig.magnitude
    return c


def build_live_cfg(variant, magnitude, goal_dist, goal_y=0.0, goal_tol=0.15,
                   timeout_s=25.0, step_duration=0.5, pwm_offset=None,
                   allow_rotation=False):
    """One live config for a variant, from the operator's magnitude + goal B.
    goal_dist/goal_y are B's forward/left coordinates relative to the start pose.
    Shared by run_live.py and the GUI so both build the policy identically."""
    # Explicit on BOTH sides + raise, like policies.make_policy. This used to be
    # `if v1: ... else: v2`, so "V1"/"velocity"/a typo silently returned a discrete
    # Variant2Config -- and the recipe in mpc/README.md tells model authors to call
    # this by hand. A velocity policy handed a Variant2Config has no .v_max/.mppi,
    # and build_policy's action_space check compares the entry against the POLICY,
    # never against the cfg, so it does not catch it either.
    v = str(variant).lower()
    if v in ("1", "vw", "v1", "velocity"):
        c = live_config_v1()
        if pwm_offset is not None:
            c.robot.pwm_offset = pwm_offset
        # magnitude is the PWM ceiling, so the speed ceiling is the affine inverse
        # of it -- NOT magnitude/pwm_per_mps, which ignores the friction offset and
        # over-states v_max by pwm_offset/pwm_per_mps (0.19 m/s at these constants).
        c.v_max = _v_of_pwm(magnitude, c.robot)
        c.robot.wheel_pwm_cap = max(c.robot.wheel_pwm_cap, magnitude * 1.8)
    elif v in ("2", "grid", "v2", "discrete"):
        c = live_config_v2()
        c.step_magnitude = magnitude
        c.step_duration = step_duration
        if allow_rotation:
            c.actions = tuple(c.actions) + (9, 10)
    else:
        raise ValueError(
            "build_live_cfg: unknown variant %r -- use 1/'vw'/'velocity' for the "
            "continuous (v,w) config or 2/'grid'/'discrete' for the mecanum-action "
            "config. (This used to fall through to variant 2, handing a velocity "
            "policy a Variant2Config with no .v_max/.mppi.)" % (variant,))
    if v in ("1", "vw", "v1", "velocity"):
        # Below the magnitude where v clears yaw_deadband*arm/(1-min_inner_frac),
        # the policy's achievable-yaw clip is identically 0: the planner would drive
        # dead straight with no steering channel, so the obstacle cost could not
        # avoid anything. Refuse rather than roll out of the driveway in a line.
        # Use a DEPLETED-pack offset for this check, not the shipped one. The friction
        # threshold moves with battery voltage -- measured 17.0 PWM at 9.8 V and 14.0
        # at 10.5, i.e. about -4.3 PWM per volt -- so a pack at 9.5 V needs ~18.3 and
        # the shipped 14.0 would wave through a magnitude that cannot steer at all.
        v_floor = c.robot.yaw_deadband * c.steer_arm / max(1e-6, 1.0 - c.min_inner_frac)
        worst_offset = c.robot.pwm_offset + PWM_OFFSET_LOW_BATT_MARGIN
        if (magnitude - worst_offset) / c.robot.pwm_per_mps <= v_floor:
            raise ValueError(
                "magnitude %.0f gives v_max %.3f m/s, at or below the %.3f m/s where "
                "the car can produce ANY yaw (yaw_deadband %.3f rad/s x arm %.3f). The "
                "planner would drive straight and could not steer around anything on a "
                "depleted pack. Use magnitude >= %.0f; 40 is the validated value."
                % (magnitude, c.v_max, v_floor, c.robot.yaw_deadband, c.steer_arm,
                   v_floor * c.robot.pwm_per_mps + worst_offset + 1))
    c.goal.goal_dist = goal_dist
    c.goal.goal_y = goal_y
    c.goal.goal_tol = goal_tol
    c.goal.timeout_s = timeout_s
    return c
