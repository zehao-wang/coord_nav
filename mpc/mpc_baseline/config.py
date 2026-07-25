"""Tunable parameters for the MPC baseline, as plain dataclasses.

Everything the planner, simulator and live runner need is here so a run is fully
described by one config object. Two factory helpers give sensible starting
points: `sim_config()` (nominal speeds, for offline development) and
`live_config()` (deliberately slow, magnitude ~20, for the real car).

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

    pwm_per_mps and wz_arm mirror car_base_node.py so the model matches what the
    real chassis does. They are only nominal; the closed-loop replanning absorbs
    scale error (calibrate with scripts/calibrate_goal.py if you want B exact).
    """
    robot_radius: float = 0.13        # half-footprint of the mecanum car (m)
    pwm_per_mps: float = 200.0        # car_base PWM = m/s * this (mag 40 -> 0.2 m/s)
    wz_arm: float = 0.5               # yaw mixing arm (m/rad), matches car_base WZ_ARM
    wheel_pwm_cap: float = 80.0       # clamp on any wheel PWM (car caps at 100)
    deadzone_pwm: float = 0.0         # if >0: bump a nonzero wheel cmd up to this
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
                                      # 0.8425 s == the shipped beta=0.7 at the default tick.
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
    w_smooth: float = 0.24            # penalise the control RATE of change -> gradual steering
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
    v_max: float = 0.22               # m/s forward cap (nominal); live lowers this
    w_max: float = 1.2                # yaw-rate cap => min turn radius v/w_max ~0.17 m. Only a
                                      # CAP; the cost keeps turns gentle unless an obstacle needs
                                      # the sharper turn. Inner wheel still rolls forward at it.
    # Differential drive: all wheels forward, steer by L/R speed difference; the constraint
    # keeps the inner wheel >= min_inner_frac * v so it never pivots (smoothness is w_smooth).
    steer_arm: float = 0.1            # PHYSICAL half-track (m), CALIBRATED so commanded w ==
                                      # actual yaw rate; also the inner-wheel-constraint arm
    min_inner_frac: float = 0.1       # inner wheel speed >= this fraction of v (keeps it rolling)
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
    step_duration: float = 0.5        # seconds per hop ("cell" size)
    horizon: int = 4                  # hops looked ahead
    samples: int = 1024               # sampled action sequences (if not exhaustive)
    exhaustive_cap: int = 20000       # enumerate all seqs when |A|^H <= this
    robot: RobotConfig = field(default_factory=RobotConfig)
    cost: CostConfig = field(default_factory=CostConfig)
    goal: GoalConfig = field(default_factory=GoalConfig)


# --- obstacle handling (base->odom, rolling memory) ----------------------
@dataclass
class ObstacleConfig:
    mem_time_s: float = 1.5           # keep obstacles seen this recently (0 = current frame only)
    mem_radius: float = 3.0           # drop remembered obstacles farther than this from the car
    merge_dist: float = 0.15          # fuse remembered circles closer than this (m)
    max_age_stale: float = 1.0        # ignore an /obstacles frame older than this (s)


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
    magnitude: float = 20.0           # START SMALL on the real car
    estop_on_link_loss: bool = True   # estop (not soft stop) if MCU link drops
    link_wait_s: float = 3.0          # refuse to drive if the MCU link isn't healthy within this
    collision_abort: bool = True      # stop if an obstacle surface reaches the footprint
    collision_margin: float = 0.0     # guard fires this far INSIDE robot_radius; 0 = on contact
                                      # (positive would false-trip on a legit graze, stop short of B)
    # An abnormal exit / Ctrl-C ALWAYS hard-estops (MCU has no motor timeout -> a dropped
    # soft stop would latch the wheels).


def sim_config_v1() -> Variant1Config:
    c = Variant1Config()
    c.robot.wz_arm = c.steer_arm      # actuation mix arm == steer constraint arm
    return c


def sim_config_v2() -> Variant2Config:
    return Variant2Config()


def live_config_v1() -> Variant1Config:
    """Slow, conservative variant-1 profile for the real car (magnitude ~20)."""
    c = Variant1Config()
    # magnitude 20 -> PWM 20 -> ~0.10 m/s ceiling; keep yaw gentle too.
    c.v_max = LiveConfig.magnitude / c.robot.pwm_per_mps   # 0.10 m/s
    c.v_min = 0.0
    c.robot.wz_arm = c.steer_arm      # actuation mix arm == steer constraint arm
    c.robot.deadzone_pwm = 30.0       # keep the inner wheel actually spinning in a turn
    c.robot.wheel_pwm_cap = 35.0      # hard ceiling on any wheel PWM live
    return c


def live_config_v2() -> Variant2Config:
    """Slow, conservative variant-2 profile for the real car (magnitude 20)."""
    c = Variant2Config()
    c.step_magnitude = LiveConfig.magnitude
    return c


def build_live_cfg(variant, magnitude, goal_dist, goal_y=0.0, goal_tol=0.15,
                   timeout_s=25.0, step_duration=0.5, deadzone_pwm=30.0,
                   allow_rotation=False):
    """One live config for a variant, from the operator's magnitude + goal B.
    goal_dist/goal_y are B's forward/left coordinates relative to the start pose.
    Shared by run_live.py and the GUI so both build the policy identically."""
    if str(variant) in ("1", "vw", "v1"):
        c = live_config_v1()
        c.v_max = magnitude / c.robot.pwm_per_mps
        c.robot.wheel_pwm_cap = max(c.robot.wheel_pwm_cap, magnitude * 1.8)
        c.robot.deadzone_pwm = deadzone_pwm
    else:
        c = live_config_v2()
        c.step_magnitude = magnitude
        c.step_duration = step_duration
        if allow_rotation:
            c.actions = tuple(c.actions) + (9, 10)
    c.goal.goal_dist = goal_dist
    c.goal.goal_y = goal_y
    c.goal.goal_tol = goal_tol
    c.goal.timeout_s = timeout_s
    return c
