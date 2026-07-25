"""Mecanum kinematics and vectorised rollout dynamics (pure numpy, no ROS).

Two jobs:
  1. Convert between body velocity (vx, vy, wz) and the four wheel PWMs, using the
     SAME convention car_base_node.py's /wheel_cmd handler expects: logical wheel
     order [FL, RL, FR, RR], forward-positive. (The car then remaps physical
     ports / rear polarity internally in _set_wheels -- we never see that here.)
  2. Provide the discrete action -> body-velocity table (matching what the car's
     drive_action node + carclient.drive actually produce) and the vectorised
     forward integrators the planner and simulator both roll trajectories with.

Standard mecanum mixing (x fwd, y left, diagonal-paired rollers):
    FL = vx - vy - wz*arm
    RL = vx + vy - wz*arm
    FR = vx + vy + wz*arm
    RR = vx - vy + wz*arm
This is the classic form; car_base uses the identical mix (its WZ_ARM == arm).
"""

import numpy as np

# Discrete mecanum action wheel patterns [FL, RL, FR, RR], unit +-1/0 -- a 1:1
# copy of drive_action_node.ACTIONS so the model mirrors the real actuator.
ACTION_PATTERNS = {
    0:  (0.0,  0.0,  0.0,  0.0),   # STOP
    1:  (1.0,  1.0,  1.0,  1.0),   # forward
    2:  (0.0,  1.0,  1.0,  0.0),   # forward-left
    3:  (-1.0, 1.0,  1.0, -1.0),   # strafe-left
    4:  (-1.0, 0.0,  0.0, -1.0),   # back-left
    5:  (-1.0, -1.0, -1.0, -1.0),  # back
    6:  (0.0, -1.0, -1.0,  0.0),   # back-right
    7:  (1.0, -1.0, -1.0,  1.0),   # strafe-right
    8:  (1.0,  0.0,  0.0,  1.0),   # forward-right
    9:  (-1.0, -1.0, 1.0,  1.0),   # rotate-ccw
    10: (1.0,  1.0, -1.0, -1.0),   # rotate-cw
}

_DIAGONALS = frozenset([2, 4, 6, 8])
_STRAFE = frozenset([3, 7])


def mix_to_wheels(vx, vy, wz, arm):
    """Body velocity (vx, vy, wz) -> wheel values [FL, RL, FR, RR] (same units
    as the inputs; multiply by pwm_per_mps to get PWM)."""
    fl = vx - vy - wz * arm
    rl = vx + vy - wz * arm
    fr = vx + vy + wz * arm
    rr = vx - vy + wz * arm
    return np.array([fl, rl, fr, rr], dtype=float)


def wheels_to_body(wheels, arm):
    """Inverse of mix_to_wheels. wheels = [FL, RL, FR, RR] -> (vx, vy, wz)."""
    fl, rl, fr, rr = wheels
    vx = (fl + rl + fr + rr) / 4.0
    vy = (-fl + rl + fr - rr) / 4.0
    wz = (-fl - rl + fr + rr) / (4.0 * arm)
    return vx, vy, wz


def velocity_to_wheel_pwm(vx, vy, wz, robot):
    """Body velocity (m/s, rad/s) -> clipped wheel PWM [FL, RL, FR, RR] ready for
    /wheel_cmd. Applies the optional deadzone bump so a small-but-nonzero command
    actually moves the motors (car deadzone is ~PWM 30)."""
    pwm = mix_to_wheels(vx, vy, wz, robot.wz_arm) * robot.pwm_per_mps
    if robot.deadzone_pwm > 0.0:
        small = (np.abs(pwm) > 1e-6) & (np.abs(pwm) < robot.deadzone_pwm)
        pwm[small] = np.sign(pwm[small]) * robot.deadzone_pwm
    return np.clip(pwm, -robot.wheel_pwm_cap, robot.wheel_pwm_cap)


def action_effective_magnitude(action_id, magnitude, robot):
    """Magnitude carclient.drive() actually sends for this action (diagonals and
    strafe get boosted so the grid cells stay even)."""
    if action_id in _DIAGONALS:
        return magnitude * robot.diag_mult
    if action_id in _STRAFE:
        return magnitude * robot.strafe_mult
    return magnitude


def action_body_velocity(action_id, magnitude, robot):
    """Discrete action -> (vx, vy, wz) body velocity in m/s, rad/s, exactly as the
    real drive_action pulse produces it (pattern x effective-magnitude PWM,
    inverted through the mecanum mix)."""
    eff = action_effective_magnitude(action_id, magnitude, robot)
    wheels_pwm = np.array(ACTION_PATTERNS[action_id], dtype=float) * eff
    wheels_mps = wheels_pwm / robot.pwm_per_mps
    return wheels_to_body(wheels_mps, robot.wz_arm)


def build_action_table(action_ids, magnitude, robot):
    """Precompute a (len(action_ids), 3) array of body velocities (vx, vy, wz)
    for the given action ids, plus the id list, for fast rollout lookup."""
    ids = list(action_ids)
    table = np.array([action_body_velocity(a, magnitude, robot) for a in ids],
                     dtype=float)
    return np.array(ids, dtype=int), table


# --- vectorised forward integrators --------------------------------------
def rollout_unicycle(x0, controls, dt):
    """Unicycle model for variant 1.

    x0       : (3,) start state [x, y, theta] in odom.
    controls : (K, H, 2) sequences of [v, w].
    returns  : (K, H, 3) states after each step.
    Uses the heading at the start of each step (semi-explicit Euler); dt is small
    so the integration error is negligible against the replanning cadence.
    """
    K, H, _ = controls.shape
    states = np.empty((K, H, 3), dtype=float)
    x = np.tile(np.asarray(x0, dtype=float), (K, 1))
    for h in range(H):
        v = controls[:, h, 0]
        w = controls[:, h, 1]
        th = x[:, 2]
        x[:, 0] = x[:, 0] + v * np.cos(th) * dt
        x[:, 1] = x[:, 1] + v * np.sin(th) * dt
        x[:, 2] = x[:, 2] + w * dt
        states[:, h, :] = x
    return states


def rollout_body(x0, body_vel, dt):
    """Holonomic model for variant 2 (and the simulator).

    x0       : (3,) start state [x, y, theta] in odom.
    body_vel : (K, H, 3) sequences of body velocity [vx, vy, wz].
    returns  : (K, H, 3) states after each step (each step lasts dt seconds).
    Body velocity is rotated into odom by the heading at the start of the step.
    """
    K, H, _ = body_vel.shape
    states = np.empty((K, H, 3), dtype=float)
    x = np.tile(np.asarray(x0, dtype=float), (K, 1))
    for h in range(H):
        th = x[:, 2]
        c, s = np.cos(th), np.sin(th)
        vx = body_vel[:, h, 0]
        vy = body_vel[:, h, 1]
        wz = body_vel[:, h, 2]
        x[:, 0] = x[:, 0] + (c * vx - s * vy) * dt
        x[:, 1] = x[:, 1] + (s * vx + c * vy) * dt
        x[:, 2] = x[:, 2] + wz * dt
        states[:, h, :] = x
    return states


def base_to_odom(circles_base, pose):
    """Transform base-frame circles [(x, y, r), ...] to odom using pose (x,y,th).
    Returns an (N, 3) array [ox, oy, r]. Radius is frame-invariant."""
    px, py, pth = pose
    if len(circles_base) == 0:
        return np.zeros((0, 3), dtype=float)
    arr = np.asarray(circles_base, dtype=float).reshape(-1, 3)
    c, s = np.cos(pth), np.sin(pth)
    ox = px + c * arr[:, 0] - s * arr[:, 1]
    oy = py + s * arr[:, 0] + c * arr[:, 1]
    return np.column_stack([ox, oy, arr[:, 2]])


def odom_to_base(points_odom, pose):
    """Inverse transform of points (N,2) from odom into the car's base frame."""
    px, py, pth = pose
    pts = np.asarray(points_odom, dtype=float).reshape(-1, 2)
    c, s = np.cos(pth), np.sin(pth)
    dx = pts[:, 0] - px
    dy = pts[:, 1] - py
    bx = c * dx + s * dy
    by = -s * dx + c * dy
    return np.column_stack([bx, by])
