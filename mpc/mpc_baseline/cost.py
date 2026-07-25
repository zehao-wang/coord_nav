"""Trajectory cost for the discrete variant 2 (pure numpy).

Given a batch of rollout states (K, H, 3) it returns one scalar cost per rollout,
combining progress to goal B with a soft obstacle barrier plus a hard collision
penalty. Both variants share this: variant 2 (discrete) uses total_cost_discrete,
the continuous (v,w) variant 1 uses total_cost_velocity (same goal + all-obstacle
barrier, plus cross-track and control smoothness) -- one cost, two action spaces.
"""

import numpy as np


def goal_cost(states, goal, cost_cfg, dt):
    """Running + terminal squared distance to goal B (goal = (x, y)).

    The running part is an INTEGRAL (x dt), not a raw sum: otherwise every running
    term scales with the number of steps, so changing the horizon silently
    re-weights running-vs-terminal and obstacle-vs-goal. That bit us for real --
    re-timing the planner from H=16/dt=0.6 to H=29/dt=1/3 at identical lookahead
    turned 0 collisions into 2 on the 1-seed suite purely through this (over 20
    seeds the re-timing itself was neutral). Weights are calibrated so the numbers
    match the old raw-sum tuning at dt=0.6."""
    pos = states[:, :, :2]                                   # (K, H, 2)
    gx, gy = goal[0], goal[1]
    d2 = (pos[:, :, 0] - gx) ** 2 + (pos[:, :, 1] - gy) ** 2  # (K, H)
    running = cost_cfg.w_goal_run * d2.sum(axis=1) * dt
    terminal = cost_cfg.w_goal_term * d2[:, -1]              # terminal: no dt
    return running + terminal


def obstacle_cost(states, field, robot_cfg, cost_cfg, dt):
    """Soft barrier inside the buffer plus a hard collision flag.

    Returns (cost (K,), collided (K,) bool). A rollout is 'collided' if any of its
    states falls inside the inflated obstacle (clearance < 0); those get the huge
    cost_cfg.collision_cost so the argmin never selects a colliding rollout.
    """
    K, H, _ = states.shape
    pts = states[:, :, :2].reshape(-1, 2)                    # (K*H, 2)
    clr = field.clearance(pts, robot_cfg.robot_radius,
                          cost_cfg.extra_margin).reshape(K, H)

    inside = clr < 0.0
    collided = inside.any(axis=1)

    # Soft barrier: quadratic in how far INTO the buffer a state is, and it keeps
    # growing once past inflation instead of saturating.
    #
    # This used to be clip(buf - clr, 0, buf), capped at buf. Inside inflation
    # (clr < 0) that made the term a CONSTANT -- measured, dC/d(clearance) was
    # exactly 0.00/m at every depth from -0.05 m to -0.30 m, against +8.33/m just
    # outside. So nothing pushed the car back out. Worse, `collided` below is one
    # boolean per rollout, so when EVERY sample is colliding the 1e6 is a common
    # offset and cancels in the argmin: the winner was then chosen on goal cost
    # alone, i.e. drive straight at the obstacle. Not clipping the top gives a
    # restoring gradient that grows with depth, so the least-bad rollout is the
    # shallowest one -- which is the behaviour you want when already inside.
    buf = cost_cfg.obs_buffer
    encroach = np.maximum(buf - clr, 0.0)                    # 0 outside the buffer
    soft = cost_cfg.w_obs * (encroach ** 2).sum(axis=1) * dt   # integral, see goal_cost

    cost = soft + collided * cost_cfg.collision_cost
    return cost, collided


def total_cost_discrete(states, goal, field, robot_cfg, cost_cfg, dt):
    """Full cost for the discrete variant. Returns (cost (K,), collided (K,)).

    No standing-still penalty: STOP keeps goal cost high far from B (never the
    argmin) and ~zero at B (correctly chosen), so a penalty would only fight settling.
    """
    g = goal_cost(states, goal, cost_cfg, dt)
    o, collided = obstacle_cost(states, field, robot_cfg, cost_cfg, dt)
    return g + o, collided


def crosstrack_cost(states, line, cost_cfg, dt):
    """Squared perpendicular deviation from the straight A->B line, summed over the
    horizon: pulls the car back onto the direct path after a detour (a tight
    go-around). line = (nx, ny, c); cross-track = nx*x + ny*y - c. 0 if no line."""
    if line is None or cost_cfg.w_track <= 0.0:
        return 0.0
    nx, ny, c = line
    ct = nx * states[:, :, 0] + ny * states[:, :, 1] - c     # (K, H)
    return cost_cfg.w_track * (ct * ct).sum(axis=1) * dt      # integral, see goal_cost


def control_cost(controls, cost_cfg, dt, u_prev=None):
    """Effort + smoothness for the continuous variant: penalise speed, yaw-rate,
    and the STEP-TO-STEP change in each (so the sampled (v,w) plan is gradual, not
    jerky). controls = (K, H, 2) of [v, w].

    `u_prev` is the control the car is CURRENTLY executing (the one this policy
    returned last tick). Without it, each tick's argmin is free to jump anywhere:
    w_smooth only couples steps WITHIN one horizon, nothing couples consecutive
    ticks. Measured on the car, that let the yaw command reverse sign tick to tick
    (+0.68 / +0.50 / -0.40 / -0.53) faster than the chassis can follow -- the
    steady-state yaw tracked ~100 % while a real run delivered only 0.608 of the
    commanded yaw. Penalising the first step's distance from what is already
    running is the standard fix and costs one term."""
    v, w = controls[:, :, 0], controls[:, :, 1]
    eff = (cost_cfg.w_ctrl_v * (v * v).sum(axis=1)
           + cost_cfg.w_ctrl_w * (w * w).sum(axis=1)) * dt
    # smoothness is a RATE: (du/dt)^2 dt = du^2/dt, so it does not change meaning
    # when the step time changes.
    dv, dw = np.diff(v, axis=1), np.diff(w, axis=1)
    smooth = cost_cfg.w_smooth * ((dv * dv).sum(axis=1) + (dw * dw).sum(axis=1)) / dt
    if u_prev is not None and cost_cfg.w_cont > 0.0:
        # same RATE form as w_smooth, so it means the same thing at any tick rate
        d0v = controls[:, 0, 0] - u_prev[0]
        d0w = controls[:, 0, 1] - u_prev[1]
        smooth = smooth + cost_cfg.w_cont * (d0v * d0v + d0w * d0w) / dt
    return eff + smooth


def total_cost_velocity(states, controls, goal, line, field, robot_cfg, cost_cfg,
                        dt, u_prev=None):
    """Full cost for the CONTINUOUS (v, w) sampling variant 1. Same goal + all-
    obstacle barrier as the discrete variant, plus cross-track (return to the A->B
    line) and control effort/smoothness. Returns (cost (K,), collided (K,))."""
    g = goal_cost(states, goal, cost_cfg, dt)
    o, collided = obstacle_cost(states, field, robot_cfg, cost_cfg, dt)
    return (g + o + crosstrack_cost(states, line, cost_cfg, dt)
            + control_cost(controls, cost_cfg, dt, u_prev)), collided
