"""Offline kinematic simulator + scenarios for developing/validating the MPC.

No ROS, no car. A World holds ground-truth obstacle circles and a start pose; the
KinematicSim integrates the same mecanum kinematics the planner assumes and fakes
an /obstacles feed (world circles within lidar range, transformed to the base
frame, matching the real message: a list of (x, y, r) tuples). run_episode drives
either policy closed-loop to the goal and returns metrics. Because the planner,
cost and kinematics are all pure numpy, this exercises the exact code that runs on
the car -- only the actuation/transport differs.
"""

from collections import namedtuple

import numpy as np

from carpolicy import Observation
from .kinematics import rollout_body, action_body_velocity
from .obstacles import ObstacleField


World = namedtuple("World", ["circles", "start", "name"])
# circles: list of (x, y, r) ground-truth obstacles in the world (== odom) frame.
# start:   (x, y, theta) car start pose. name: label.

EpisodeResult = namedtuple("EpisodeResult", [
    "name", "variant", "reached", "collided", "sim_time", "steps",
    "path_length", "min_clearance", "final_goal_dist", "control_effort", "traj"])


class KinematicSim(object):
    def __init__(self, world, sense_range=3.0, robot_radius=0.13,
                 noise_xy=0.0, dropout=0.0, seed=0):
        self.world = world
        self.circles = np.asarray(world.circles, dtype=float).reshape(-1, 3) \
            if len(world.circles) else np.zeros((0, 3))
        self.sense_range = sense_range
        self.robot_radius = robot_radius
        self.noise_xy = noise_xy            # gaussian std added to sensed centres
        self.dropout = dropout              # prob a visible obstacle is missed
        self.rng = np.random.default_rng(seed)
        self.pose = np.asarray(world.start, dtype=float).copy()
        self.t = 0.0

    def clock(self):
        return self.t

    def step(self, body_vel, dt):
        """Advance the true pose by one body-velocity command held for dt."""
        vb = np.asarray(body_vel, dtype=float).reshape(1, 1, 3)
        self.pose = rollout_body(self.pose, vb, dt)[0, 0]
        self.t += dt
        return self.pose

    def sense(self):
        """Fake /obstacles: world circles within sense_range, in the base frame,
        as a list of (x, y, r) -- the same shape carclient.obstacles().circles is."""
        out = []
        px, py, pth = self.pose
        c, s = np.cos(-pth), np.sin(-pth)   # world->base rotation (by -theta)
        for (wx, wy, r) in self.circles:
            dx, dy = wx - px, wy - py
            if np.hypot(dx, dy) - r > self.sense_range:
                continue
            if self.dropout and self.rng.random() < self.dropout:
                continue
            bx = c * dx - s * dy
            by = s * dx + c * dy
            if self.noise_xy:
                bx += self.rng.normal(0.0, self.noise_xy)
                by += self.rng.normal(0.0, self.noise_xy)
            out.append((float(bx), float(by), float(r)))
        return out

    def true_min_clearance(self):
        """Distance (m) from the car centre to the nearest RAW obstacle edge in
        the world -- negative means the footprint centre is inside an obstacle."""
        if len(self.circles) == 0:
            return np.inf
        d = np.hypot(self.circles[:, 0] - self.pose[0],
                     self.circles[:, 1] - self.pose[1]) - self.circles[:, 2]
        return float(d.min())


def goal_from_start(start, goal_dist, goal_y=0.0):
    """Goal B in the start body frame: goal_dist forward, goal_y left."""
    x, y, th = start
    c, s = np.cos(th), np.sin(th)
    return np.array([x + goal_dist * c - goal_y * s, y + goal_dist * s + goal_y * c])


def run_episode(sim, policy, variant, obs_cfg, goal_cfg, plan_dt=0.25,
                max_steps=400, robot_cfg=None):
    """Drive any Policy closed-loop in `sim` toward goal B.

    velocity policies: apply (v, w) as a body velocity held for plan_dt each cycle.
    discrete policies: execute the chosen action for its own duration (plan_dt is
      ignored) -- which needs a RobotConfig to turn (action, magnitude) into a body
      velocity. Pass it as `robot_cfg`; it falls back to `policy.cfg.robot` for the
      built-in variants. The Policy interface does NOT require a `.cfg`, so a
      custom discrete policy should be given `robot_cfg` explicitly.
    Returns an EpisodeResult with trajectory and metrics.
    """
    field = ObstacleField(obs_cfg, sim.clock)
    goal = goal_from_start(sim.world.start, goal_cfg.goal_dist, goal_cfg.goal_y)

    traj = [sim.pose.copy()]
    min_clr = sim.true_min_clearance()
    path_len = 0.0
    effort = 0.0
    collided = False
    reached = False

    for _ in range(max_steps):
        circles = sim.sense()
        field.update(circles, sim.pose)
        act = policy.plan(Observation(sim.pose.copy(), goal, circles, field))

        if act.space == "velocity":
            body = np.array([act.v, 0.0, act.w])
            dt = plan_dt
            effort += (abs(act.v) + abs(act.w)) * dt
        else:
            rc = robot_cfg
            if rc is None:
                rc = getattr(getattr(policy, "cfg", None), "robot", None)
            if rc is None:
                raise TypeError(
                    "discrete policy %s needs a RobotConfig to convert (action, "
                    "magnitude) into a body velocity. Pass run_episode(..., "
                    "robot_cfg=cfg.robot) -- the Policy interface does not require a "
                    "`.cfg` attribute, only this offline sim path needs the chassis "
                    "model." % type(policy).__name__)
            body = np.asarray(action_body_velocity(
                act.action_id, act.magnitude, rc))
            dt = act.duration
            effort += float(np.hypot(body[0], body[1]) + abs(body[2])) * dt

        prev = sim.pose.copy()
        sim.step(body, dt)
        path_len += float(np.hypot(sim.pose[0] - prev[0], sim.pose[1] - prev[1]))
        traj.append(sim.pose.copy())

        clr = sim.true_min_clearance()
        min_clr = min(min_clr, clr)
        if clr < 0.0:
            collided = True
            break
        if np.hypot(goal[0] - sim.pose[0], goal[1] - sim.pose[1]) <= goal_cfg.goal_tol:
            reached = True
            break
        if sim.t >= goal_cfg.timeout_s:
            break

    final_gd = float(np.hypot(goal[0] - sim.pose[0], goal[1] - sim.pose[1]))
    return EpisodeResult(sim.world.name, variant, reached, collided, sim.t,
                         len(traj) - 1, path_len, min_clr, final_gd, effort,
                         np.array(traj))


# --- built-in scenarios --------------------------------------------------
def default_scenarios(goal_dist=1.0):
    """A small suite for goal B = (goal_dist, 0). Every goal sits in free space:
    inflation = robot_radius (0.13) + CostConfig.extra_margin (0.10) added to each
    obstacle radius, so a 0.12 m obstacle inflates to ~0.35 m -- goals are kept
    clear of that. Each scenario blocks the straight path a different way.
    """
    return [
        World([], (0, 0, 0), "clear"),
        World([(0.50, 0.00, 0.12)], (0, 0, 0), "dead_ahead"),
        World([(0.55, 0.08, 0.14)], (0, 0, 0), "slightly_offset"),
        World([(0.40, 0.00, 0.11), (0.62, 0.00, 0.11)], (0, 0, 0), "wall_inline"),
        World([(0.55, 0.42, 0.12), (0.55, -0.42, 0.12)], (0, 0, 0), "gap_thread"),
        World([(0.40, 0.10, 0.12), (0.68, -0.24, 0.12)], (0, 0, 0), "slalom"),
    ]
