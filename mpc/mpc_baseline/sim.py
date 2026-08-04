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
                 noise_xy=0.0, dropout=0.0, seed=0, disturbance=None):
        self.world = world
        # DisturbanceConfig or None. With it, step() executes commands the way the
        # REAL car does (fit from 298 on-car ticks): multiplicative speed noise and
        # a first-order yaw lag (tau 0.48 s ~= 1.4 ticks). Without it the sim is a
        # perfect-execution world, which mis-ranks controllers -- w_cont=0.6 looked
        # free undisturbed and went 0/2 on the car, because sluggishness only costs
        # when there is something to correct.
        self.dist = disturbance
        self._w_exec = 0.0                  # yaw-lag filter state
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
        """Advance the true pose by one body-velocity command held for dt,
        through the execution-disturbance model if one is configured."""
        vb = np.asarray(body_vel, dtype=float).copy()
        if self.dist is not None:
            d = self.dist
            # multiplicative speed noise (translation axes together: the fit is on
            # the arc speed, and vx/vy come from the same four wheels)
            vb[:2] *= self.rng.normal(d.v_gain, d.v_std)
            # first-order yaw lag: a from the time constant, b scaled to keep the
            # fitted steady-state gain b0/(1-a0) at any dt
            a0 = float(np.exp(-(1.0 / 3.0) / d.w_tau))
            ss = d.w_b / (1.0 - a0)
            a = float(np.exp(-dt / d.w_tau))
            b = ss * (1.0 - a)
            self._w_exec = a * self._w_exec + b * vb[2] + self.rng.normal(0.0, d.w_noise)
            vb[2] = self._w_exec
        self.pose = rollout_body(self.pose, vb.reshape(1, 1, 3), dt)[0, 0]
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
                max_steps=400, robot_cfg=None, buffered=False):
    """Drive any Policy closed-loop in `sim` toward goal B.

    velocity policies: apply (v, w) as a body velocity held for plan_dt each cycle.
    discrete policies: execute the chosen action for its own duration (plan_dt is
      ignored) -- which needs a RobotConfig to turn (action, magnitude) into a body
      velocity. Pass it as `robot_cfg`; it falls back to `policy.cfg.robot` for the
      built-in variants. The Policy interface does NOT require a `.cfg`, so a
      custom discrete policy should be given `robot_cfg` explicitly.

    buffered=True reproduces the LIVE runner's loop shape: the action planned at
    tick N executes at tick N+1 (superseding), and the policy plans from the pose
    advanced by the command in flight (the runner's dead-time compensation). The
    default unbuffered loop mattered once already: w_cont=0.6 looked FREE through
    it while the buffered loop shows a 15 % timeout cost even UNDISTURBED --
    combine with KinematicSim(disturbance=...) for the live-faithful evaluation
    that smoothness/robustness tuning must be screened against.

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

    def _body_of(act):
        """One planned Action -> (body velocity, step time)."""
        if act.space == "velocity":
            return np.array([act.v, 0.0, act.w]), plan_dt
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
        body = np.asarray(action_body_velocity(act.action_id, act.magnitude, rc))
        # buffered = the tick model: one tick of the hop executes, then the next
        # tick's hop supersedes it (runner rollout_dt semantics)
        return body, (plan_dt if buffered else act.duration)

    pending = None                       # buffered: body decided last tick
    for _ in range(max_steps):
        circles = sim.sense()
        field.update(circles, sim.pose)

        plan_pose = sim.pose.copy()
        if buffered and pending is not None:
            # dead-time compensation, as the live runner does: plan from where the
            # car will be once the in-flight command has run its tick
            plan_pose = rollout_body(plan_pose, pending[0].reshape(1, 1, 3),
                                     pending[1])[0, 0]
        act = policy.plan(Observation(plan_pose, goal, circles, field))

        if buffered:
            body, dt = pending if pending is not None else (np.zeros(3), plan_dt)
            pending = _body_of(act)
        else:
            body, dt = _body_of(act)
        effort += float(np.hypot(body[0], body[1]) + abs(body[2])) * dt

        prev = sim.pose.copy()
        sim.step(body, dt)
        path_len += float(np.hypot(sim.pose[0] - prev[0], sim.pose[1] - prev[1]))
        traj.append(sim.pose.copy())

        clr = sim.true_min_clearance()
        min_clr = min(min_clr, clr)
        # Contact = the obstacle edge reaches the FOOTPRINT, the same test the live
        # guard aborts on (runner._imminent_collision, margin 0). The old `clr < 0`
        # only fired once the footprint CENTRE was inside the obstacle, so an
        # episode could end "success" with the surface 12 cm into the car: v1's
        # published tight-suite collision rates were 0.008/0.167 under that metric
        # and are 0.275/0.500 under this one (v2 stays 0.000 under both).
        if clr < sim.robot_radius:
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


def realistic_scenarios(goal_dist=3.0):
    """The regime the car is actually driven in: B ~3 m ahead, obstacles 1.2-1.5 m
    out (the default_scenarios suite is deliberately TIGHTER than the car's
    physical turn radius supports -- see mpc/README.md). This suite is what the
    README's realistic-regime row runs on; it is committed because that row was
    first published from ad-hoc worlds that never made it into the repo, so the
    number could not be reproduced. benchmark.py --suite realistic runs it.
    """
    return [
        World([(1.35, 0.00, 0.14)], (0, 0, 0), "box_ahead"),
        World([(1.20, 0.15, 0.14)], (0, 0, 0), "box_offset"),
        World([(1.30, -0.10, 0.14), (1.30, 0.18, 0.14)], (0, 0, 0), "wall_on_line"),
        World([(1.20, 0.20, 0.14), (1.50, -0.25, 0.14)], (0, 0, 0), "slalom_far"),
    ]
