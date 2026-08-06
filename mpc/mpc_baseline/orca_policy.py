"""ORCA (Optimal Reciprocal Collision Avoidance) as a plug-in baseline policy.

The classic model-based crowd-navigation baseline (van den Berg et al., RVO2
library), and one of the standard comparison points in the crowd-nav
literature. It slots into this stack almost for free because its inputs are
exactly what our tracker produces: per-obstacle (position, velocity, radius) --
`obs.field.circles()` + `obs.field.velocities()`, i.e. the odom-frame memory
with coasting and the six-gate velocity trust.

Shape of the policy:
  * one fresh RVO2 sim per tick (stateless: the field already carries all
    cross-frame state). The robot is agent 0 with a preferred velocity aimed
    straight at B; every remembered circle is an agent whose current velocity
    AND preferred velocity are the tracker's gated estimate (static clutter and
    walls are simply zero-velocity agents).
  * one `doStep()` gives the collision-free holonomic velocity; we rotate it
    into the base frame and snap it to the nearest of the 8 mecanum translation
    actions (the same action table the actuator executes), or STOP when ORCA
    yields.

Honest caveats, so the benchmark rows read correctly: purely reactive (no
lookahead beyond the velocity horizon -- pockets and narrow static passages are
its known weakness), and it assumes RECIPROCITY (agents share avoidance
responsibility 50/50), which is optimistic against non-yielding pedestrians.

Requires the Python-RVO2 bindings (built from source; see CHANGELOG 0.9.23).
"""

import numpy as np

import rvo2

from carpolicy import Policy, Action
from .kinematics import build_action_table


class ORCAPolicy(Policy):
    action_space = "discrete"

    # RVO2 parameters, crowd-nav-conventional but scaled to our speeds
    NEIGHBOR_DIST = 4.0
    MAX_NEIGHBORS = 16
    TIME_HORIZON = 2.5          # s, agent-agent avoidance lookahead
    TIME_HORIZON_OBST = 2.5
    STOP_SPEED = 0.03           # below this ORCA is yielding: emit STOP

    def __init__(self, cfg):
        self.cfg = cfg          # a Variant2Config-shaped live config
        ids, table = build_action_table((1, 2, 3, 4, 5, 6, 7, 8),
                                        cfg.step_magnitude, cfg.robot)
        self.ids = ids
        norms = np.hypot(table[:, 0], table[:, 1])
        self._dirs = table[:, :2] / norms[:, None]          # unit body directions
        self._speed = float(norms.max())                     # executable speed
        self._margin = cfg.cost.extra_margin

    def plan(self, obs):
        pose = np.asarray(obs.pose, dtype=float)
        goal = np.asarray(obs.goal, dtype=float)[:2]
        tick = 1.0 / 3.0

        sim = rvo2.PyRVOSimulator(
            tick, self.NEIGHBOR_DIST, self.MAX_NEIGHBORS,
            self.TIME_HORIZON, self.TIME_HORIZON_OBST,
            self.cfg.robot.robot_radius + self._margin, self._speed)

        robot = sim.addAgent((float(pose[0]), float(pose[1])))
        to_goal = goal - pose[:2]
        d = float(np.hypot(to_goal[0], to_goal[1]))
        vpref = to_goal / d * min(self._speed, d / tick) if d > 1e-6 else (0.0, 0.0)
        sim.setAgentPrefVelocity(robot, (float(vpref[0]), float(vpref[1])))

        circles = obs.field.circles()
        vels = obs.field.velocities()
        for i in range(len(circles)):
            x, y, r = circles[i]
            vx, vy = (vels[i] if len(vels) else (0.0, 0.0))
            speed = float(np.hypot(vx, vy))
            a = sim.addAgent((float(x), float(y)), self.NEIGHBOR_DIST,
                             self.MAX_NEIGHBORS, self.TIME_HORIZON,
                             self.TIME_HORIZON_OBST,
                             float(r) + self._margin,
                             max(speed, 0.01), (float(vx), float(vy)))
            # movers keep their course; static clutter prefers to stay put
            sim.setAgentPrefVelocity(a, (float(vx), float(vy)))

        sim.doStep()
        vx, vy = sim.getAgentVelocity(robot)                 # odom frame

        # odom -> base, snap to the executable action set
        c, s = np.cos(-pose[2]), np.sin(-pose[2])
        bx, by = c * vx - s * vy, s * vx + c * vy
        sp = float(np.hypot(bx, by))
        dur = self.cfg.step_duration
        if sp < self.STOP_SPEED:
            aid, mag = 0, self.cfg.step_magnitude            # ORCA yields: STOP
            body = np.zeros(3)
        else:
            k = int(np.argmax(self._dirs @ (np.array([bx, by]) / sp)))
            aid = int(self.ids[k])
            # execute ORCA's chosen SPEED, not just its direction: near
            # obstacles it deliberately creeps (measured 0.15 m/s raw against
            # a fixed 0.41 m/s full-magnitude snap -- a 3x overspeed that
            # negated its caution and showed up as L4 collisions). The affine
            # plant maps speed back to magnitude; floored where wheels stall.
            frac = min(1.0, sp / self._speed)
            off = self.cfg.robot.pwm_offset
            mag = off + frac * (self.cfg.step_magnitude - off)
            mag = float(max(off + 4.0, min(self.cfg.step_magnitude, mag)))
            exec_speed = frac * self._speed
            body = np.array([self._dirs[k, 0] * exec_speed,
                             self._dirs[k, 1] * exec_speed, 0.0])
        # short straight extrapolation of the chosen hop, for the GUI overlay
        steps = np.arange(1, 5) * tick
        cb, sb = np.cos(pose[2]), np.sin(pose[2])
        ox = pose[0] + (cb * body[0] - sb * body[1]) * steps
        oy = pose[1] + (sb * body[0] + cb * body[1]) * steps
        traj = np.column_stack([ox, oy, np.full(len(steps), pose[2])])
        return Action.discrete(aid, mag, dur, traj=traj,
                               controls=[(aid, mag, dur)])

    def reset(self):
        pass
