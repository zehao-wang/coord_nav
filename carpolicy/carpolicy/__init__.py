"""The Policy interface -- the one contract every A->B policy implements. NOT
MPC-specific: MPC, a learned net, a script, or teleop are all just a Policy.
Subclass `Policy`, set `action_space`, override `plan`. The runner and GUI depend
only on this, so any backend is a drop-in (register it in mpc_baseline.registry).

plan(Observation) -> Action, once per control cycle (receding horizon).

INPUT  Observation(pose, goal, circles, field):
  pose    (x, y, yaw)    current pose, relative to the run's start (m, rad, x fwd/y left/CCW+)
  goal    (x, y)         goal B, also relative to start
  circles [(x,y,r), ...] this cycle's obstacles, base frame -- carclient.obstacles()
  field   ObstacleField  odom-frame obstacle memory + clearance queries (may be None)

OUTPUT  Action, in the policy's action_space:
  "velocity":  Action.velocity(v, w)          v fwd m/s, w yaw rad/s
  "discrete":  Action.discrete(id, mag, dur)   id 0..10, mag PWM, dur s

  MULTI-STEP (optional, like MPC): also pass `controls` = the whole planned
  horizon -- velocity [(v,w), ...] or discrete [(id,mag,dur), ...]. The scalar
  args are just its first step. The runner executes ONE step then re-plans by
  default; set LiveConfig.execute_steps > 1 to apply that many planned steps
  open-loop before re-planning. `traj` (H,3 predicted x,y,yaw) is viz-only.

  reset(): clear per-episode state (warm starts); called before each run.
"""

import abc
from collections import namedtuple

__all__ = ["Observation", "Action", "Policy"]

Observation = namedtuple("Observation", ["pose", "goal", "circles", "field"])


class Action(object):
    """Policy output. Use velocity()/discrete(); `space` says which fields apply.
    `controls` (optional) is the full planned horizon for multi-step execution;
    `traj` (optional) is a predicted path for visualisation only."""

    __slots__ = ("space", "v", "w", "action_id", "magnitude", "duration",
                 "traj", "controls")

    def __init__(self, space, v=0.0, w=0.0, action_id=0, magnitude=0.0,
                 duration=0.0, traj=None, controls=None):
        self.space = space
        self.v = v
        self.w = w
        self.action_id = action_id
        self.magnitude = magnitude
        self.duration = duration
        self.traj = traj
        self.controls = controls

    @classmethod
    def velocity(cls, v, w, traj=None, controls=None):
        """Body command: v forward (m/s), w yaw-rate (rad/s). controls: optional
        [(v,w), ...] planned horizon."""
        return cls("velocity", v=float(v), w=float(w), traj=traj, controls=controls)

    @classmethod
    def discrete(cls, action_id, magnitude, duration, traj=None, controls=None):
        """One mecanum hop: id 0..10, magnitude PWM, duration s. controls: optional
        [(id, mag, dur), ...] planned horizon."""
        return cls("discrete", action_id=int(action_id), magnitude=float(magnitude),
                   duration=float(duration), traj=traj, controls=controls)

    def __repr__(self):
        if self.space == "velocity":
            return "Action.velocity(v=%.3f, w=%.3f)" % (self.v, self.w)
        return "Action.discrete(id=%d, mag=%.0f, dur=%.2f)" % (
            self.action_id, self.magnitude, self.duration)


class Policy(abc.ABC):
    """Base for any A->B policy. Subclass, set `action_space`
    ("velocity"|"discrete"), override `plan`; override `reset` if it keeps
    per-episode state. Every policy is interchangeable through this interface."""

    action_space = None

    @abc.abstractmethod
    def plan(self, obs):
        """Observation -> Action. Called once per control cycle."""
        raise NotImplementedError

    def reset(self):
        """Clear per-episode state (e.g. warm starts). Optional override."""
        pass
