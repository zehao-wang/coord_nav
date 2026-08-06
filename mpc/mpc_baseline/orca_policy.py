"""ORCA (Optimal Reciprocal Collision Avoidance) as a plug-in baseline policy.

The classic model-based crowd-navigation baseline (van den Berg et al., RVO2
library), and one of the standard comparison points in the crowd-nav
literature. It slots into this stack almost for free because its inputs are
exactly what our tracker produces: per-obstacle (position, velocity, radius) --
`obs.field.circles()` + `obs.field.velocities()`, i.e. the odom-frame memory
with coasting and the six-gate velocity trust.

Environment modelling -- the two RVO2 channels, used for what they are for:
  * MOVING obstacles (gated velocity != 0) are agents: the reciprocal VO
    half-plane machinery, current velocity AND preferred velocity set to the
    tracker's estimate.
  * STATIC obstacles (gated velocity == 0) go through RVO2's native
    line-obstacle channel as CCW octagons circumscribing the circle. That
    channel is the right physics for things that never yield: responsibility
    1.0 (no reciprocal halving), constraints that keep wall-parallel
    velocities feasible (head-on approach resolves to sliding along, not
    freezing), HARD constraints in the infeasible fallback (never relaxed
    toward a wall), and no competition with movers for the MAX_NEIGHBORS
    agent slots. Feeding statics as zero-velocity agents (the crowd-nav-sim
    convention, and what this file used to do) loses all four properties --
    measurable as freezing and thin margins on the static benchmark tiers.
    The margin is counted ONCE for statics (on the robot agent's radius);
    mover agents keep the historical r + margin.

The preferred velocity gets a small seeded perturbation -- the symmetry-
breaking device from RVO2's own Blocks.cpp example ("Perturb a little to avoid
deadlocks due to perfect symmetry"): a dead-centred goal behind a wall
otherwise projects to a zero tangential velocity and the slide never starts.
Unlike Blocks.cpp it is sampled ONCE per episode, not per tick: a fresh
zero-mean draw every tick is a random walk (the symmetric stall re-centres
itself; measured as dead_ahead timeouts) and puts sign-flipping yaw jitter
into the vw variant. A sticky draw is a consistent side bias that actually
accumulates into a slide, and stays deterministic under the episode seed.

Two action-space variants, mirroring the MPC pair (mpc_grid / mpc_vw):
  * ORCAPolicy   ("discrete"): snap to the nearest of the 8 mecanum
    translation hops; dispatched magnitude follows ORCA's chosen SPEED
    through the chosen DIRECTION's own affine plant (near obstacles ORCA
    deliberately creeps -- executing full magnitude negated its caution,
    measured as L4 collisions; and diagonals/strafes carry their own
    dispatch multiplier, so the inversion is per-direction).
  * ORCAVWPolicy ("velocity"): track the holonomic velocity with the same
    forward-only unicycle contract the vw MPC drives -- heading-proportional
    yaw clipped to what the car can DELIVER at that v (inner wheel keeps
    rolling), v = ORCA speed x cos(heading error), with a roll floor that
    preserves yaw authority while the target is off the nose but still
    ahead. When ORCA's velocity points BEHIND the nose (cos <= 0) the
    policy yields outright: any forward roll would leave ORCA's certified
    half-planes, and the deliverable-yaw model has no pivot-in-place.

Honest caveats, so the benchmark rows read correctly: purely reactive (no
lookahead beyond the velocity horizon) with the goal entering only through the
preferred velocity -- it slides along walls but does not PLAN around concave
arrangements, and it assumes RECIPROCITY from movers (optimistic against
non-yielding pedestrians). The vw variant additionally projects a holonomic
certificate onto nonholonomic execution: between solves the car's velocity is
only an approximation of ORCA's, so the pairwise no-collision guarantee does
not strictly transfer (the 3 Hz replan bounds the divergence per tick).

Requires the Python-RVO2 bindings (built from source; see CHANGELOG 0.9.23).
"""

import numpy as np

import rvo2

from carpolicy import Policy, Action
from .kinematics import build_action_table

# unit CCW octagon (RVO2 obstacle polygons: CCW = solid, verified empirically);
# scaled by 1/cos(pi/8) the polygon CIRCUMSCRIBES the circle it stands in for.
_OCT = np.array([[np.cos(a), np.sin(a)]
                 for a in (np.arange(8) + 0.5) * (np.pi / 4.0)])
_OCT_SCALE = 1.0 / np.cos(np.pi / 8.0)


class _ORCABase(Policy):
    """Shared RVO2 core: one fresh sim per tick (stateless -- the field already
    carries all cross-frame state), returning the collision-free holonomic
    velocity in the odom frame. Subclasses map it into their action space."""

    # RVO2 parameters, crowd-nav-conventional but scaled to our speeds
    NEIGHBOR_DIST = 4.0
    MAX_NEIGHBORS = 16
    TIME_HORIZON = 2.5          # s, agent-agent avoidance lookahead
    TIME_HORIZON_OBST = 2.5     # s, agent-obstacle lookahead
    STOP_SPEED = 0.02           # below this ORCA is yielding: emit STOP
    PERTURB_ANGLE = 0.12        # rad; Blocks.cpp-style deadlock breaking
    PERTURB_MIN_FRAC = 0.75     # |angle| >= this fraction of PERTURB_ANGLE: the
                                # tangential slide it seeds (speed x sin) must
                                # clear every yield threshold below, or a
                                # head-on stall reads as a yield forever (a
                                # near-zero sticky draw did exactly that:
                                # wall_inline 4/5 -> 0/5)
    PERTURB_FRAC = 0.02         # fractional speed jitter, same purpose

    def __init__(self, cfg, speed):
        self.cfg = cfg
        self._speed = float(speed)                   # executable speed cap
        self._margin = cfg.cost.extra_margin
        self.rng = np.random.default_rng(0)          # reseeded by eval._seed_policy
        self._pang = None                            # sticky per-episode perturbation

    def _tick(self):
        """The executed control period: resolve_policy/runner write it into
        rollout_dt (discrete cfg) / mppi.dt (vw cfg) AFTER build, so read it
        per plan instead of freezing 1/3 at construction."""
        dt = getattr(self.cfg, "rollout_dt", None)
        if dt is None:
            dt = getattr(getattr(self.cfg, "mppi", None), "dt", None)
        return float(dt) if dt else 1.0 / 3.0

    def _orca_velocity(self, obs):
        """-> (pose, odom-frame velocity) from one RVO2 solve."""
        pose = np.asarray(obs.pose, dtype=float)
        goal = np.asarray(obs.goal, dtype=float)[:2]
        tick = self._tick()
        if self._pang is None:                       # first plan of the episode
            mag = self.PERTURB_ANGLE * (
                self.PERTURB_MIN_FRAC
                + (1.0 - self.PERTURB_MIN_FRAC) * float(self.rng.uniform()))
            self._pang = mag if self.rng.uniform() < 0.5 else -mag
            self._pmag = 1.0 - self.PERTURB_FRAC * float(self.rng.uniform())

        sim = rvo2.PyRVOSimulator(
            tick, self.NEIGHBOR_DIST, self.MAX_NEIGHBORS,
            self.TIME_HORIZON, self.TIME_HORIZON_OBST,
            self.cfg.robot.robot_radius + self._margin, self._speed)

        robot = sim.addAgent((float(pose[0]), float(pose[1])))
        to_goal = goal - pose[:2]
        d = float(np.hypot(to_goal[0], to_goal[1]))
        if d > 1e-6:
            c, s = np.cos(self._pang), np.sin(self._pang)
            u = to_goal / d
            u = np.array([c * u[0] - s * u[1], s * u[0] + c * u[1]])
            spd = min(self._speed, d / tick) * self._pmag
            vpref = (float(u[0] * spd), float(u[1] * spd))
        else:
            vpref = (0.0, 0.0)
        sim.setAgentPrefVelocity(robot, vpref)

        circles = obs.field.circles()
        vels = obs.field.velocities()
        statics = False
        for i in range(len(circles)):
            x, y, r = circles[i]
            vx, vy = (vels[i] if len(vels) else (0.0, 0.0))
            if vx == 0.0 and vy == 0.0:
                pts = np.array([x, y]) + _OCT * (float(r) * _OCT_SCALE)
                sim.addObstacle([(float(px), float(py)) for px, py in pts])
                statics = True
            else:
                speed = float(np.hypot(vx, vy))
                a = sim.addAgent((float(x), float(y)), self.NEIGHBOR_DIST,
                                 self.MAX_NEIGHBORS, self.TIME_HORIZON,
                                 self.TIME_HORIZON_OBST,
                                 float(r) + self._margin,
                                 speed + 0.01, (float(vx), float(vy)))
                # movers keep their course (reciprocity is ORCA's assumption)
                sim.setAgentPrefVelocity(a, (float(vx), float(vy)))
        if statics:
            sim.processObstacles()

        sim.doStep()
        return pose, np.asarray(sim.getAgentVelocity(robot), dtype=float)

    def _to_body(self, pose, v):
        c, s = np.cos(-pose[2]), np.sin(-pose[2])
        return c * v[0] - s * v[1], s * v[0] + c * v[1]

    def reset(self):
        self._pang = None                            # fresh draw next episode


class ORCAPolicy(_ORCABase):
    """Discrete variant: ORCA velocity -> nearest of the 8 mecanum hops."""

    action_space = "discrete"

    def __init__(self, cfg):
        from .kinematics import action_body_velocity, action_effective_magnitude
        ids, table = build_action_table((1, 2, 3, 4, 5, 6, 7, 8),
                                        cfg.step_magnitude, cfg.robot)
        norms = np.hypot(table[:, 0], table[:, 1])
        super().__init__(cfg, float(norms.max()))
        self.ids = ids
        self._dirs = table[:, :2] / norms[:, None]          # unit body directions
        # Per-direction affine plant for speed -> magnitude inversion. Diagonals
        # and strafes carry their own dispatch multiplier, so each direction has
        # its own stall floor and slope; inverting through the straight-hop
        # plant overspeeds 4 of the 8 directions exactly in the creep regime.
        off = cfg.robot.pwm_offset
        mult = np.array([action_effective_magnitude(a, 1.0, cfg.robot)
                         for a in ids])
        self._m_lo = (off + 4.0) / mult                     # commanded floor
        lo = np.array([action_body_velocity(a, m, cfg.robot)
                       for a, m in zip(ids, self._m_lo)])
        self._n_lo = np.hypot(lo[:, 0], lo[:, 1])           # speed at the floor
        self._slope = (norms - self._n_lo) / np.maximum(
            cfg.step_magnitude - self._m_lo, 1e-9)          # m/s per magnitude

    def plan(self, obs):
        pose, v = self._orca_velocity(obs)
        bx, by = self._to_body(pose, v)
        sp = float(np.hypot(bx, by))
        dur = self.cfg.step_duration
        if sp < self.STOP_SPEED:
            aid, mag = 0, self.cfg.step_magnitude            # ORCA yields: STOP
            body = np.zeros(3)
        else:
            k = int(np.argmax(self._dirs @ (np.array([bx, by]) / sp)))
            aid = int(self.ids[k])
            if sp < 0.5 * self._n_lo[k]:
                # the direction's stall floor would execute > 2x ORCA's
                # request -- the same overspeed-negates-caution mechanism as
                # the full-magnitude snap, just relocated to the creep regime
                # (measured as L4 collisions when the floor executed 0.02-0.03
                # requests at 0.055). Better to yield a tick and re-solve.
                aid, mag = 0, self.cfg.step_magnitude
                body = np.zeros(3)
            else:
                # execute ORCA's chosen SPEED, not just its direction: near
                # obstacles it deliberately creeps (measured 0.15 m/s raw
                # against a fixed 0.41 m/s full-magnitude snap -- a 3x
                # overspeed that negated its caution and showed up as L4
                # collisions). Inverted through the chosen direction's OWN
                # affine plant (diag/strafe dispatch multipliers included),
                # floored where its wheels stall.
                mag = self._m_lo[k] + (sp - self._n_lo[k]) / self._slope[k]
                mag = float(np.clip(mag, self._m_lo[k], self.cfg.step_magnitude))
                exec_speed = self._n_lo[k] + self._slope[k] * (mag - self._m_lo[k])
                body = np.array([self._dirs[k, 0] * exec_speed,
                                 self._dirs[k, 1] * exec_speed, 0.0])
        # short straight extrapolation of the chosen hop, for the GUI overlay
        tick = self._tick()
        steps = np.arange(1, 5) * tick
        cb, sb = np.cos(pose[2]), np.sin(pose[2])
        ox = pose[0] + (cb * body[0] - sb * body[1]) * steps
        oy = pose[1] + (sb * body[0] + cb * body[1]) * steps
        traj = np.column_stack([ox, oy, np.full(len(steps), pose[2])])
        return Action.discrete(aid, mag, dur, traj=traj,
                               controls=[(aid, mag, dur)])


class ORCAVWPolicy(_ORCABase):
    """(v, w) variant: track the holonomic ORCA velocity with the forward-only
    unicycle contract the vw MPC drives (v >= 0, yaw authority scales with v)."""

    action_space = "velocity"

    KP_YAW = 1.8                # heading-error P gain (rad/s per rad)
    TURN_SPEED = 0.08           # m/s roll floor while the nose is far off target

    def __init__(self, cfg):
        super().__init__(cfg, float(cfg.v_max))

    def plan(self, obs):
        pose, v = self._orca_velocity(obs)
        bx, by = self._to_body(pose, v)
        sp = float(np.hypot(bx, by))
        if sp < self.STOP_SPEED:
            return Action.velocity(0.0, 0.0)
        err = float(np.arctan2(by, bx))
        if np.cos(err) <= 0.0:
            # ORCA's velocity points BEHIND the nose: any forward roll leaves
            # its certified half-planes (measured as head-on-mover collisions
            # when a roll floor was applied here), and the deliverable-yaw
            # model has no pivot-in-place. Yield outright and re-solve next
            # tick -- the reactive analogue of the discrete variant's STOP.
            return Action.velocity(0.0, 0.0)
        # forward speed follows the heading alignment; keep rolling gently when
        # the target direction is off the nose but still ahead, else yaw
        # authority is zero (the deliverable-yaw limit scales with v)
        vcmd = sp * np.cos(err)
        if abs(err) > 0.35:
            vcmd = max(vcmd, min(self.TURN_SPEED, self._speed))
        vcmd = float(np.clip(vcmd, self.cfg.v_min, self.cfg.v_max))
        # yaw the car can DELIVER at vcmd -- same limit Variant1 samples under
        # (inner wheel keeps rolling forward, scrub deadband, w_max cap)
        robot = self.cfg.robot
        arm = max(self.cfg.steer_arm, 1e-6)
        raw = (1.0 - self.cfg.min_inner_frac) * vcmd / arm
        achievable = getattr(robot, "yaw_gain", 1.0) * max(
            0.0, raw - getattr(robot, "yaw_deadband", 0.0))
        wlim = max(0.0, min(self.cfg.w_max, achievable))
        w = float(np.clip(self.KP_YAW * err, -wlim, wlim))
        # short unicycle extrapolation for the GUI overlay
        tick = self._tick()
        traj = np.empty((4, 3))
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        for h in range(4):
            x += vcmd * np.cos(yaw) * tick
            y += vcmd * np.sin(yaw) * tick
            yaw += w * tick
            traj[h] = (x, y, yaw)
        return Action.velocity(vcmd, w, traj=traj, controls=[(vcmd, w)])
