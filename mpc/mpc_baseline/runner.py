"""Live MPC runner: drive the real car A->B around obstacles (ROS side).

Ties the pure-numpy planner to the car via carclient. Each cycle it reads the
latest /obstacles frame, folds it into the field, plans, and actuates -- variant 1
as one /drive_wheels body-velocity PULSE per cycle (car-side keep-alive), variant 2
as one discrete /drive_action hop.

Pose / progress to B uses **/odom** by default (pose_source="odom"): steady 10 Hz
with no gaps, real closed-loop feedback. Its yaw comes from the IMU gyro; confirm
ACCURACY on the ground first with smoke/calib_gyro.py (a propped-up car tells you
nothing about odom accuracy). pose_source="dead_reckon" integrates the executed
commands with our kinematics model instead -- immune to odom dropout, at the cost
of drifting if the model speed differs from reality.

Safety (this car has NO motor timeout in firmware -- a lost stop latches the
wheels): an abnormal exit / Ctrl-C / link loss always hard-estops; an imminent-
collision guard estops (or soft-stops, see collision_estop) before the footprint
touches an obstacle; the runner refuses to start unless the MCU link reads healthy
(so the link-loss estop is armed). These are safety interlocks, not a planning
fallback.
"""

import time

import numpy as np

from .obstacles import ObstacleField
from carpolicy import Observation
from .policies import make_policy
from .actuators import VelocityPulseActuator, DriveActionActuator
from .kinematics import action_body_velocity, rollout_body


def _nearest_base_edge(circles):
    """Distance (m) from the car centre to the nearest obstacle edge in the base
    frame, or +inf if none. circles = [(x, y, r), ...]."""
    if not circles:
        return np.inf
    return min(float(np.hypot(x, y) - r) for (x, y, r) in circles)


class PolicyRunner(object):
    """Drives ANY Policy (policy.py) A->B: reads obstacles, builds an Observation,
    plans, actuates the Action, re-plans. Not MPC-specific -- the first argument is
    either a ready Policy or a variant key (which builds the MPC policy)."""

    def __init__(self, policy, cfg, live_cfg, obs_cfg, client, log=print,
                 pose_source="odom", on_step=None, collision_estop=True):
        self.cfg = cfg
        self.live = live_cfg
        self.obs_cfg = obs_cfg
        self.client = client
        self.log = log
        self.pose_source = pose_source        # "odom" (default) | "lidar" | "dead_reckon"
        self.on_step = on_step                # optional cb(dict) each cycle (GUI)
        self.collision_estop = collision_estop  # False -> soft stop instead of SSH estop

        # accept a pre-built Policy object, or a variant key for the MPC policy
        if hasattr(policy, "plan"):
            self.policy = policy
            self.label = type(policy).__name__
        else:
            self.policy = make_policy(policy, cfg)
            self.label = "variant-%s" % policy
        self.action_space = getattr(self.policy, "action_space", "discrete")

        self.field = ObstacleField(obs_cfg, time.monotonic)
        tick = live_cfg.tick
        if self.action_space == "velocity":
            # The car keep-alives each pulse for tick.action_duration (>1 tick), so a
            # single dropped command holds the last velocity instead of stuttering,
            # and two in a row let it expire and brake.
            self.act = VelocityPulseActuator(client, cfg.robot,
                                             pulse_duration=tick.action_duration)
        else:
            self.act = DriveActionActuator(client)
            # A hop shorter than a tick expires before the next tick supersedes it
            # -> the car brakes in the gap and the motion stutters.
            hop = getattr(cfg, "step_duration", None)
            if hop is not None and hop < tick.period:
                raise ValueError(
                    "step_duration=%.2fs is shorter than one tick (%.2fs at %.1f Hz): "
                    "each hop would expire before the next tick replaces it and the "
                    "car would brake between hops. Raise step_duration to >= %.2f."
                    % (hop, tick.period, tick.rate_hz, tick.period))
        self._link_seen = False
        self._abort = False
        self._dr = np.zeros(3)                # dead-reckoned pose [x, y, yaw]
        self._t = {}                          # per-tick timing/continuity, for the tick log
        self._lodom = None
        if self.pose_source == "lidar":       # lidar ICP odometry (accurate forward)
            from .lidar_odom import LidarOdometry
            self._lodom = LidarOdometry()

    def abort(self):
        """Request a clean stop from another thread (e.g. a GUI Stop button)."""
        self._abort = True

    # -- pose (odom by default) -------------------------------------------
    def _read_pose(self):
        """Return (pose (3,), age_s). Dead-reckon is always fresh (age 0)."""
        if self.pose_source == "odom":
            p = self.client.pose()
            if p is None:
                return None, None
            return np.array([p.x, p.y, p.yaw]), p.age
        if self.pose_source == "lidar":
            p = self._lodom.pose()
            if p is None:
                return None, None
            return np.array([p.x, p.y, p.yaw]), p.age
        return self._dr.copy(), 0.0

    def _advance_dr(self, body_vel, dt):
        self._dr = rollout_body(self._dr, np.asarray(body_vel, float).reshape(1, 1, 3),
                                dt)[0, 0]

    # -- safety helpers ---------------------------------------------------
    def _link_bad(self):
        ok = self.client.link_ok()
        if ok:
            self._link_seen = True
        return self.live.estop_on_link_loss and self._link_seen and not ok

    def _imminent_collision(self, circles):
        # _nearest_base_edge is centre-to-obstacle-surface; the footprint edge is
        # at robot_radius, so fire collision_margin BEFORE the rim touches.
        return (self.live.collision_abort and
                _nearest_base_edge(circles) <
                self.cfg.robot.robot_radius + self.live.collision_margin)

    def _wait_link(self, timeout):
        """Arm the link-loss interlock: poll until the MCU link reads healthy so
        _link_seen is set (a later drop is then detectable). Returns True if seen."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and not self.client.is_shutdown():
            if self.client.link_ok():
                self._link_seen = True
                return True
            time.sleep(0.1)
        return False

    # -- main loop --------------------------------------------------------
    def run(self, max_time=None):
        """Drive to B. Blocks until reached / aborted / timed out / estopped.
        Returns a summary dict. Call abort() from another thread to stop early."""
        self._abort = False
        if self.client.wait_obstacles(timeout=5.0) is None:
            raise RuntimeError("no /obstacles -- is perception running? (roscar)")

        # Goal B: dead-reckon starts at the origin heading 0 (B straight ahead);
        # odom mode anchors at the current odom pose.
        if self.pose_source == "odom":
            if self.client.wait_pose(timeout=5.0) is None:
                raise RuntimeError("no /odom for pose_source='odom'")
            p0 = self.client.pose()
            start = np.array([p0.x, p0.y, p0.yaw])
        elif self.pose_source == "lidar":
            if not self._lodom.wait(timeout=6.0):
                raise RuntimeError("lidar odometry not ready (no /scan?)")
            start = np.zeros(3)
        else:
            self._dr = np.zeros(3)
            start = np.zeros(3)
        # Goal B in the start body frame: goal_dist forward (x), goal_y left (y).
        gx, gy = self.cfg.goal.goal_dist, self.cfg.goal.goal_y
        c, s = np.cos(start[2]), np.sin(start[2])
        goal = np.array([start[0] + gx * c - gy * s, start[1] + gx * s + gy * c])
        self.log("A->B: policy=%s pose=%s B=(fwd %.2f, left %.2f) mag=%.0f" % (
            self.label, self.pose_source, gx, gy, self.live.magnitude))

        # Arm the link-loss interlock before any motion.
        if self.live.estop_on_link_loss and not self._wait_link(self.live.link_wait_s):
            raise RuntimeError(
                "MCU link not healthy (no fresh /battery_v > %.0fV in %.0fs) -- "
                "refusing to drive" % (self.client.MIN_LINK_VOLT, self.live.link_wait_s))

        t_end = self.cfg.goal.timeout_s if max_time is None else max_time
        t0 = time.monotonic()
        reason, reached = "timeout", False
        if self.action_space == "velocity":
            self.act.start()

        try:
            tick = self.live.tick
            period = tick.period
            n_exec = max(1, int(getattr(self.live, "execute_steps", 1)))
            plan, idx = None, 0        # cached (controls, act) + step index into it
            pending = None             # (ctl, act) decided last tick, dispatched this tick
            last_fid = None
            n_tick = 0
            t_tick_prev = None
            while not self.client.is_shutdown():
                # ---- TICK BOUNDARY: one observation frame = one tick ----------
                t_wait0 = time.monotonic()
                frame = self.client.wait_frame(after=last_fid,
                                               timeout=tick.wait_ticks * period)
                t_tick = time.monotonic()
                if frame is None:      # no new perception frame in time
                    self.log("no new observation frame -- holding")
                    self._hold()
                    plan, pending, idx = None, None, 0
                    continue
                n_tick += 1
                # Timing breakdown of the tick, so a log can show where the period
                # went and whether the loop overran (planning + grace > period, which
                # makes the NEXT wait_frame return instantly on an already-queued
                # frame -- fine occasionally, a policy that is too slow if sustained).
                self._t = {
                    "tick": n_tick,
                    "wait_s": t_tick - t_wait0,
                    "period_s": None if t_tick_prev is None else t_tick - t_tick_prev,
                    "skipped": (0 if last_fid is None else
                                max(0, frame.frame_id - last_fid - 1)),
                }
                t_tick_prev = t_tick
                last_fid = frame.frame_id
                obs = frame

                if self._abort:
                    reason = "aborted"
                    break
                if time.monotonic() - t0 > t_end:
                    reason = "timeout"
                    break
                if self._link_bad():
                    self.client.estop()
                    return self._summary("link_lost_estop", False, goal, t0)

                pose, page = self._read_pose()
                if (pose is None or obs.age > self.obs_cfg.max_age_stale or
                        (page is not None and page > self.obs_cfg.max_age_stale)):
                    self.log("stale obstacles/pose -- holding")
                    self._hold()
                    plan, pending, idx = None, None, 0   # re-plan fresh once data returns
                    continue

                # Safety runs on the FRESH frame, before anything decided a tick ago
                # is allowed to reach the wheels.
                if self._imminent_collision(obs.circles):
                    if self.collision_estop:
                        self.client.estop()
                        return self._summary("collision_estop", False, goal, t0)
                    self._hold()
                    return self._summary("collision_soft_stop", False, goal, t0)

                gd = float(np.hypot(goal[0] - pose[0], goal[1] - pose[1]))
                if gd <= self.cfg.goal.goal_tol:
                    reached, reason = True, "reached"
                    break

                # ---- DISPATCH what last tick decided (overrides the car's current
                # command). Nothing pending -> send nothing: the car keeps running
                # its last command, and holds once that command expires.
                if pending is not None:
                    ctl, act = pending
                    pending = None
                    if act.space == "velocity":
                        v, w = ctl
                        self.act.set_velocity(v, 0.0, w)
                        self._advance_dr((v, 0.0, w), period)
                        self._emit_step(pose, goal, gd, obs, v=v, w=w, traj=act.traj)
                    else:
                        aid, mag, dur = ctl
                        self.act.step(aid, mag, dur)          # non-blocking
                        self._advance_dr(action_body_velocity(aid, mag, self.cfg.robot),
                                         period)
                        self._emit_step(pose, goal, gd, obs, action=aid, traj=act.traj)

                # ---- PLAN this tick's observation -> the action for the NEXT tick.
                t_plan0 = time.monotonic()
                replanned = plan is None or idx >= min(n_exec, len(plan[0]))
                if replanned:
                    self.field.update(obs.circles, pose)
                    act = self.policy.plan(Observation(pose, goal, obs.circles, self.field))
                    plan, idx = (self._controls_of(act), act), 0
                controls, act = plan
                pending = (controls[idx], act)
                idx += 1
                self._t["plan_s"] = time.monotonic() - t_plan0
                self._t["replanned"] = replanned
                self._t["work_s"] = time.monotonic() - t_tick
            return self._summary(reason, reached, goal, t0)
        except BaseException as exc:                       # incl. KeyboardInterrupt
            self.log("ABORT (%s) -- estop" % type(exc).__name__)
            try:
                self.client.estop()
            finally:
                pass
            raise
        finally:
            self._shutdown_actuator()

    def _controls_of(self, act):
        """Per-step controls for the plan: act.controls (the full horizon) if the
        policy gave one, else the single Action as a 1-step plan."""
        if act.space == "velocity":
            if act.controls is not None:
                return [(float(v), float(w)) for v, w in act.controls]
            return [(act.v, act.w)]
        if act.controls is not None:
            return [(int(a), float(m), float(d)) for a, m, d in act.controls]
        return [(act.action_id, act.magnitude, act.duration)]

    def _emit_step(self, pose, goal, gd, obs, action=None, v=None, w=None, traj=None):
        if self.action_space == "velocity":
            self.log("gd=%.2f v=%.3f w=%+.3f obs=%d" % (gd, v, w, len(obs.circles)))
        else:
            self.log("gd=%.2f act=%d mag=%.0f obs=%d" % (
                gd, action, self.live.magnitude, len(obs.circles)))
        if self.on_step is not None:
            self.on_step({"pose": pose.tolist(), "goal": goal.tolist(), "gd": gd,
                          "action": action, "v": v, "w": w,
                          "n_obs": len(obs.circles), "policy": self.label,
                          "traj": traj.tolist() if traj is not None else None})

    def _hold(self):
        """Stop the car NOW, whatever it is running.

        Both action spaces need this: since hops are dispatched non-blocking, a
        discrete hop can still be running when a tick decides to hold, so "discrete
        policies are between hops" (the old assumption, true only while the runner
        slept through each hop) no longer holds."""
        if self.action_space == "velocity":
            self.act.set_velocity(0.0, 0.0, 0.0)
        else:
            self.act.stop()

    def _shutdown_actuator(self):
        try:
            self.act.close()
        except Exception:
            pass

    def _summary(self, reason, reached, goal, t0):
        pose, _ = self._read_pose()
        fgd = (round(float(np.hypot(goal[0] - pose[0], goal[1] - pose[1])), 3)
               if pose is not None else None)
        s = {"policy": self.label, "reached": reached, "reason": reason,
             "time_s": round(time.monotonic() - t0, 2), "final_goal_dist": fgd}
        self.log("DONE %s" % s)
        return s


# Back-compat alias: the runner drives any Policy, not only MPC.
MPCRunner = PolicyRunner
