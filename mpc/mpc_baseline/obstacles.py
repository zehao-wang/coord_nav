"""Obstacle bookkeeping for the planner (pure numpy, no ROS).

The car publishes /obstacles as base-frame circles that only reflect the CURRENT
scan (no map). During a go-around the obstacle can slip out of the lidar FOV, so
this holds a short rolling memory in the ODOM frame: each frame's circles are
transformed to odom, merged with recent ones, and old/far ones are pruned. Set
ObstacleConfig.mem_time_s = 0 to fall back to current-frame-only.

The field also estimates each tracked circle's VELOCITY from the frame sequence
(constant-velocity tracking: per-track finite difference + EMA, with a sighting
gate, a deadband and a speed cap so noise on a static box never invents motion).
Estimation always runs -- it is pure bookkeeping -- but it only changes planning
when a policy asks for `predict()` / `clearance_pred()` (the *_t "time-aware"
registry variants do; the plain variants keep the frozen-world queries). This is
the standard constant-velocity-prediction baseline for MPC with dynamic
obstacles, and it lives HERE so the live runner and the sim share it verbatim.

All planner queries are vectorised: pass an (M, 2) array of query points and get
per-point clearance or collision flags against every remembered circle.
"""

import numpy as np

from .kinematics import base_to_odom


class ObstacleField(object):
    def __init__(self, cfg, wall_clock):
        """cfg: ObstacleConfig. wall_clock: a zero-arg callable returning seconds
        (time.monotonic live, or the sim clock offline) -- injected so the field
        never calls a real clock itself and stays deterministic in sim."""
        self.cfg = cfg
        self._clock = wall_clock
        # remembered circles in odom:
        # rows [x, y, r, last_seen_t, vx, vy, n_sightings, obs_x, obs_y]
        # (x, y) is the EMA-smoothed position planning queries use; (obs_x,
        # obs_y) is the last RAW observation -- velocity must difference raw
        # observations, because the EMA position lags a steady mover by v*T and
        # differencing against it overestimates the speed by exactly 2x.
        self._mem = np.zeros((0, 9), dtype=float)

    @staticmethod
    def _rows(fresh, now):
        fresh = np.asarray(fresh, dtype=float).reshape(-1, 3)
        return np.column_stack([fresh, np.full(len(fresh), now),
                                np.zeros((len(fresh), 2)),
                                np.ones(len(fresh)),
                                fresh[:, :2]])

    def _vel(self):
        """(N, 2) usable velocity per track: zero until a track has been sighted
        vel_min_sightings times and its EMA speed clears vel_deadband -- so a
        noisy static box reads exactly 0 and time-aware planning on a static
        world degenerates to the frozen-world planner."""
        if not len(self._mem):
            return np.zeros((0, 2))
        v = self._mem[:, 4:6].copy()
        sp = np.hypot(v[:, 0], v[:, 1])
        gate = ((self._mem[:, 6] >= self.cfg.vel_min_sightings) &
                (sp >= self.cfg.vel_deadband))
        v[~gate] = 0.0
        return v

    def update(self, circles_base, pose):
        """Fold one /obstacles frame (base-frame [(x,y,r),...]) into the memory
        using the car pose (x, y, theta) in odom. Returns the merged odom
        circles as (N, 3)."""
        now = self._clock()
        fresh = base_to_odom(circles_base, pose)          # (N, 3)

        if self.cfg.mem_time_s <= 0.0:
            # current-frame-only: no cross-frame association, so no velocities
            self._mem = self._rows(fresh, now) if len(fresh) else \
                np.zeros((0, 9))
            return fresh

        # prune stale and far-away memory relative to the car's odom position
        if len(self._mem):
            age_ok = (now - self._mem[:, 3]) <= self.cfg.mem_time_s
            d = np.hypot(self._mem[:, 0] - pose[0], self._mem[:, 1] - pose[1])
            near_ok = d <= self.cfg.mem_radius
            self._mem = self._mem[age_ok & near_ok]

        # merge each fresh circle: refresh a nearby remembered one, else append.
        # Association is against the COASTED position (last seen + v * gap) so a
        # tracked mover that was hidden for a frame or two is re-acquired as the
        # same track instead of spawning a fresh one with no velocity.
        for row in fresh:
            if len(self._mem):
                gap = now - self._mem[:, 3]
                # Coast from the RAW last observation with the UNGATED (but
                # capped, EMA'd) velocity. Both choices are about the
                # acquisition cliff: matching against the lagging EMA position
                # with the GATED (still zero) velocity meant a mover above
                # 0.30 m/s out-ran merge_dist before its gate ever opened,
                # fragmenting into 1-sighting tracks with v=0 forever -- the
                # 0.40 m/s cross_fast archetype was silently untrackable. A
                # young track's raw velocity may be noisy, but it only biases
                # ASSOCIATION; planning still sees the gated value.
                px = self._mem[:, 7] + self._mem[:, 4] * gap
                py = self._mem[:, 8] + self._mem[:, 5] * gap
                d = np.hypot(px - row[0], py - row[1])
                j = int(np.argmin(d))
                if d[j] <= self.cfg.merge_dist:
                    dt = now - self._mem[j, 3]
                    if dt > 1e-6:
                        # raw velocity = delta of RAW observations (see __init__:
                        # differencing the EMA position doubles a steady mover's
                        # speed), then EMA'd and capped so one bad association
                        # cannot invent a fast phantom mover.
                        rvx = (row[0] - self._mem[j, 7]) / dt
                        rvy = (row[1] - self._mem[j, 8]) / dt
                        sp = np.hypot(rvx, rvy)
                        if sp > self.cfg.vel_cap:
                            rvx *= self.cfg.vel_cap / sp
                            rvy *= self.cfg.vel_cap / sp
                        a = self.cfg.vel_ema
                        self._mem[j, 4] = (1 - a) * self._mem[j, 4] + a * rvx
                        self._mem[j, 5] = (1 - a) * self._mem[j, 5] + a * rvy
                    # EMA on position so a moving obstacle isn't frozen at its
                    # first-seen spot; keep larger radius (conservative), refresh ts.
                    self._mem[j, 0] = 0.5 * self._mem[j, 0] + 0.5 * row[0]
                    self._mem[j, 1] = 0.5 * self._mem[j, 1] + 0.5 * row[1]
                    self._mem[j, 2] = max(self._mem[j, 2], row[2])
                    self._mem[j, 3] = now
                    self._mem[j, 6] += 1
                    self._mem[j, 7] = row[0]
                    self._mem[j, 8] = row[1]
                    continue
            self._mem = np.vstack([self._mem, self._rows(row[None, :], now)])
        return self.circles()

    def circles(self):
        """Remembered obstacles as (N, 3) [x, y, r] in odom."""
        return self._mem[:, :3].copy() if len(self._mem) else np.zeros((0, 3))

    def velocities(self):
        """(N, 2) estimated odom velocity per remembered circle (gated: exactly
        0 for tracks that are too young or below the deadband)."""
        return self._vel()

    # --- constant-velocity prediction (the *_t time-aware variants) --------
    def predict(self, dts):
        """(T, N, 3) circles extrapolated to `dts` seconds from NOW, constant
        velocity. Each track coasts from where it was LAST SEEN, so a mover
        hidden behind another obstacle keeps moving in the prediction instead of
        freezing at its last sighting -- and the memory-trail problem dissolves:
        the trail rows carry the same velocity, so they extrapolate ONTO the
        mover's path instead of pinning a soft wall where it used to be. Total
        extrapolation (age since last seen + dt) is capped at pred_cap_s: beyond
        that a constant-velocity guess is fiction, so the circle holds there.
        Static tracks (gated v == 0) predict exactly their current position."""
        dts = np.atleast_1d(np.asarray(dts, dtype=float))
        if not len(self._mem):
            return np.zeros((len(dts), 0, 3))
        now = self._clock()
        vel = self._vel()                                    # (N, 2) gated
        age = now - self._mem[:, 3]                          # (N,)
        t = np.minimum(age[None, :] + dts[:, None], self.cfg.pred_cap_s)  # (T, N)
        base = self._mem[:, :3].copy()
        # Moving tracks extrapolate from the RAW last observation: the 50/50
        # position EMA lags a steady mover by exactly v*tick, which put every
        # prediction one tick behind the world (0.07 m at 0.22 m/s, in the
        # cut-in-front direction). Static tracks (gated v == 0) keep the EMA
        # position, so a static world predicts exactly circles() and the
        # *_t == plain byte-identical guarantee survives.
        moving = (vel[:, 0] != 0.0) | (vel[:, 1] != 0.0)
        base[moving, 0] = self._mem[moving, 7]
        base[moving, 1] = self._mem[moving, 8]
        out = np.repeat(base[None, :, :], len(dts), axis=0)
        out[:, :, 0] += vel[None, :, 0] * t
        out[:, :, 1] += vel[None, :, 1] * t
        return out

    def clearance_pred(self, states_xy, dts, robot_radius, extra_margin):
        """Time-indexed clearance for a rollout batch: states_xy (K, H, 2) with
        step h at dts[h] seconds from now -> (K, H) signed clearance of each
        state to the nearest inflated circle AT THAT TIME. +inf if no obstacles.
        This is the frozen-world `clearance()` with the circles moved to where
        the tracker says they will be when the car gets there."""
        states_xy = np.asarray(states_xy, dtype=float)
        K, H = states_xy.shape[0], states_xy.shape[1]
        pred = self.predict(dts)                             # (H, N, 3)
        if pred.shape[1] == 0:
            return np.full((K, H), np.inf)
        dx = states_xy[:, :, 0, None] - pred[None, :, :, 0]  # (K, H, N)
        dy = states_xy[:, :, 1, None] - pred[None, :, :, 1]
        radii = pred[:, :, 2] + robot_radius + extra_margin  # (H, N)
        dist = np.sqrt(dx * dx + dy * dy) - radii[None, :, :]
        return dist.min(axis=2)

    # --- vectorised planner queries --------------------------------------
    def _inflated(self, robot_radius, extra_margin):
        """(centres (N,2), inflated_radii (N,)) for collision/clearance tests."""
        c = self.circles()
        if len(c) == 0:
            return np.zeros((0, 2)), np.zeros((0,))
        return c[:, :2], c[:, 2] + robot_radius + extra_margin

    def clearance(self, points, robot_radius, extra_margin):
        """Signed clearance (m) of each query point (M,2) to the nearest inflated
        circle edge: positive = outside, negative = inside. +inf if no obstacles.
        Returns (M,)."""
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        centres, radii = self._inflated(robot_radius, extra_margin)
        if len(centres) == 0:
            return np.full(len(pts), np.inf)
        # (M, N) distances point->centre minus inflated radius
        dx = pts[:, 0][:, None] - centres[None, :, 0]
        dy = pts[:, 1][:, None] - centres[None, :, 1]
        dist = np.sqrt(dx * dx + dy * dy) - radii[None, :]
        return dist.min(axis=1)

    def raw_min_distance(self, point):
        """Distance (m) from a single point to the nearest RAW circle edge
        (no robot inflation). Feeds the tick log's `dmem` column -- how close the
        car is to the obstacles the POLICY remembers. +inf if no obstacles."""
        c = self.circles()
        if len(c) == 0:
            return np.inf
        d = np.hypot(c[:, 0] - point[0], c[:, 1] - point[1]) - c[:, 2]
        return float(d.min())
