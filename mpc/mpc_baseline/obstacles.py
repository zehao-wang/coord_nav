"""Obstacle bookkeeping for the planner (pure numpy + scipy Hungarian, no ROS).

The car publishes /obstacles as base-frame circles that only reflect the CURRENT
scan (no map). This module holds a short rolling memory of those circles in the
ODOM frame and estimates each one's VELOCITY over time -- the sensing layer both
the time-aware (*_t) MPC variants and the ORCA baseline plan against.

Since 0.9.24 the tracker is the standard DATMO/SORT recipe instead of the
hand-rolled EMA + gate battery it replaced:

  * per-track constant-velocity KALMAN FILTER (state [px py vx vy], white-noise
    acceleration model): smooth positions, principled velocity + covariance. A
    steady mover is followed WITHOUT lag (the old 50/50 position EMA lagged by
    v*tick and needed a raw-observation workaround for predictions).
  * HUNGARIAN one-shot association on Mahalanobis distance (chi-square gated,
    plus an absolute distance gate): globally optimal matching replaces greedy
    nearest-neighbour -- the "track hijack" and same-frame double-merge classes
    die structurally, and a young track's wide velocity covariance opens its
    association gate automatically (the old assoc_young_boost hack, now
    principled).
  * BIRTH / DEATH / RE-IDENTIFICATION management: unmatched detections start
    tentative tracks; unseen tracks coast and expire after mem_time_s; freshly
    dead tracks linger in a graveyard for reid_time_s and a new detection near
    a dead track's coasted position RESURRECTS it with its velocity and history
    (the user-reported "loses the circle" failure).

What survives from the gate battery -- the parts that encode SENSOR pathologies
a Kalman filter cannot know about (all measured on recorded car data):
isolation (walls arrive as CHAINS of clusters; split objects become adjacent
tracks), the trust-range bands (far lidar centroids jitter; 2026-08-05
no-human capture), and the speed deadband. Coherence / net-displacement /
sighting counting are replaced by the chi-square SIGNIFICANCE of the KF
velocity against its own covariance.

Static-world guarantee, unchanged: with noiseless repeated observations the KF
innovation is zero and the velocity stays EXACTLY 0.0, so time-aware planning
degenerates byte-identically to the frozen-world planner (regression-tested).

All planner queries are vectorised and unchanged: clearance(), predict(),
clearance_pred(), raw_min_distance().
"""

import numpy as np
from scipy.optimize import linear_sum_assignment

from .kinematics import base_to_odom

_H = np.array([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0]])
_I4 = np.eye(4)


class _Track(object):
    # t_state: the time the KF state refers to (advanced by every predict);
    # last_seen: the last MEASUREMENT time (drives expiry and prediction age).
    # Keeping them separate matters: a coasting track is predicted forward each
    # frame, and computing the next predict's dt from last_seen would
    # double-integrate the same interval.
    __slots__ = ("x", "P", "r", "t_state", "last_seen", "born", "hits",
                 "misses", "moving", "low_streak")

    def __init__(self, z, r, now, meas_var, init_vel_std):
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.diag([meas_var, meas_var,
                          init_vel_std ** 2, init_vel_std ** 2])
        self.r = float(r)
        self.t_state = now
        self.last_seen = now
        self.born = now
        self.hits = 1
        self.misses = 0
        self.moving = False     # hysteresis state of the significance gate
        self.low_streak = 0     # consecutive ticks below the ENTRY threshold

    def predict_to(self, now, accel_var):
        dt = now - self.t_state
        self.t_state = max(self.t_state, now)
        if dt <= 0.0:
            return
        F = np.array([[1.0, 0.0, dt, 0.0],
                      [0.0, 1.0, 0.0, dt],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]])
        d2, d3, d4 = dt * dt, dt ** 3, dt ** 4
        Q = accel_var * np.array([[d4 / 4, 0.0, d3 / 2, 0.0],
                                  [0.0, d4 / 4, 0.0, d3 / 2],
                                  [d3 / 2, 0.0, d2, 0.0],
                                  [0.0, d3 / 2, 0.0, d2]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def innovation(self, z, meas_var):
        y = np.asarray(z, dtype=float) - _H @ self.x
        S = _H @ self.P @ _H.T + meas_var * np.eye(2)
        return y, S

    def update(self, z, r, now, meas_var, r_alpha=0.5):
        y, S = self.innovation(z, meas_var)
        K = self.P @ _H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (_I4 - K @ _H) @ self.P
        self.r = (1.0 - r_alpha) * self.r + r_alpha * float(r)
        self.last_seen = now
        self.hits += 1
        self.misses = 0


class ObstacleField(object):
    def __init__(self, cfg, wall_clock):
        """cfg: ObstacleConfig. wall_clock: a zero-arg callable returning seconds
        (time.monotonic live, or the sim clock offline) -- injected so the field
        never calls a real clock itself and stays deterministic in sim."""
        self.cfg = cfg
        self._clock = wall_clock
        self._tracks = []
        self._grave = []                  # (track, death_time) for re-ID
        self._car_xy = None
        self._cache()

    # ------------------------------------------------------------------ core
    def update(self, circles_base, pose):
        """Fold one /obstacles frame (base-frame [(x,y,r),...]) into the memory
        using the car pose (x, y, theta) in odom. Returns the merged odom
        circles as (N, 3)."""
        now = self._clock()
        fresh = base_to_odom(circles_base, pose)          # (N, 3)
        self._car_xy = (float(pose[0]), float(pose[1]))
        cfg = self.cfg

        if cfg.mem_time_s <= 0.0:
            # current-frame-only mode: no cross-frame association, no velocity
            self._tracks = [
                _Track(row[:2], row[2], now, cfg.kf_meas_std ** 2,
                       cfg.kf_init_vel_std) for row in fresh]
            self._grave = []
            self._cache()
            return self.circles()

        # 1. retire stale/far tracks into the graveyard, prune the graveyard
        keep = []
        for tr in self._tracks:
            stale = (now - tr.last_seen) > cfg.mem_time_s
            far = np.hypot(tr.x[0] - pose[0], tr.x[1] - pose[1]) > cfg.mem_radius
            if stale or far:
                self._grave.append((tr, now))
            else:
                keep.append(tr)
        self._tracks = keep
        self._grave = [(tr, td) for (tr, td) in self._grave
                       if now - td <= cfg.reid_time_s]

        # 2. Kalman-predict every live track to `now`
        av = cfg.kf_accel_std ** 2
        mv = cfg.kf_meas_std ** 2
        for tr in self._tracks:
            tr.predict_to(now, av)

        # 3. Hungarian association: Mahalanobis cost, chi-square + absolute gate
        T, D = len(self._tracks), len(fresh)
        matched_t, matched_d = set(), set()
        if T and D:
            cost = np.full((T, D), 1e6)
            for i, tr in enumerate(self._tracks):
                for j in range(D):
                    y, S = tr.innovation(fresh[j, :2], mv)
                    d_abs = float(np.hypot(y[0], y[1]))
                    if d_abs > cfg.assoc_abs_gate:
                        continue
                    m2 = float(y @ np.linalg.solve(S, y))
                    if m2 <= cfg.assoc_chi2:
                        cost[i, j] = m2
            ri, ci = linear_sum_assignment(cost)
            for i, j in zip(ri, ci):
                if cost[i, j] < 1e6:
                    self._tracks[i].update(fresh[j, :2], fresh[j, 2], now, mv)
                    matched_t.add(i)
                    matched_d.add(j)

        # 4. unmatched detections: try graveyard re-ID, else a new track
        for j in range(D):
            if j in matched_d:
                continue
            z = fresh[j, :2]
            best, bd = None, cfg.reid_dist
            for k, (tr, td) in enumerate(self._grave):
                gap = now - tr.t_state
                px = tr.x[0] + tr.x[2] * gap
                py = tr.x[1] + tr.x[3] * gap
                d = float(np.hypot(px - z[0], py - z[1]))
                if d < bd:
                    best, bd = k, d
            if best is not None:
                tr, _ = self._grave.pop(best)
                tr.predict_to(now, av)               # coast through the gap
                tr.update(z, fresh[j, 2], now, mv)
                self._tracks.append(tr)
            else:
                self._tracks.append(_Track(z, fresh[j, 2], now,
                                           mv, cfg.kf_init_vel_std))

        # 5. unmatched tracks just coast (their predict already ran)
        for i, tr in enumerate(self._tracks):
            if T and i < T and i not in matched_t:
                tr.misses += 1

        self._cache()
        return self.circles()

    def _cache(self):
        n = len(self._tracks)
        self._pos = np.array([t.x[:2] for t in self._tracks]) if n else np.zeros((0, 2))
        self._rad = np.array([t.r for t in self._tracks]) if n else np.zeros(0)
        self._raw_vel = np.array([t.x[2:] for t in self._tracks]) if n else np.zeros((0, 2))
        self._last_seen = np.array([t.last_seen for t in self._tracks]) if n else np.zeros(0)

    # --------------------------------------------------------------- outputs
    def circles(self):
        """Remembered obstacles as (N, 3) [x, y, r] in odom (KF-smoothed)."""
        if not len(self._tracks):
            return np.zeros((0, 3))
        return np.column_stack([self._pos, self._rad])

    def _vel(self):
        """(N, 2) usable velocity per track, gated so REAL perception on a
        static world reads exactly 0. The statistical gates of the old design
        (coherence / net displacement / sighting count) are replaced by the
        chi-square SIGNIFICANCE of the KF velocity against its own covariance;
        the SENSOR gates survive: isolation (wall chains, split objects),
        speed deadband, and the near/far trust bands (far centroids jitter --
        far tracks need more history and higher significance)."""
        n = len(self._tracks)
        if not n:
            return np.zeros((0, 2))
        cfg = self.cfg
        v = self._raw_vel.copy()
        sp = np.hypot(v[:, 0], v[:, 1])
        over = sp > cfg.vel_cap
        if over.any():
            v[over] *= (cfg.vel_cap / sp[over])[:, None]
            sp = np.minimum(sp, cfg.vel_cap)

        sig = np.zeros(n)
        hits = np.zeros(n)
        for i, tr in enumerate(self._tracks):
            Pv = tr.P[2:, 2:]
            try:
                sig[i] = float(tr.x[2:] @ np.linalg.solve(Pv, tr.x[2:]))
            except np.linalg.LinAlgError:
                sig[i] = 0.0
            hits[i] = tr.hits

        if n > 1:
            dx = self._pos[:, 0][:, None] - self._pos[:, 0][None, :]
            dy = self._pos[:, 1][:, None] - self._pos[:, 1][None, :]
            dd = np.hypot(dx, dy)
            np.fill_diagonal(dd, np.inf)
            iso = dd.min(axis=1)
        else:
            iso = np.full(n, np.inf)

        # significance with HYSTERESIS (Schmitt trigger): ENTRY needs the
        # full chi-square threshold, but a track already declared moving stays
        # trusted down to vel_sig_exit -- the 28% real dropout rate inflates
        # P_v through the coasting predicts and made a plain threshold flap
        # (measured 40-59% retention; phantoms still must pass the high bar
        # once before they can enjoy the low one).
        entry = sig >= cfg.vel_sig_chi2
        stay = sig >= cfg.vel_sig_exit
        for i, tr in enumerate(self._tracks):
            tr.low_streak = 0 if entry[i] else tr.low_streak + 1
            # a moving track may coast below ENTRY for at most sig_low_ticks
            # in a row (bridges real dropout gaps) before it must re-qualify
            tr.moving = (bool(stay[i]) and tr.low_streak <= cfg.sig_low_ticks
                         and (tr.moving or bool(entry[i])))
        momentum = np.array([tr.moving for tr in self._tracks], dtype=bool)
        gate = ((sp >= cfg.vel_deadband) &
                (hits >= cfg.vel_min_hits) &
                momentum &
                (iso >= cfg.vel_isolation))
        if self._car_xy is not None:
            dcar = np.hypot(self._pos[:, 0] - self._car_xy[0],
                            self._pos[:, 1] - self._car_xy[1])
            near = dcar <= cfg.vel_trust_range
            far = (~near) & (dcar <= cfg.vel_far_range)
            far_entry = sig >= cfg.far_sig_mult * cfg.vel_sig_chi2
            gate &= (near |
                     (far & (hits >= cfg.vel_min_hits + cfg.far_hits_extra) &
                      (far_entry | momentum)))
        v[~gate] = 0.0
        return v

    def velocities(self):
        """(N, 2) estimated odom velocity per remembered circle (gated: exactly
        0 for any track the gates in _vel do not trust)."""
        return self._vel()

    # --- constant-velocity prediction (the *_t time-aware variants) --------
    def predict(self, dts):
        """(T, N, 3) circles extrapolated to `dts` seconds from NOW, constant
        velocity. Each track coasts from its KF position (which, unlike the old
        EMA, does not lag a steady mover), so a mover hidden behind another
        obstacle keeps moving in the prediction instead of freezing at its last
        sighting. Total extrapolation (age since last seen + dt) is capped at
        pred_cap_s: beyond that a constant-velocity guess is fiction. Static
        tracks (gated v == 0) predict exactly their current position."""
        dts = np.atleast_1d(np.asarray(dts, dtype=float))
        n = len(self._tracks)
        if not n:
            return np.zeros((len(dts), 0, 3))
        now = self._clock()
        vel = self._vel()                                    # (N, 2) gated
        age = now - self._last_seen                          # (N,)
        t = np.minimum(age[None, :] + dts[:, None], self.cfg.pred_cap_s)
        out = np.repeat(np.column_stack([self._pos, self._rad])[None, :, :],
                        len(dts), axis=0)
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
        if pred.shape[1]:
            # cull circles far from the rollout envelope (exact, see _cull) --
            # a track is kept if ANY of its predicted positions is near the box
            pts = states_xy.reshape(-1, 2)
            lo = pts.min(axis=0) - self._CULL_SLACK
            hi = pts.max(axis=0) + self._CULL_SLACK
            r_all = pred[:, :, 2] + robot_radius + extra_margin        # (H, N)
            near = ((pred[:, :, 0] + r_all >= lo[0]) & (pred[:, :, 0] - r_all <= hi[0]) &
                    (pred[:, :, 1] + r_all >= lo[1]) & (pred[:, :, 1] - r_all <= hi[1]))
            keep = near.any(axis=0)                                    # (N,)
            pred = pred[:, keep, :]
        if pred.shape[1] == 0:
            return np.full((K, H), np.inf)
        dx = states_xy[:, :, 0, None] - pred[None, :, :, 0]  # (K, H, N)
        dy = states_xy[:, :, 1, None] - pred[None, :, :, 1]
        radii = pred[:, :, 2] + robot_radius + extra_margin  # (H, N)
        dist = np.sqrt(dx * dx + dy * dy) - radii[None, :, :]
        return dist.min(axis=2)

    # --- vectorised planner queries --------------------------------------
    # Circles farther than this from the query bounding box cannot contribute a
    # clearance below it (soft cost is zero beyond inflation + obs_buffer), so
    # culling them is EXACT for every cost/collision decision downstream --
    # only the (unused) magnitude of large clearances changes. 0.8 m covers
    # inflation (~0.45) + obs_buffer (0.15) + headroom.
    _CULL_SLACK = 0.8

    def _cull(self, pts, centres, radii):
        lo = pts.min(axis=0) - self._CULL_SLACK
        hi = pts.max(axis=0) + self._CULL_SLACK
        keep = ((centres[:, 0] + radii >= lo[0]) & (centres[:, 0] - radii <= hi[0]) &
                (centres[:, 1] + radii >= lo[1]) & (centres[:, 1] - radii <= hi[1]))
        return centres[keep], radii[keep]

    def _inflated(self, robot_radius, extra_margin):
        """(centres (N,2), inflated_radii (N,)) for collision/clearance tests."""
        c = self.circles()
        if len(c) == 0:
            return np.zeros((0, 2)), np.zeros((0,))
        return c[:, :2], c[:, 2] + robot_radius + extra_margin

    def clearance(self, points, robot_radius, extra_margin):
        """Signed clearance (m) of each query point (M,2) to the nearest inflated
        circle edge: positive = outside, negative = inside. +inf if no obstacles
        (or none anywhere near the query box -- see _cull). Returns (M,)."""
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        centres, radii = self._inflated(robot_radius, extra_margin)
        if len(centres):
            centres, radii = self._cull(pts, centres, radii)
        if len(centres) == 0:
            return np.full(len(pts), np.inf)
        dx = pts[:, 0][:, None] - centres[None, :, 0]
        dy = pts[:, 1][:, None] - centres[None, :, 1]
        dist = np.sqrt(dx * dx + dy * dy) - radii[None, :]
        return dist.min(axis=1)

    def raw_min_distance(self, point):
        """Distance (m) from a single point to the nearest RAW circle edge
        (no robot inflation). Feeds the tick log's `dmem` column -- how close the
        car is to the obstacles the POLICY remembers. +inf if no obstacles.
        NOTE: current positions, not predictions -- like the collision guard,
        this column reports the world as last seen."""
        c = self.circles()
        if len(c) == 0:
            return np.inf
        d = np.hypot(c[:, 0] - point[0], c[:, 1] - point[1]) - c[:, 2]
        return float(d.min())
