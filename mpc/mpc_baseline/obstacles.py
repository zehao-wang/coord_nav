"""Obstacle bookkeeping for the planner (pure numpy, no ROS).

The car publishes /obstacles as base-frame circles that only reflect the CURRENT
scan (no map). During a go-around the obstacle can slip out of the lidar FOV, so
this holds a short rolling memory in the ODOM frame: each frame's circles are
transformed to odom, merged with recent ones, and old/far ones are pruned. Set
ObstacleConfig.mem_time_s = 0 to fall back to current-frame-only.

The field also estimates each tracked circle's VELOCITY from the frame sequence
(constant-velocity tracking: per-track finite difference of RAW observations +
EMA + a battery of gates tuned on real recordings -- see _vel). Estimation
always runs -- it is pure bookkeeping -- but it only changes planning when a
policy asks for `predict()` / `clearance_pred()` (the *_t "time-aware" registry
variants do; the plain variants keep the frozen-world queries). This is the
standard constant-velocity-prediction baseline for MPC with dynamic obstacles,
and it lives HERE so the live runner and the sim share it verbatim.

A structural invariant of `update`: EVERY TRACK ABSORBS AT MOST ONE OBSERVATION
PER FRAME. Without it, a physical object split into two circles by clustering
double-merged into one track, and the dt=0 second merge overwrote the raw-obs
anchor -- the next frame then measured a constant cross-centroid "velocity"
that was perfectly coherent and sustained (a gated 0.45 m/s phantom on a STATIC
object). With the invariant, a split spawns a SECOND track next to the first,
and the isolation gate silences both.

All planner queries are vectorised: pass an (M, 2) array of query points and get
per-point clearance or collision flags against every remembered circle.
"""

import numpy as np

from .kinematics import base_to_odom

# Column layout of ObstacleField._mem. (X, Y) is the EMA-smoothed position that
# planning queries use; (OX, OY) is the last RAW observation -- velocity must
# difference raw observations, because the EMA position lags a steady mover by
# v*T and differencing against it overestimates the speed by exactly 2x.
# (SVX, SVY, SMAG) are decayed sums of raw velocity samples for the coherence
# gate. (A1*, A2*) are ping-pong anchors for the net-displacement gate: A2 is
# refreshed at half-window cadence, A1 is the retired previous A2, so the
# displacement baseline stays roughly [window/2, window] seconds old (and the
# gate rate-normalises by the ACTUAL anchor age, so sighting gaps cannot
# stretch the window silently).
X, Y, R, T = 0, 1, 2, 3
VX, VY, N = 4, 5, 6
OX, OY = 7, 8
SVX, SVY, SMAG = 9, 10, 11
A1X, A1Y, A1T = 12, 13, 14
A2X, A2Y, A2T = 15, 16, 17
NCOLS = 18


class ObstacleField(object):
    def __init__(self, cfg, wall_clock):
        """cfg: ObstacleConfig. wall_clock: a zero-arg callable returning seconds
        (time.monotonic live, or the sim clock offline) -- injected so the field
        never calls a real clock itself and stays deterministic in sim."""
        self.cfg = cfg
        self._clock = wall_clock
        self._mem = np.zeros((0, NCOLS), dtype=float)

    @staticmethod
    def _rows(fresh, now):
        fresh = np.asarray(fresh, dtype=float).reshape(-1, 3)
        n = len(fresh)
        return np.column_stack([fresh, np.full(n, now),
                                np.zeros((n, 2)),        # vx, vy
                                np.ones(n),              # sightings
                                fresh[:, :2],            # last raw obs
                                np.zeros((n, 3)),        # svx, svy, smag
                                fresh[:, :2], np.full(n, now),   # anchor 1
                                fresh[:, :2], np.full(n, now)])  # anchor 2

    def _vel(self):
        """(N, 2) usable velocity per track, gated so REAL perception on a
        static world reads exactly 0 (and time-aware planning degenerates to
        the frozen-world planner). Replaying 220 recorded on-car ticks showed
        naive sightings+deadband gating leaking phantom velocities on 95% of
        ticks (wall-cluster centroid slide + association churn, p90 0.245 m/s
        -- overlapping pedestrian speeds). The gates target the MECHANISMS;
        tuned on that recording plus noise-matched injected movers
        (scripts/tracker_eval.py), they cut phantom events by ~97% while
        acquiring 0.25-0.35 m/s movers in ~2 s:

          sightings >= vel_min_sightings   young tracks say nothing
          speed     >= vel_deadband        cheap belt-and-braces floor (the
                                           displacement gate below subsumes it
                                           in every recorded scenario)
          coherence >= vel_coherence       |sum of raw samples|/sum|samples|:
                                           association churn flips direction,
                                           a real mover does not
          isolation >= vel_isolation       walls arrive as CHAINS of clusters
                                           (and a split object becomes TWO
                                           adjacent tracks); a mover crossing
                                           open floor is alone
          disp rate >= min_disp/window     net displacement of raw obs per
                                           second of ACTUAL anchor age: slide
                                           wanders, a mover goes somewhere,
                                           and a sighting gap only makes the
                                           bar proportionally higher
        """
        if not len(self._mem):
            return np.zeros((0, 2))
        m = self._mem
        now = self._clock()
        v = m[:, VX:VY + 1].copy()
        sp = np.hypot(v[:, 0], v[:, 1])
        coher = np.where(m[:, SMAG] > 1e-9,
                         np.hypot(m[:, SVX], m[:, SVY]) /
                         np.maximum(m[:, SMAG], 1e-9),
                         0.0)
        disp = np.hypot(m[:, OX] - m[:, A1X], m[:, OY] - m[:, A1Y])
        age = np.maximum(now - m[:, A1T], 0.5 * self.cfg.vel_disp_window)
        disp_rate = disp / age
        min_rate = self.cfg.vel_min_disp / self.cfg.vel_disp_window
        if len(m) > 1:
            dx = m[:, X][:, None] - m[:, X][None, :]
            dy = m[:, Y][:, None] - m[:, Y][None, :]
            dd = np.hypot(dx, dy)
            np.fill_diagonal(dd, np.inf)
            iso = dd.min(axis=1)
        else:
            iso = np.full(len(m), np.inf)
        gate = ((m[:, N] >= self.cfg.vel_min_sightings) &
                (sp >= self.cfg.vel_deadband) &
                (coher >= self.cfg.vel_coherence) &
                (iso >= self.cfg.vel_isolation) &
                (disp_rate >= min_rate))
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
                np.zeros((0, NCOLS))
            return fresh

        # prune stale and far-away memory relative to the car's odom position
        if len(self._mem):
            age_ok = (now - self._mem[:, T]) <= self.cfg.mem_time_s
            d = np.hypot(self._mem[:, X] - pose[0], self._mem[:, Y] - pose[1])
            near_ok = d <= self.cfg.mem_radius
            self._mem = self._mem[age_ok & near_ok]

        # Merge each fresh circle into at most one track, and each track absorbs
        # at most ONE circle per frame (see module docstring for why that
        # invariant is load-bearing). Association is against the COASTED raw
        # observation (last raw obs + UNGATED velocity * gap): matching the
        # lagging EMA position with the still-gated zero velocity meant a mover
        # above ~0.30 m/s out-ran merge_dist before its gate ever opened and
        # fragmented forever. A young track's raw velocity may be noisy, but it
        # only biases ASSOCIATION; planning still sees the gated value. YOUNG
        # tracks additionally get a wider match radius (assoc_young_boost): a
        # fast mover's first re-sighting lands merge_dist + v*tick away.
        claimed = set()
        for row in fresh:
            if len(self._mem):
                gap = now - self._mem[:, T]
                px = self._mem[:, OX] + self._mem[:, VX] * gap
                py = self._mem[:, OY] + self._mem[:, VY] * gap
                d = np.hypot(px - row[0], py - row[1])
                lim = np.where(self._mem[:, N] < 3,
                               self.cfg.merge_dist + self.cfg.assoc_young_boost,
                               self.cfg.merge_dist)
                # eligible = within its OWN limit and not already fed this
                # frame; pick the nearest eligible (argmin over all tracks used
                # to shadow a young track whose boosted radius covered the
                # observation whenever a mature track was marginally nearer)
                elig = d <= lim
                if claimed:
                    elig[list(claimed)] = False
                if elig.any():
                    j = int(np.argmin(np.where(elig, d, np.inf)))
                    claimed.add(j)
                    self._merge(j, row, now)
                    continue
            self._mem = np.vstack([self._mem, self._rows(row[None, :], now)])
            claimed.add(len(self._mem) - 1)   # a track born this frame is fed
        return self.circles()

    def _merge(self, j, row, now):
        """Fold one observation into track j (call at most once per frame)."""
        m = self._mem
        dt = now - m[j, T]
        if dt > 1e-6:
            # raw velocity = delta of RAW observations (see column notes), EMA'd
            # and capped so one bad association cannot invent a fast phantom
            rvx = (row[0] - m[j, OX]) / dt
            rvy = (row[1] - m[j, OY]) / dt
            sp = np.hypot(rvx, rvy)
            if sp > self.cfg.vel_cap:
                rvx *= self.cfg.vel_cap / sp
                rvy *= self.cfg.vel_cap / sp
            a = self.cfg.vel_ema
            m[j, VX] = (1 - a) * m[j, VX] + a * rvx
            m[j, VY] = (1 - a) * m[j, VY] + a * rvy
            dcy = self.cfg.vel_coher_decay
            m[j, SVX] = dcy * m[j, SVX] + rvx
            m[j, SVY] = dcy * m[j, SVY] + rvy
            m[j, SMAG] = dcy * m[j, SMAG] + np.hypot(rvx, rvy)
        # EMA on position so a moving obstacle isn't frozen at its first-seen
        # spot; keep the larger radius (conservative); refresh bookkeeping
        m[j, X] = 0.5 * m[j, X] + 0.5 * row[0]
        m[j, Y] = 0.5 * m[j, Y] + 0.5 * row[1]
        m[j, R] = max(m[j, R], row[2])
        m[j, T] = now
        m[j, N] += 1
        m[j, OX] = row[0]
        m[j, OY] = row[1]
        # ping-pong displacement anchors (see column notes)
        if now - m[j, A2T] > 0.5 * self.cfg.vel_disp_window:
            m[j, A1X:A1T + 1] = m[j, A2X:A2T + 1]
            m[j, A2X] = row[0]
            m[j, A2Y] = row[1]
            m[j, A2T] = now

    def circles(self):
        """Remembered obstacles as (N, 3) [x, y, r] in odom."""
        return self._mem[:, :3].copy() if len(self._mem) else np.zeros((0, 3))

    def velocities(self):
        """(N, 2) estimated odom velocity per remembered circle (gated: exactly
        0 for any track the gates in _vel do not trust)."""
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
        age = now - self._mem[:, T]                          # (N,)
        t = np.minimum(age[None, :] + dts[:, None], self.cfg.pred_cap_s)  # (T, N)
        base = self._mem[:, :3].copy()
        # Moving tracks extrapolate from the RAW last observation: the 50/50
        # position EMA lags a steady mover by exactly v*tick, which put every
        # prediction one tick behind the world (0.07 m at 0.22 m/s, in the
        # cut-in-front direction). Static tracks (gated v == 0) keep the EMA
        # position, so a static world predicts exactly circles() and the
        # *_t == plain byte-identical guarantee survives.
        moving = (vel[:, 0] != 0.0) | (vel[:, 1] != 0.0)
        base[moving, 0] = self._mem[moving, OX]
        base[moving, 1] = self._mem[moving, OY]
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
        car is to the obstacles the POLICY remembers. +inf if no obstacles.
        NOTE: current positions, not predictions -- like the collision guard,
        this column reports the world as last seen."""
        c = self.circles()
        if len(c) == 0:
            return np.inf
        d = np.hypot(c[:, 0] - point[0], c[:, 1] - point[1]) - c[:, 2]
        return float(d.min())
