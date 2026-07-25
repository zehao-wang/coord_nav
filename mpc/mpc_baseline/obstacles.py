"""Obstacle bookkeeping for the planner (pure numpy, no ROS).

The car publishes /obstacles as base-frame circles that only reflect the CURRENT
scan (no map). During a go-around the obstacle can slip out of the lidar FOV, so
this holds a short rolling memory in the ODOM frame: each frame's circles are
transformed to odom, merged with recent ones, and old/far ones are pruned. Set
ObstacleConfig.mem_time_s = 0 to fall back to current-frame-only.

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
        # remembered circles in odom: rows [x, y, r, last_seen_t]
        self._mem = np.zeros((0, 4), dtype=float)

    def update(self, circles_base, pose):
        """Fold one /obstacles frame (base-frame [(x,y,r),...]) into the memory
        using the car pose (x, y, theta) in odom. Returns the merged odom
        circles as (N, 3)."""
        now = self._clock()
        fresh = base_to_odom(circles_base, pose)          # (N, 3)

        if self.cfg.mem_time_s <= 0.0:
            self._mem = np.column_stack(
                [fresh, np.full(len(fresh), now)]) if len(fresh) else \
                np.zeros((0, 4))
            return fresh

        # prune stale and far-away memory relative to the car's odom position
        if len(self._mem):
            age_ok = (now - self._mem[:, 3]) <= self.cfg.mem_time_s
            d = np.hypot(self._mem[:, 0] - pose[0], self._mem[:, 1] - pose[1])
            near_ok = d <= self.cfg.mem_radius
            self._mem = self._mem[age_ok & near_ok]

        # merge each fresh circle: refresh a nearby remembered one, else append
        for row in fresh:
            if len(self._mem):
                d = np.hypot(self._mem[:, 0] - row[0], self._mem[:, 1] - row[1])
                j = int(np.argmin(d))
                if d[j] <= self.cfg.merge_dist:
                    # EMA on position so a moving obstacle isn't frozen at its
                    # first-seen spot; keep larger radius (conservative), refresh ts.
                    self._mem[j, 0] = 0.5 * self._mem[j, 0] + 0.5 * row[0]
                    self._mem[j, 1] = 0.5 * self._mem[j, 1] + 0.5 * row[1]
                    self._mem[j, 2] = max(self._mem[j, 2], row[2])
                    self._mem[j, 3] = now
                    continue
            self._mem = np.vstack([self._mem, [row[0], row[1], row[2], now]])
        return self.circles()

    def circles(self):
        """Remembered obstacles as (N, 3) [x, y, r] in odom."""
        return self._mem[:, :3].copy() if len(self._mem) else np.zeros((0, 3))

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
        (no robot inflation). Used for the collision/clearance metrics and the
        live hard-abort guard. +inf if no obstacles."""
        c = self.circles()
        if len(c) == 0:
            return np.inf
        d = np.hypot(c[:, 0] - point[0], c[:, 1] - point[1]) - c[:, 2]
        return float(d.min())
