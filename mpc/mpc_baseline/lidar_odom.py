"""Lidar scan-matching odometry (ICP) -- the pose source for the live MPC.

This car's wheel /odom is dead on the forward/back axis: measured on the car, a
0.28 m forward drive is reported by /odom as ~0.025 m (~10% of true), while strafe
and yaw come back ~90% correct. The firmware computes odometry from a mis-mapped
wheel frame, so the forward component cancels. Since B is straight ahead, wheel
/odom cannot measure progress and closed-loop control on it fails.

Lidar odometry sidesteps it: accumulate the rigid-body transform between
consecutive /scan frames (point-to-point ICP) into a global pose. Validated on the
car -- the 0.28 m forward drive above is recovered here as 0.283 m, and a 0.53 m
strafe as 0.528 m, signs correct (forward -> +x, left -> +y).

Pose is (x, y, yaw) in a frame fixed at the first scan, x forward, yaw CCW. Runs
on the workstation off /scan (small, ~7.5 Hz); the user has OK'd moving it on-board
later if WiFi latency ever matters.
"""

import math
import time
import threading
from collections import namedtuple

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

LidarPose = namedtuple("LidarPose", ["x", "y", "yaw", "age"])


def scan_to_points(msg, lidar_yaw=math.pi, lidar_x=0.0, lidar_y=0.0):
    """LaserScan -> (N, 2) base-frame points. The laser->base transform matches
    viz.launch (lidar_yaw=pi, mounted at the drive centre), same as the perception
    node, so these points share the /obstacles frame."""
    r = np.asarray(msg.ranges, dtype=float)
    n = len(r)
    a = msg.angle_min + np.arange(n) * msg.angle_increment
    rmax = msg.range_max if msg.range_max > 0 else 12.0
    ok = np.isfinite(r) & (r > msg.range_min) & (r < rmax)
    lx = r[ok] * np.cos(a[ok])
    ly = r[ok] * np.sin(a[ok])
    c, s = math.cos(lidar_yaw), math.sin(lidar_yaw)
    return np.column_stack([c * lx - s * ly + lidar_x, s * lx + c * ly + lidar_y])


def _best_rigid(P, Q):
    """Least-squares rigid transform (R, t) with Q ~ R@P + t (Umeyama/SVD)."""
    mp, mq = P.mean(0), Q.mean(0)
    H = (P - mp).T @ (Q - mq)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt = Vt.copy()
        Vt[-1] *= -1.0
        R = Vt.T @ U.T
    return R, mq - R @ mp


def icp(P, Q, iters=25, max_corr=0.4, init_yaw=0.0):
    """Point-to-point ICP aligning P onto Q. Returns (R, t, resid) with Q ~ R@P+t.
    For incremental odometry the motion is small, so an identity init converges."""
    c, s = math.cos(init_yaw), math.sin(init_yaw)
    Pc = (np.array([[c, -s], [s, c]]) @ P.T).T
    R_tot = np.array([[c, -s], [s, c]])
    t_tot = np.zeros(2)
    resid = float("inf")
    for _ in range(iters):
        d = np.linalg.norm(Pc[:, None, :] - Q[None, :, :], axis=2)   # (Np, Nq)
        j = d.argmin(1)
        dm = d[np.arange(len(Pc)), j]
        m = dm < max_corr
        if m.sum() < 15:
            break
        R, t = _best_rigid(Pc[m], Q[j[m]])
        Pc = (R @ Pc.T).T + t
        R_tot = R @ R_tot
        t_tot = R @ t_tot + t
        resid = float(dm[m].mean())
        if np.hypot(*t) < 1e-4 and abs(math.atan2(R[1, 0], R[0, 0])) < 1e-4:
            break
    return R_tot, t_tot, resid


class LidarOdometry(object):
    """Accumulates a global (x, y, yaw) pose from /scan by incremental ICP.
    Thread-safe; the ICP runs in the scan callback."""

    def __init__(self, scan_topic="/scan", lidar_yaw=math.pi, subsample=200,
                 max_corr=0.4, iters=25):
        self.lidar_yaw = lidar_yaw
        self.subsample = subsample
        self.max_corr = max_corr
        self.iters = iters
        self._pose = np.zeros(3)
        self._prev = None
        self._t = None
        self._count = 0
        self._last_dth = 0.0            # last inter-scan ego rotation (ICP warm start)
        self._lock = threading.Lock()
        rospy.Subscriber(scan_topic, LaserScan, self._on_scan, queue_size=1)

    def _on_scan(self, msg):
        pts = scan_to_points(msg, self.lidar_yaw)
        if self.subsample and len(pts) > self.subsample:
            idx = np.linspace(0, len(pts) - 1, self.subsample).astype(int)
            pts = pts[idx]
        with self._lock:
            if self._prev is not None and len(pts) > 20:
                # Two inits -- identity and the last inter-scan rotation -- keep ICP
                # from losing yaw on fast rotation (single identity-init drifts and
                # smears the map into a spiral). Keep the lower-residual result.
                R0, t0, r0 = icp(self._prev, pts, self.iters, self.max_corr, 0.0)
                Rg, tg, rg = icp(self._prev, pts, self.iters, self.max_corr,
                                 -self._last_dth)
                R, t = (R0, t0) if r0 <= rg else (Rg, tg)
                dth = -math.atan2(R[1, 0], R[0, 0])       # ego yaw (inverse of point yaw)
                self._last_dth = dth
                d = -(R.T @ t)                            # ego translation, robot frame
                x, y, th = self._pose
                c, s = math.cos(th), math.sin(th)
                self._pose = np.array([x + d[0] * c - d[1] * s,
                                       y + d[0] * s + d[1] * c, th + dth])
                self._count += 1
            self._prev = pts
            self._t = time.monotonic()

    def pose(self):
        """Latest LidarPose(x, y, yaw, age), or None before the first scan."""
        with self._lock:
            if self._t is None:
                return None
            x, y, th = self._pose
            return LidarPose(x, y, th, time.monotonic() - self._t)

    def updates(self):
        with self._lock:
            return self._count

    def wait(self, timeout=6.0, min_updates=2):
        """Block until at least min_updates ICP steps have accumulated (so the pose
        is moving), or timeout. Returns True if ready."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and not rospy.is_shutdown():
            if self.updates() >= min_updates:
                return True
            time.sleep(0.05)
        return self.updates() >= min_updates

    def reset(self):
        with self._lock:
            self._pose = np.zeros(3)
            self._prev = None
            self._count = 0
            self._last_dth = 0.0
