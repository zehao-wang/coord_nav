#!/usr/bin/env python3
"""Yaw ground-truth check: is the encoder /odom yaw right, and does the IMU gyro
agree with the lidar?

Rotates the car in place a known-ish amount and measures the yaw change three
independent ways:
  * ENCODER  -- the current /odom yaw (rebuilt from wheel counts, k_ang).
  * GYRO     -- integral of /imu/data_raw angular_velocity.z, minus a rest bias.
  * LIDAR    -- incremental ICP on /scan (LidarOdometry), the ground truth here.

If the encoder disagrees with the lidar but the gyro matches, the fix is to drive
odom yaw from the gyro (car_base_node). Also prints the gyro sign relative to the
odom CCW+ convention so the on-car integration uses the right sign.

    roscar
    python smoke/calib_gyro.py --deg 90        # CCW ~90 deg in place

Drives the car (in-place rotation only). Ctrl-C estops.
"""
import os
import sys
import time
import math
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mpc"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "carclient"))

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf.transformations as T
from carclient import CarClient
from mpc_baseline.lidar_odom import LidarOdometry


def yaw_of(q):
    return T.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def unwrap_delta(a, b):
    return math.atan2(math.sin(b - a), math.cos(b - a))


def run(args):
    c = CarClient(init_node=True)
    gz = []                      # (ros_stamp, angular_velocity.z)
    od = []                      # (ros_stamp, yaw)
    rospy.Subscriber("/imu/data_raw", Imu,
                     lambda m: gz.append((m.header.stamp.to_sec(), m.angular_velocity.z)))
    rospy.Subscriber("/odom", Odometry,
                     lambda m: od.append((m.header.stamp.to_sec(), yaw_of(m.pose.pose.orientation))))
    lo = LidarOdometry()
    if not lo.wait(timeout=6.0):
        print("no lidar odometry (no /scan?) -- ground truth unavailable")
    time.sleep(1.5)
    if not gz or not od:
        print("no /imu or /odom"); return

    # NOTE: the car's clock is skewed from the workstation, so header stamps are
    # NOT comparable to rospy.Time.now(). Slice the gyro stream by ARRIVAL INDEX
    # (clock-independent); integrate using the stamps' own differences (dt is fine
    # under a constant offset).

    # 1) rest: estimate gyro bias + snapshot start yaws
    i_rest0 = len(gz)
    time.sleep(2.0)
    i_rest1 = len(gz)
    g = np.array(gz)
    bias = g[i_rest0:i_rest1, 1].mean() if i_rest1 - i_rest0 > 2 else 0.0
    yaw_od0 = od[-1][1]
    p0 = lo.pose()
    ylid0 = p0.yaw if p0 else float("nan")
    print("gyro bias (rest) = %.5f rad/s  (n=%d)" % (bias, i_rest1 - i_rest0))

    # 2) rotate in place CCW: left wheels back, right wheels fwd
    i_turn0 = len(gz)
    print("rotating in place (CCW) ~%.0f deg ..." % args.deg)
    t0 = time.time()
    while time.time() - t0 < args.secs:
        c.drive_wheels(-args.mag, -args.mag, args.mag, args.mag, 0.3)
        time.sleep(0.15)
    for _ in range(6):
        c.drive_wheels(0, 0, 0, 0, 0.2); time.sleep(0.1)
    time.sleep(0.8)
    i_turn1 = len(gz)

    # 3) three independent yaw deltas over the turn window (index-sliced)
    g = np.array(gz)
    gw = g[i_turn0:i_turn1]
    gyro_dyaw = np.trapz(gw[:, 1] - bias, gw[:, 0]) if len(gw) > 2 else float("nan")
    yaw_od1 = od[-1][1]
    enc_dyaw = unwrap_delta(yaw_od0, yaw_od1)
    p1 = lo.pose()
    lid_dyaw = unwrap_delta(ylid0, p1.yaw) if (p1 and math.isfinite(ylid0)) else float("nan")

    print("\n=== yaw change over the in-place rotation ===")
    print("  ENCODER /odom : %+7.1f deg" % math.degrees(enc_dyaw))
    print("  GYRO integ    : %+7.1f deg  (bias-corrected)" % math.degrees(gyro_dyaw))
    print("  LIDAR ICP (GT): %+7.1f deg" % math.degrees(lid_dyaw))
    if math.isfinite(gyro_dyaw) and math.isfinite(lid_dyaw) and abs(lid_dyaw) > 0.1:
        print("\n  gyro sign vs odom CCW+: %s" %
              ("SAME (+1)" if (gyro_dyaw > 0) == (enc_dyaw > 0) else "OPPOSITE (gyro_sign=-1)"))
        print("  gyro/lidar ratio  = %.3f  (want ~1.0)" % (gyro_dyaw / lid_dyaw))
        print("  encoder/lidar ratio = %.3f  (want ~1.0; <1 = odom under-reports yaw)" %
              (enc_dyaw / lid_dyaw))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--deg", type=float, default=90.0, help="target rotation (informational)")
    ap.add_argument("--secs", type=float, default=4.0, help="rotation drive time (s)")
    ap.add_argument("--mag", type=float, default=22.0, help="wheel PWM magnitude")
    args = ap.parse_args()
    try:
        run(args)
    except BaseException as e:
        try:
            CarClient(init_node=False).estop()
        except Exception:
            pass
        if not isinstance(e, KeyboardInterrupt):
            raise


if __name__ == "__main__":
    main()
