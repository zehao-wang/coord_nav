#!/usr/bin/env python3
"""Per-wheel bring-up test for the mecanum car, self-verified via wheel odometry.

Runs on the workstation (ROS Noetic conda env) against the car's master.
Drives each wheel FORWARD one at a time via /wheel_cmd and confirms the wheel
actually turned by watching /odom (which the MCU derives from the encoders), so
the PASS/FAIL result does not depend on anyone watching. Then reads one /scan
and prints a processed obstacle summary.

  /wheel_cmd : std_msgs/Float32MultiArray, data = [FL, RL, FR, RR],
               forward-positive, -100..100. car_base maps rl->M4, rr->M2
               (the rear motor ports are swapped on this build) and handles the
               rear-plug polarity, so positive == forward on every wheel.

SAFETY: the car MUST be elevated (wheels off the ground). Low PWM, short pulses,
always sends a stop on exit; the car_base watchdog also brakes on silence.
"""

import math
import sys
import time

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# --- tunables -----------------------------------------------------------
PWM = 50.0          # per-wheel drive strength, -100..100 (50 clears the deadband)
PULSE_S = 2.5       # seconds to spin each wheel
GAP_S = 2.0         # pause between wheels
PERIOD_S = 0.05     # command/sample period (20 Hz, beats the ~1s watchdog)
MOVE_EPS = 0.02     # |twist| above this counts as "the wheel is turning"
COUNTDOWN = 5       # abort window before motion starts

LIDAR_YAW = math.pi  # lidar mount yaw vs chassis front (viz.launch default)

WHEELS = [("front-left ", 0),
          ("rear-left  ", 1),
          ("front-right", 2),
          ("rear-right ", 3)]


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


class WheelTest(object):
    def __init__(self):
        self.motion = 0.0
        rospy.Subscriber("/odom", Odometry, self._on_odom)
        self.pub = rospy.Publisher("/wheel_cmd", Float32MultiArray, queue_size=10)
        t0 = time.time()
        while self.pub.get_num_connections() < 1 and time.time() - t0 < 5.0:
            time.sleep(0.1)
        if self.pub.get_num_connections() < 1:
            rospy.logwarn("no subscriber on /wheel_cmd -- is car_base running?")

    def _on_odom(self, m):
        t = m.twist.twist
        self.motion = abs(t.linear.x) + abs(t.linear.y) + abs(t.angular.z)

    def send(self, v):
        self.pub.publish(Float32MultiArray(data=v))

    def stop(self):
        for _ in range(3):
            self.send([0, 0, 0, 0]); time.sleep(0.03)

    def drive_and_check(self, idx):
        """Drive one wheel for PULSE_S, return (moving_fraction, sustained)."""
        vals = [0.0, 0.0, 0.0, 0.0]; vals[idx] = PWM
        samples = []
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < PULSE_S:
            self.send(vals)
            # ignore the first 0.3s: odom lags the command a little
            if time.time() - t0 > 0.3:
                samples.append(self.motion)
            time.sleep(PERIOD_S)
        self.stop()
        if not samples:
            return 0.0, False
        moving = [s for s in samples if s > MOVE_EPS]
        frac = len(moving) / float(len(samples))
        tail = samples[-5:]                       # last ~0.25s
        sustained = sum(1 for s in tail if s > MOVE_EPS) >= max(1, len(tail) - 1)
        return frac, sustained

    def run_motion(self):
        print("\n=== per-wheel test: each wheel FORWARD at PWM %d for %.1fs ==="
              % (PWM, PULSE_S))
        print("!!! wheels must be OFF THE GROUND !!!  Ctrl-C to abort.")
        for n in range(COUNTDOWN, 0, -1):
            sys.stdout.write("\r  starting in %d ... " % n); sys.stdout.flush()
            time.sleep(1.0)
        print("\r  go!               ")
        results = []
        for name, idx in WHEELS:
            sys.stdout.write("  -> %s (index %d) ... " % (name, idx))
            sys.stdout.flush()
            frac, sustained = self.drive_and_check(idx)
            ok = frac > 0.6 and sustained
            verdict = "PASS" if ok else ("WEAK/INTERMITTENT" if frac > 0.1 else "NO MOTION")
            print("%s  (turning %.0f%% of the pulse, sustained=%s)"
                  % (verdict, frac * 100, sustained))
            results.append((name, ok))
            time.sleep(GAP_S)
        self.stop()
        bad = [n for n, ok in results if not ok]
        if bad:
            print("  -> check these wheels: %s" % ", ".join(x.strip() for x in bad))
        else:
            print("  -> all four wheels turned forward and sustained. OK.")

    def read_lidar(self):
        print("\n=== processed lidar (/scan) ===")
        try:
            scan = rospy.wait_for_message("/scan", LaserScan, timeout=8.0)
        except rospy.ROSException:
            print("  no /scan received within 8s -- lidar running?")
            return
        pts = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            car_ang = norm(scan.angle_min + i * scan.angle_increment + LIDAR_YAW)
            pts.append((car_ang, r))
        if not pts:
            print("  scan had no valid returns."); return
        nb, nr = min(pts, key=lambda p: p[1])

        def sect(lo, hi):
            vals = [r for a, r in pts if lo <= a < hi]
            return min(vals) if vals else float("inf")
        front = sect(-math.pi/4, math.pi/4)
        left = sect(math.pi/4, 3*math.pi/4)
        right = sect(-3*math.pi/4, -math.pi/4)
        back = min(sect(3*math.pi/4, math.pi + 0.01),
                   sect(-math.pi - 0.01, -3*math.pi/4))
        fmt = lambda v: ("%.2f m" % v) if math.isfinite(v) else "  --  "
        print("  valid returns : %d / %d" % (len(pts), len(scan.ranges)))
        print("  nearest       : %s at %+.0f deg (car frame)"
              % (fmt(nr), math.degrees(nb)))
        print("  front %s | left %s | right %s | back %s"
              % (fmt(front), fmt(left), fmt(right), fmt(back)))


def main():
    rospy.init_node("wheel_test", anonymous=True, disable_signals=True)
    wt = WheelTest()
    try:
        wt.run_motion()
    except KeyboardInterrupt:
        print("\n  aborted -- stopping wheels.")
    finally:
        wt.stop()
    wt.read_lidar()
    print("\nall done.")


if __name__ == "__main__":
    main()
