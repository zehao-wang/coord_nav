#!/usr/bin/env python3
"""Slow, one-at-a-time wheel diagnostic. Watch the car and note, for EACH step,
which physical wheel turns and in which direction.

Drives /wheel_cmd index by index at a clearly visible PWM with long gaps.
SAFETY: car elevated, wheels off the ground.
"""
import sys, time
import rospy
from std_msgs.msg import Float32MultiArray

PWM = 50.0
PULSE_S = 3.0
GAP_S = 4.0
RATE_HZ = 10.0

WHEELS = [("index 0  -> expected FRONT-LEFT ", 0),
          ("index 1  -> expected REAR-LEFT  ", 1),
          ("index 2  -> expected FRONT-RIGHT", 2),
          ("index 3  -> expected REAR-RIGHT ", 3)]


def main():
    rospy.init_node("wheel_diag", anonymous=True, disable_signals=True)
    pub = rospy.Publisher("/wheel_cmd", Float32MultiArray, queue_size=10)
    t0 = time.time()
    while pub.get_num_connections() < 1 and time.time() - t0 < 5.0:
        time.sleep(0.1)

    def send(v):
        pub.publish(Float32MultiArray(data=v))

    def stop():
        for _ in range(3):
            send([0, 0, 0, 0]); time.sleep(0.05)

    print("\n=== wheel diagnostic: PWM %d, %.0fs each, %.0fs gap ===" % (PWM, PULSE_S, GAP_S))
    print("Watch the car. For each step below, note: which wheel moved + direction.\n")
    try:
        for name, idx in WHEELS:
            print(">>> NOW DRIVING  %s  (positive = should be forward)" % name)
            vals = [0.0, 0.0, 0.0, 0.0]; vals[idx] = PWM
            rate = rospy.Rate(RATE_HZ); ts = time.time()
            while not rospy.is_shutdown() and time.time() - ts < PULSE_S:
                send(vals); rate.sleep()
            stop()
            print("    ...stopped. (%0.0fs pause)\n" % GAP_S)
            time.sleep(GAP_S)
    except KeyboardInterrupt:
        print("\naborted.")
    finally:
        stop()
    print("done. Tell me for EACH index whether a wheel moved, which one, and direction.")


if __name__ == "__main__":
    main()
