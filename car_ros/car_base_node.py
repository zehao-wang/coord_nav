#!/usr/bin/env python2
"""ROS base driver for the Yahboom Rosmaster chassis.

Owns the serial link to the MCU on /dev/myserial. Subscribes /cmd_vel, drives
the wheels, and publishes /odom plus the odom->base_link transform built by
integrating the velocity the MCU reports back.

Nothing else may hold that serial port while this runs -- in particular
claw-hwd must be stopped (`sh ~/claw.sh --disable hwd`).
"""

import sys
import math
import threading

sys.path.insert(0, "/home/jetson/Rosmaster-App/py_install_V3.3.1")

import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Float32MultiArray

from Rosmaster_Lib import Rosmaster


class CarBase(object):
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/myserial")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.imu_frame = rospy.get_param("~imu_frame", "imu_link")
        self.rate_hz = rospy.get_param("~rate", 20.0)
        # Stop the wheels if /cmd_vel goes quiet. The MCU keeps the last speed
        # forever otherwise, so a dead publisher would leave the car driving.
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 1.0)
        self.publish_odom_tf = rospy.get_param("~publish_odom_tf", True)
        # The MCU's +x drives towards the chassis rear on this build, so ROS's
        # "x is forward" has to be flipped on the way in and on the way back
        # out. A 180 deg turn about z negates x and y but leaves yaw rate
        # alone, so wz is deliberately not touched here.
        self.invert_drive = rospy.get_param("~invert_drive", False)
        self.drive_sign = -1.0 if self.invert_drive else 1.0

        self.lock = threading.Lock()
        self.bot = Rosmaster(com=self.port)
        self.bot.create_receive_threading()
        rospy.sleep(0.5)

        version = self.bot.get_version()
        rospy.loginfo("rosmaster on %s, fw=%s", self.port, version)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_cmd_ts = None
        self.moving = False

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=20)
        self.imu_pub = rospy.Publisher("imu/data_raw", Imu, queue_size=20)
        self.batt_pub = rospy.Publisher("battery", BatteryState, queue_size=5)
        self.tf_bc = tf.TransformBroadcaster()
        rospy.Subscriber("cmd_vel", Twist, self.on_cmd_vel, queue_size=10)
        # Direct per-wheel PWM for bring-up/testing: data = [FL, RL, FR, RR],
        # forward-positive, clipped to -100..100. Shares this node's serial link
        # and the cmd_vel watchdog, and does not touch the cmd_vel path.
        rospy.Subscriber("wheel_cmd", Float32MultiArray, self.on_wheel_cmd,
                         queue_size=10)

        rospy.on_shutdown(self.shutdown)

    # Wheel-level drive. The rear motor plugs are keyed with their polarity
    # physically reversed (a positive command spins them backwards), which the
    # MCU kinematics cannot know about -- through set_car_motion a strafe came
    # out as pure rotation. So mecanum mixing happens here and the wheels are
    # driven as raw PWM with M2/M4 negated. This also bypasses the firmware's
    # internal motion controller entirely, which once ran away on a wz command
    # and ignored velocity-level stops (2026-07-22).
    PWM_PER_MPS = 200.0   # ~0.4 m/s -> PWM 80; the lidar loop absorbs scale error
    WZ_ARM = 0.5          # wz mixing arm, m/rad (wz is banned at the bridge anyway)

    def _set_motors(self, vx, vy, wz):
        m1 = (vx - vy - wz * self.WZ_ARM) * self.PWM_PER_MPS   # front-left
        m2 = (vx + vy - wz * self.WZ_ARM) * self.PWM_PER_MPS   # rear-left
        m3 = (vx + vy + wz * self.WZ_ARM) * self.PWM_PER_MPS   # front-right
        m4 = (vx - vy + wz * self.WZ_ARM) * self.PWM_PER_MPS   # rear-right
        clip = lambda v: max(-100, min(100, int(round(v))))
        # rear polarity is reversed in hardware -> negate M2/M4 here
        self.bot.set_motor(clip(m1), clip(-m2), clip(m3), clip(-m4))

    def _set_wheels(self, fl, rl, fr, rr):
        # Direct per-wheel drive, forward-positive. Motor ports on this build:
        #   M1 = front-left,  M2 = REAR-RIGHT,
        #   M3 = front-right, M4 = REAR-LEFT
        # i.e. the two REAR ports are swapped vs the obvious order (verified
        # per-wheel 2026-07-23). The rear plugs are also polarity-reversed, so a
        # forward command is negated on the rear motors. Net: rl -> M4, rr -> M2.
        clip = lambda v: max(-100, min(100, int(round(v))))
        self.bot.set_motor(clip(fl), clip(-rr), clip(fr), clip(-rl))

    def on_wheel_cmd(self, msg):
        d = list(msg.data)
        if len(d) != 4:
            rospy.logwarn("wheel_cmd needs 4 values [FL,RL,FR,RR], got %d", len(d))
            return
        fl, rl, fr, rr = d
        with self.lock:
            self._set_wheels(fl, rl, fr, rr)
            self.last_cmd_ts = rospy.Time.now()
            # Reuse the cmd_vel watchdog: if wheel_cmd goes quiet for
            # cmd_timeout, check_watchdog brakes via _set_motors(0,0,0).
            self.moving = bool(fl or rl or fr or rr)

    def on_cmd_vel(self, msg):
        vx = msg.linear.x * self.drive_sign
        vy = msg.linear.y * self.drive_sign
        wz = msg.angular.z
        with self.lock:
            self._set_motors(vx, vy, wz)
            self.last_cmd_ts = rospy.Time.now()
            self.moving = bool(vx or vy or wz)

    def check_watchdog(self, now):
        if not self.moving or self.last_cmd_ts is None:
            return
        if (now - self.last_cmd_ts).to_sec() < self.cmd_timeout:
            return
        rospy.logwarn("cmd_vel timeout, stopping")
        with self.lock:
            self._set_motors(0.0, 0.0, 0.0)
            self.moving = False

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        last = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last).to_sec()
            last = now
            if dt <= 0.0 or dt > 1.0:
                # First tick, or we were starved -- integrating this would
                # teleport the robot. Skip and pick up on the next one.
                rate.sleep()
                continue

            self.check_watchdog(now)

            with self.lock:
                vx, vy, wz = self.bot.get_motion_data()
            # Measured velocity comes back in the MCU's frame too, so undo the
            # same flip before it reaches odometry.
            vx *= self.drive_sign
            vy *= self.drive_sign
            with self.lock:
                ax, ay, az = self.bot.get_accelerometer_data()
                gx, gy, gz = self.bot.get_gyroscope_data()
                volt = self.bot.get_battery_voltage()

            # Integrate in the odom frame. The chassis is mecanum, so vy is
            # real lateral motion and has to be carried through.
            delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
            delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
            self.x += delta_x
            self.y += delta_y
            self.th += wz * dt
            self.th = math.atan2(math.sin(self.th), math.cos(self.th))

            quat = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.th))

            if self.publish_odom_tf:
                self.tf_bc.sendTransform(
                    (self.x, self.y, 0.0),
                    (quat.x, quat.y, quat.z, quat.w),
                    now, self.base_frame, self.odom_frame)

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = quat
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = wz
            # Wheel odometry drifts; give the consumers a non-zero covariance
            # so they weight it sanely instead of trusting it absolutely.
            odom.pose.covariance[0] = 1e-3
            odom.pose.covariance[7] = 1e-3
            odom.pose.covariance[35] = 1e-2
            odom.twist.covariance[0] = 1e-3
            odom.twist.covariance[7] = 1e-3
            odom.twist.covariance[35] = 1e-2
            self.odom_pub.publish(odom)

            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = self.imu_frame
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            # No orientation estimate from this driver.
            imu.orientation_covariance[0] = -1.0
            self.imu_pub.publish(imu)

            batt = BatteryState()
            batt.header.stamp = now
            batt.voltage = volt
            batt.present = volt > 0.0
            self.batt_pub.publish(batt)

            rate.sleep()

    def shutdown(self):
        rospy.loginfo("stopping wheels")
        try:
            with self.lock:
                self._set_motors(0.0, 0.0, 0.0)
        except Exception as exc:
            rospy.logerr("failed to stop wheels: %s", exc)


if __name__ == "__main__":
    rospy.init_node("car_base")
    try:
        CarBase().spin()
    except rospy.ROSInterruptException:
        pass
