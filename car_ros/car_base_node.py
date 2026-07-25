#!/usr/bin/env python2
"""ROS base driver for the Yahboom Rosmaster chassis.

Owns the serial link to the MCU on /dev/myserial. Subscribes /cmd_vel, drives
the wheels, and publishes /odom plus the odom->base_link transform. Odom is built
from RAW per-motor encoder deltas (vx/vy scales calibrated vs lidar), with YAW
integrated from the IMU GYRO (yaw_source="gyro") -- NOT the MCU-reported velocity
and NOT the encoder differential (which under-reports yaw in a forward-arc turn).

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
        # "x forward" is flipped in and out. A 180 deg turn about z negates x/y
        # but leaves yaw rate alone, so wz is deliberately not touched.
        self.invert_drive = rospy.get_param("~invert_drive", False)
        self.drive_sign = -1.0 if self.invert_drive else 1.0

        # Odometry from RAW wheel encoders (default): the firmware's get_motion_data
        # has a mis-mapped wheel frame so real forward drive reports ~10% of true.
        # We rebuild (vx,vy,wz) from per-motor deltas with the correct mapping
        # (M1=FL, M2=-RR, M3=FR, M4=-RL) and lidar-ICP scales. ~odom_source=firmware
        # reverts.
        self.odom_source = rospy.get_param("~odom_source", "encoder")
        self.k_lin_x = rospy.get_param("~k_lin_x", 0.000164)   # m per count, forward
        self.k_lin_y = rospy.get_param("~k_lin_y", 0.000176)   # m per count, strafe
        self.k_ang = rospy.get_param("~k_ang", 0.001056)       # rad per count, yaw
        self._prev_enc = None

        # Yaw from the IMU GYRO, not the encoders: the encoder differential is fine
        # in a pure in-place spin but badly under-reports yaw during a forward-arc
        # turn (planner then thinks the car is going straight and spirals off).
        # gyro_sign = IMU z axis vs the odom CCW+ convention (measured OPPOSITE ->
        # -1, via calibration/calib_gyro.py).
        self.yaw_source = rospy.get_param("~yaw_source", "gyro")   # gyro | encoder
        self.gyro_sign = rospy.get_param("~gyro_sign", -1.0)
        self.gyro_bias = rospy.get_param("~gyro_bias", None)       # rad/s; None = auto
        self._gbias_acc = []
        self._gbias_n = int(self.rate_hz * 2.0)                    # ~2 s still at boot

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
        # Raw per-motor encoder counts [m1,m2,m3,m4]; odom is rebuilt from these.
        self.enc_pub = rospy.Publisher("wheel_encoders", Float32MultiArray, queue_size=10)
        self.tf_bc = tf.TransformBroadcaster()
        rospy.Subscriber("cmd_vel", Twist, self.on_cmd_vel, queue_size=10)
        # Direct per-wheel PWM for bring-up/testing: data = [FL, RL, FR, RR],
        # forward-positive, clipped to -100..100. Shares this node's serial link
        # and the cmd_vel watchdog, and does not touch the cmd_vel path.
        rospy.Subscriber("wheel_cmd", Float32MultiArray, self.on_wheel_cmd,
                         queue_size=10)

        rospy.on_shutdown(self.shutdown)

    # Wheel-level drive. Rear motor plugs are polarity-reversed in hardware (a
    # positive command spins them backwards), so mecanum mixing happens here and
    # wheels are driven as raw PWM with M2/M4 negated. Also bypasses the firmware
    # motion controller entirely (it once ran away on wz and ignored velocity stops).
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
        # M1=FL, M2=REAR-RIGHT, M3=FR, M4=REAR-LEFT (the two rear ports are swapped
        # vs the obvious order) and the rear plugs are polarity-reversed, so a
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
            # Reuse the cmd_vel watchdog: wheel_cmd going quiet for cmd_timeout brakes.
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
                # First tick or starved: integrating this dt would teleport odom.
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
                enc = self.bot.get_motor_encoder()          # raw [m1,m2,m3,m4] counts

            # Rebuild body velocity from raw encoder deltas with the correct wheel
            # mapping (overrides the firmware's broken-forward get_motion_data).
            if self.odom_source == "encoder" and self._prev_enc is not None and dt > 0:
                d0 = enc[0] - self._prev_enc[0]
                d1 = enc[1] - self._prev_enc[1]
                d2 = enc[2] - self._prev_enc[2]
                d3 = enc[3] - self._prev_enc[3]
                fl, rr, fr, rl = d0, -d1, d2, -d3           # M1=FL,M2=-RR,M3=FR,M4=-RL
                vx = self.k_lin_x * (fl + rl + fr + rr) / 4.0 / dt
                vy = self.k_lin_y * (-fl + rl + fr - rr) / 4.0 / dt
                wz = self.k_ang * (-fl - rl + fr + rr) / 4.0 / dt
            self._prev_enc = enc

            # Yaw rate: prefer the gyro (see __init__). Estimate rest bias from the
            # first ~2 s at boot, then wz = sign*(gz - bias); fall back to encoder wz
            # until the bias is settled.
            if self.yaw_source == "gyro" and self.gyro_bias is None:
                self._gbias_acc.append(gz)
                if len(self._gbias_acc) >= self._gbias_n:
                    self.gyro_bias = sum(self._gbias_acc) / len(self._gbias_acc)
                    rospy.loginfo("gyro yaw bias = %.5f rad/s (n=%d)",
                                  self.gyro_bias, len(self._gbias_acc))
            if self.yaw_source == "gyro" and self.gyro_bias is not None:
                wz = self.gyro_sign * (gz - self.gyro_bias)

            # Integrate in the odom frame. Mecanum chassis -> vy is real lateral
            # motion and must be carried through.
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
            # Wheel odometry drifts; non-zero covariance so consumers don't trust
            # it absolutely.
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

            self.enc_pub.publish(Float32MultiArray(data=[float(e) for e in enc]))

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
