#!/usr/bin/env python2
"""Discrete mecanum drive-action server (open-loop timed pulses).

The GUI / a future model sends ONE discrete action; this node looks up the
standard mecanum inverse-kinematics pattern, scales it by a magnitude, and
drives that wheel vector for a fixed duration -- republishing /wheel_cmd at
20 Hz the whole time so car_base's ~1 s watchdog never brakes mid-step.

SAFETY: the MCU firmware has NO motor timeout, so a single lost "stop" latches
the wheels forever. Therefore every move is followed by a SUSTAINED brake: we
keep publishing [0,0,0,0] for STOP_HOLD_S (~30 messages), so a few dropped stops
can't cause a runaway. All shared state is guarded by a lock (the /drive_action
callback and the 20 Hz timer run on different threads).

  in :  /drive_action  std_msgs/Float32MultiArray  data=[action_id, magnitude, duration_s]
  out:  /wheel_cmd     std_msgs/Float32MultiArray  [FL,RL,FR,RR], -100..100
        /drive_result  std_msgs/String  JSON per finished/stopped move
        /battery_v     std_msgs/Float32  MCU voltage (BatteryState is md5-incompatible cross-distro)
"""

import json
import threading

import rospy
from std_msgs.msg import Float32MultiArray, String, Float32
from sensor_msgs.msg import BatteryState

ACTIONS = {
    0:  [0.0,  0.0,  0.0,  0.0],   # STOP
    1:  [1.0,  1.0,  1.0,  1.0],   # forward
    2:  [0.0,  1.0,  1.0,  0.0],   # forward-left
    3:  [-1.0, 1.0,  1.0, -1.0],   # strafe-left
    4:  [-1.0, 0.0,  0.0, -1.0],   # back-left
    5:  [-1.0, -1.0, -1.0, -1.0],  # back
    6:  [0.0, -1.0, -1.0,  0.0],   # back-right
    7:  [1.0, -1.0, -1.0,  1.0],   # strafe-right
    8:  [1.0,  0.0,  0.0,  1.0],   # forward-right
    9:  [-1.0, -1.0, 1.0,  1.0],   # rotate-left (ccw)
    10: [1.0,  1.0, -1.0, -1.0],   # rotate-right (cw)
}

ZERO = [0.0, 0.0, 0.0, 0.0]


class DriveActionNode(object):
    def __init__(self):
        self.rate_hz = rospy.get_param("~rate_hz", 20.0)
        self.def_mag = rospy.get_param("~default_magnitude", 40.0)
        self.def_dur = rospy.get_param("~default_duration_s", 0.8)
        self.max_mag = rospy.get_param("~max_magnitude", 100.0)
        self.max_dur = rospy.get_param("~max_duration_s", 3.0)
        self.stop_hold_s = rospy.get_param("~stop_hold_s", 1.5)   # sustained brake

        self._lock = threading.Lock()
        self.vec = None            # [FL,RL,FR,RR] currently driven, or None
        self.action = 0
        self.mag = 0.0
        self.dur = 0.0
        self.t_start = None
        self.end_time = None
        self.brake_until = None    # keep sending zeros until this time

        self.wheel_pub = rospy.Publisher("wheel_cmd", Float32MultiArray, queue_size=10)
        self.result_pub = rospy.Publisher("drive_result", String, queue_size=10)
        rospy.Subscriber("drive_action", Float32MultiArray, self.on_action, queue_size=10)

        self.batt_pub = rospy.Publisher("battery_v", Float32, queue_size=5)
        self._batt_min_period = 1.0 / 3.0     # throttle /battery_v to ~3 Hz
        self._batt_last = None
        rospy.Subscriber("battery", BatteryState, self.on_battery, queue_size=5)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.on_tick)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo("drive_action: rate=%.0fHz def_mag=%.0f def_dur=%.2fs stop_hold=%.1fs",
                      self.rate_hz, self.def_mag, self.def_dur, self.stop_hold_s)

    # ---- command intake --------------------------------------------------
    def on_action(self, msg):
        d = list(msg.data)
        if not d:
            return
        action = int(round(d[0]))
        if action not in ACTIONS:
            rospy.logwarn("drive_action: bad id %d", action)
            return
        mag = d[1] if len(d) > 1 and d[1] > 0 else self.def_mag
        dur = d[2] if len(d) > 2 and d[2] > 0 else self.def_dur
        mag = max(0.0, min(self.max_mag, mag))
        dur = max(0.0, min(self.max_dur, dur))
        now = rospy.Time.now()

        with self._lock:
            if self.vec is not None:
                # end the running move (emits result, arms the sustained brake)
                self._finish_locked("stopped" if action == 0 else "superseded", now)
            if action == 0:
                self.action = 0
                self._emit(0, "stopped", 0.0, 0.0, 0)
                out = ZERO
            else:
                self.vec = [w * mag for w in ACTIONS[action]]
                self.action, self.mag, self.dur = action, mag, dur
                self.t_start = now
                self.end_time = now + rospy.Duration(dur)
                self.brake_until = None           # a new move cancels braking
                out = list(self.vec)
        self.wheel_pub.publish(Float32MultiArray(data=out))   # act immediately

    # ---- 20 Hz keep-alive / sustained brake ------------------------------
    def on_tick(self, _evt):
        now = rospy.Time.now()
        with self._lock:
            if self.vec is not None:
                if now >= self.end_time:
                    self._finish_locked("completed", now)   # arms brake_until
                    out = ZERO
                else:
                    out = list(self.vec)
            elif self.brake_until is not None:
                if now < self.brake_until:
                    out = ZERO                              # sustained brake
                else:
                    self.brake_until = None
                    out = None
            else:
                out = None
        if out is not None:
            self.wheel_pub.publish(Float32MultiArray(data=out))

    def _finish_locked(self, reason, now):
        """Caller holds the lock. End the move, emit, and arm a sustained brake
        so a few dropped stop messages cannot leave the wheels latched."""
        took = int((now - self.t_start).to_sec() * 1000) if self.t_start else 0
        self._emit(self.action, reason, self.mag, self.dur, took)
        self.vec = None
        self.end_time = None
        self.brake_until = now + rospy.Duration(self.stop_hold_s)

    def _emit(self, action, reason, mag, dur, took):
        self.result_pub.publish(String(data=json.dumps({
            "action": action, "done": True, "reason": reason,
            "took_ms": took, "magnitude": round(mag, 1), "duration_s": round(dur, 2),
        })))

    def on_battery(self, msg):
        now = rospy.Time.now()
        if self._batt_last is not None and (now - self._batt_last).to_sec() < self._batt_min_period:
            return                                    # throttle to ~3 Hz
        self._batt_last = now
        self.batt_pub.publish(Float32(data=msg.voltage))

    def _shutdown(self):
        for _ in range(5):
            self.wheel_pub.publish(Float32MultiArray(data=ZERO))


if __name__ == "__main__":
    rospy.init_node("drive_action")
    DriveActionNode()
    rospy.spin()
