"""carclient - minimal ROS1 client for the Jetson mecanum car.

Transport is ROS. Point ROS_MASTER_URI at the car (the `roscar` env does this),
then:

    from carclient import CarClient, Action

    car = CarClient()
    car.wait_obstacles(timeout=5)                 # wait for the first frame

    while not car.is_shutdown():                   # a model loop
        obs = car.obstacles()                      # latest circles, non-blocking
        if obs and obs.age < 1.0:
            action = my_model(obs.circles)         # 0..10, see Action
            car.drive(action)                      # send one discrete step
        car.sleep(1 / 3.0)                         # ~3 Hz

Reads /obstacles, publishes /drive_action, listens /drive_result. Latest frame
is cached (thread-safe) so obstacles() never blocks or polls the master.
"""

import os
import json
import time
import threading
import subprocess
from collections import namedtuple, deque

import rospy
from std_msgs.msg import Float32MultiArray, String, Float32

__all__ = ["CarClient", "Action", "Obstacles"]

# circles: list of (x, y, r) in metres, base frame (x forward, y left).
Obstacles = namedtuple("Obstacles", ["frame_id", "circles", "age"])


class Action(object):
    """Discrete mecanum actions (the ids the board's drive_action node uses)."""
    STOP = 0
    FORWARD = 1
    FWD_LEFT = 2
    LEFT = 3
    BACK_LEFT = 4
    BACK = 5
    BACK_RIGHT = 6
    RIGHT = 7
    FWD_RIGHT = 8
    ROT_CCW = 9
    ROT_CW = 10


# Diagonal actions drive only 2 wheels (half the per-axis speed), so their
# magnitude is scaled by CarClient.diag_mult to reach the (n,n) grid cell.
DIAGONALS = frozenset([Action.FWD_LEFT, Action.BACK_LEFT,
                       Action.BACK_RIGHT, Action.FWD_RIGHT])


def _parse(data):
    fid = int(data[0])
    circ = [(data[i], data[i + 1], data[i + 2]) for i in range(1, len(data) - 2, 3)]
    return fid, circ


class CarClient(object):
    # A healthy pack reads ~11-13 V; the MCU reports 0 V when its serial link is
    # wedged (same link as motors/odom), so a low reading == link lost.
    MIN_LINK_VOLT = 6.0

    def __init__(self, magnitude=40.0, duration=0.8, stale_after=1.0,
                 car_ip=None, ssh_key=None, init_node=True, dump_path=None,
                 diag_mult=2.0):
        """magnitude/duration are the (deliberately conservative) defaults for
        drive(); stale_after (s) is when obstacles are considered disconnected.
        car_ip/ssh_key are only for estop() and fall back to $CAR_IP /
        $CAR_SSH_KEY. dump_path: if given, every obstacle frame is appended to
        that file as JSONL so the full record lives on disk while memory stays
        bounded to the last 100 frames."""
        self.default_mag = magnitude
        self.default_dur = duration
        self.diag_mult = diag_mult          # magnitude boost for diagonal actions
        self.stale_after = stale_after
        self.car_ip = car_ip or os.environ.get("CAR_IP", "10.42.0.187")
        self.ssh_key = os.path.expanduser(
            ssh_key or os.environ.get("CAR_SSH_KEY", "~/.ssh/id_ed25519"))

        self._lock = threading.Lock()
        self._latest = None            # (frame_id, circles, monotonic_time)
        self._history = deque(maxlen=100)   # bounded in-memory obstacle history
        self._last_result = None
        self._result_cb = None
        self._volt = None              # (voltage, monotonic_time)

        if init_node and not rospy.core.is_initialized():
            rospy.init_node("carclient", anonymous=True, disable_signals=True)
        self._pub = rospy.Publisher("/drive_action", Float32MultiArray, queue_size=5)
        rospy.Subscriber("/obstacles", Float32MultiArray, self._on_obs, queue_size=2)
        rospy.Subscriber("/drive_result", String, self._on_res, queue_size=10)
        # /battery_v (std_msgs/Float32, republished on the car) not /battery:
        # sensor_msgs/BatteryState has an incompatible md5 across Melodic/Noetic.
        rospy.Subscriber("/battery_v", Float32, self._on_batt, queue_size=5)

        self._dump_fh = open(dump_path, "a") if dump_path else None

    # -- obstacles --------------------------------------------------------
    def _on_obs(self, m):
        if not m.data:
            return
        fid, circ = _parse(m.data)
        now = time.monotonic()
        with self._lock:
            self._latest = (fid, circ, now)
            self._history.append((fid, circ, now))   # bounded queue (maxlen 100)
        if self._dump_fh is not None:                 # dump to disk; memory stays bounded
            try:
                self._dump_fh.write(json.dumps(
                    {"t": time.time(), "frame_id": fid, "circles": circ}) + "\n")
                self._dump_fh.flush()
            except IOError:
                pass

    def obstacles(self):
        """Latest obstacles as Obstacles(frame_id, circles, age), or None if
        none received yet. Non-blocking."""
        with self._lock:
            snap = self._latest
        if snap is None:
            return None
        fid, circ, t = snap
        return Obstacles(fid, circ, time.monotonic() - t)

    def history(self):
        """The in-memory bounded queue (up to 100 recent frames), oldest first,
        as a list of Obstacles(frame_id, circles, age)."""
        now = time.monotonic()
        with self._lock:
            return [Obstacles(f, c, now - t) for (f, c, t) in self._history]

    def dump(self, path):
        """Snapshot the in-memory queue to a JSONL file (one frame per line).
        Returns the number of frames written."""
        with self._lock:
            snap = list(self._history)
        with open(path, "w") as fh:
            for (f, c, _t) in snap:
                fh.write(json.dumps({"frame_id": f, "circles": c}) + "\n")
        return len(snap)

    def wait_obstacles(self, timeout=2.0):
        """Block for the next fresh frame. Returns Obstacles or None on timeout."""
        try:
            m = rospy.wait_for_message("/obstacles", Float32MultiArray, timeout=timeout)
        except rospy.ROSException:
            return None
        fid, circ = _parse(m.data)
        return Obstacles(fid, circ, 0.0)

    def connected(self):
        """True if a non-stale obstacle frame has arrived recently."""
        o = self.obstacles()
        return o is not None and o.age < self.stale_after

    # -- MCU link health (via /battery, same serial as motors) ------------
    def _on_batt(self, m):
        with self._lock:
            self._volt = (m.data, time.monotonic())

    def battery(self):
        """Latest battery voltage (V), or None if no recent reading. Reads 0
        (i.e. < MIN_LINK_VOLT) when the MCU serial link is wedged."""
        with self._lock:
            snap = self._volt
        if snap is None or time.monotonic() - snap[1] > 2.0:
            return None
        return snap[0]

    def link_ok(self):
        """True if the MCU serial link looks alive (fresh, plausible voltage).
        False means motor commands may be lost -- use estop(), not stop()."""
        v = self.battery()
        return v is not None and v > self.MIN_LINK_VOLT

    # -- drive ------------------------------------------------------------
    def drive(self, action, magnitude=None, duration=None):
        """Send one discrete action (0..10). magnitude/duration default to the
        client's. Non-blocking; the board runs the timed pulse and replies on
        /drive_result."""
        mag = self.default_mag if magnitude is None else magnitude
        dur = self.default_dur if duration is None else duration
        if action in DIAGONALS:
            mag = mag * self.diag_mult      # diagonals need a boost to reach (n,n)
        self._pub.publish(Float32MultiArray(data=[float(action), float(mag), float(dur)]))

    def stop(self):
        """Soft stop (halt the wheels now)."""
        self.drive(Action.STOP)

    # -- result -----------------------------------------------------------
    def _on_res(self, s):
        try:
            r = json.loads(s.data)
        except ValueError:
            r = {"raw": s.data}
        with self._lock:
            self._last_result = r
        if self._result_cb is not None:
            self._result_cb(r)

    def last_result(self):
        """The most recent move-completion dict, or None."""
        with self._lock:
            return self._last_result

    def on_result(self, cb):
        """Register cb(result_dict), called on each move completion (rospy
        thread)."""
        self._result_cb = cb

    # -- estop (SSH, ROS-independent) -------------------------------------
    def estop(self):
        """Hard emergency stop: SSHes estop.sh on the board. Kills car-ros and
        zeroes the motors; does not depend on ROS being healthy."""
        subprocess.Popen(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=4",
             "-i", self.ssh_key, "jetson@" + self.car_ip, "sh /home/jetson/estop.sh"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # -- lifecycle passthroughs (so callers need not import rospy) ---------
    def sleep(self, seconds):
        rospy.sleep(seconds)

    def is_shutdown(self):
        return rospy.is_shutdown()

    def close(self):
        if self._dump_fh is not None:
            try:
                self._dump_fh.close()
            except IOError:
                pass
        rospy.signal_shutdown("carclient closed")
