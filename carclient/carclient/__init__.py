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
import math
import time
import threading
import subprocess
from collections import namedtuple, deque, OrderedDict

import rospy
from std_msgs.msg import Float32MultiArray, String, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

__all__ = ["CarClient", "Action", "Obstacles", "Pose", "ScanPoints", "ObsPoints",
           "Frame"]

# circles: list of (x, y, r) in metres, base frame (x forward, y left).
Obstacles = namedtuple("Obstacles", ["frame_id", "circles", "age"])
# pose from /odom: metres/radians in the odom frame, x forward of the start pose.
Pose = namedtuple("Pose", ["x", "y", "yaw", "age"])
# raw /scan converted to base-frame points: pts = [(x, y), ...] (no radius).
# UNSYNCHRONISED with the circles -- the car samples /scan on its own rate_hz
# timer, so the newest scan here is usually not the one behind the circles.
ScanPoints = namedtuple("ScanPoints", ["pts", "age"])
# /obstacle_points: the points the circles of THIS frame_id were clustered from.
ObsPoints = namedtuple("ObsPoints", ["frame_id", "pts", "age"])
# one sample: circles and points that carry the same frame_id (points may be None).
Frame = namedtuple("Frame", ["frame_id", "circles", "points", "age"])


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
# Strafe is slower than forward (roller drag), scaled by CarClient.strafe_mult.
STRAFE = frozenset([Action.LEFT, Action.RIGHT])


def _parse(data):
    fid = int(data[0])
    circ = [(data[i], data[i + 1], data[i + 2]) for i in range(1, len(data) - 2, 3)]
    return fid, circ


class CarClient(object):
    # A healthy pack reads ~11-13 V; the MCU reports 0 V when its serial link is
    # wedged (same link as motors/odom), so a low reading == link lost.
    MIN_LINK_VOLT = 6.0
    # how many recent /obstacle_points frames to keep for frame_id matching. The
    # car publishes points and circles back-to-back, but ROS gives no cross-topic
    # ordering guarantee, so hold a few either way.
    POINT_FRAMES = 4

    def __init__(self, magnitude=40.0, duration=0.5, stale_after=1.0,
                 car_ip=None, ssh_key=None, init_node=True, dump_path=None,
                 diag_mult=1.6, strafe_mult=1.2,
                 lidar_yaw=math.pi, lidar_x=0.0, lidar_y=0.0,
                 subscribe_scan=False):
        """magnitude/duration are the conservative defaults for drive();
        stale_after (s) is when obstacles count as disconnected. car_ip/ssh_key
        are only for estop() and fall back to $CAR_IP / $CAR_SSH_KEY. dump_path:
        if given, every obstacle frame is appended to it as JSONL (in-memory
        history stays bounded to the last 100 frames). lidar_yaw/x/y is the
        laser->base 2D transform used by scan_points() (match viz.launch).

        subscribe_scan (default OFF) enables the raw /scan subscription behind
        scan_points(). It is off by default because /scan costs ~43 KB/s over the
        car's WiFi (measured: 7.6 Hz x 720 beams) and viewers should use
        observation()/obstacle_points() instead -- those are frame-synced with the
        circles AND a third of the bandwidth. Link latency on this car has already
        been a root cause of closed-loop failure, so we do not hold the link open
        for data nobody reads. Turn it on only if you genuinely want raw beams."""
        self.default_mag = magnitude
        self.default_dur = duration
        self.diag_mult = diag_mult
        self.strafe_mult = strafe_mult
        self.stale_after = stale_after
        self.car_ip = car_ip or os.environ.get("CAR_IP", "10.42.0.187")
        self.ssh_key = os.path.expanduser(
            ssh_key or os.environ.get("CAR_SSH_KEY", "~/.ssh/id_ed25519"))
        self._lyaw, self._lx, self._ly = lidar_yaw, lidar_x, lidar_y

        self._lock = threading.Lock()
        # signalled on every new /obstacles frame -- this is what wait_frame()
        # (the stack's tick boundary) blocks on.
        self._frame_cv = threading.Condition(self._lock)
        self._latest = None            # (frame_id, circles, monotonic_time)
        self._history = deque(maxlen=100)
        self._last_result = None
        self._result_cb = None
        self._volt = None              # (voltage, monotonic_time)
        self._pose = None              # (x, y, yaw, monotonic_time)
        self._scan = None              # (base-frame points [(x,y),...], monotonic_time)
        # frame_id -> (points [(x,y),...], monotonic_time). Ordered: evicted by
        # arrival, not by id (the car's frame counter restarts with car-ros).
        self._opts = OrderedDict()

        if init_node and not rospy.core.is_initialized():
            rospy.init_node("carclient", anonymous=True, disable_signals=True)
        self._pub = rospy.Publisher("/drive_action", Float32MultiArray, queue_size=5)
        # /wheel_cmd [FL,RL,FR,RR] LEGACY direct wheel drive (manual/bring-up only):
        # BYPASSES all safety harness AND has no keep-alive. Streaming it over lossy
        # WiFi could burst-wedge the MCU serial -- prefer /drive_wheels below.
        self._wheel_pub = rospy.Publisher("/wheel_cmd", Float32MultiArray, queue_size=5)
        # /drive_wheels [FL,RL,FR,RR,duration] velocity PULSE -> the car keep-alives it
        # locally (robust to WiFi loss). Robust actuation path for the variant-1 MPC.
        self._dwheels_pub = rospy.Publisher("/drive_wheels", Float32MultiArray, queue_size=5)
        # Open the dump file BEFORE subscribing: /obstacles can fire _on_obs the instant
        # the subscriber registers, before a later assignment sets _dump_fh (race).
        self._dump_fh = open(dump_path, "a") if dump_path else None
        rospy.Subscriber("/obstacles", Float32MultiArray, self._on_obs, queue_size=2)
        # /obstacle_points carries the SAME frame_id as /obstacles -- pair on it via
        # observation() instead of guessing from the unsynchronised /scan cache.
        rospy.Subscriber("/obstacle_points", Float32MultiArray,
                         self._on_obs_points, queue_size=2)
        rospy.Subscriber("/drive_result", String, self._on_res, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self._on_odom, queue_size=5)
        self._scan_on = bool(subscribe_scan)
        if self._scan_on:                        # opt-in: ~43 KB/s, see __init__ doc
            rospy.Subscriber("/scan", LaserScan, self._on_scan, queue_size=1)
        # /battery_v (std_msgs/Float32, republished on the car) not /battery:
        # sensor_msgs/BatteryState has an incompatible md5 across Melodic/Noetic.
        rospy.Subscriber("/battery_v", Float32, self._on_batt, queue_size=5)

    # -- obstacles --------------------------------------------------------
    def _on_obs(self, m):
        if not m.data:
            return
        fid, circ = _parse(m.data)
        now = time.monotonic()
        with self._frame_cv:               # same underlying lock as self._lock
            self._latest = (fid, circ, now)
            self._history.append((fid, circ, now))
            self._frame_cv.notify_all()    # release everyone waiting on the tick
        if self._dump_fh is not None:
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

    # -- /obstacle_points: the points THIS frame's circles came from -----
    def _on_obs_points(self, m):
        if not m.data:
            return
        fid = int(m.data[0])
        d = m.data
        pts = [(d[i], d[i + 1]) for i in range(1, len(d) - 1, 2)]
        with self._frame_cv:               # same underlying lock as self._lock
            # Evict by ARRIVAL ORDER, never by frame_id value: the car's
            # frame_counter_ restarts at 1 whenever car-ros does (the watchdog and
            # the GUI's Restart button both do this routinely). Keeping the
            # numerically largest ids would pin the cache to the pre-restart ids
            # and drop every new frame on arrival -- points would go permanently
            # None until the client was restarted.
            self._opts[fid] = (pts, time.monotonic())
            self._opts.move_to_end(fid)
            while len(self._opts) > self.POINT_FRAMES:
                self._opts.popitem(last=False)
            self._frame_cv.notify_all()    # wake wait_frame's points grace period

    def obstacle_points(self, frame_id=None):
        """Base-frame points from /obstacle_points as ObsPoints(frame_id, pts, age).

        These are the scan points the circles of that frame were computed from, so
        they pair with obstacles() by frame_id. They are the INPUT to clustering:
        a few percent are DBSCAN noise lying inside no circle, and the circles are
        additionally EMA-smoothed. Same sample, not a per-point correspondence. `frame_id=None` returns the
        most recently ARRIVED frame (not the numerically largest id -- the car's
        counter restarts with car-ros); a specific id returns that frame or None.
        Unlike scan_points() this is synchronised with the circles -- see
        observation().
        """
        with self._lock:
            if not self._opts:
                return None
            fid = next(reversed(self._opts)) if frame_id is None else frame_id
            snap = self._opts.get(fid)
        if snap is None:
            return None
        pts, t = snap
        return ObsPoints(fid, pts, time.monotonic() - t)

    # A tick fires on /obstacles, but the car publishes /obstacle_points in the same
    # process() call microseconds later and ROS gives no cross-topic ordering, so the
    # circles almost always win the race. Without a grace period wait_frame returned
    # points=None on 24 of 25 real ticks. Measured skew is sub-millisecond; 60 ms is
    # 18% of a tick and still leaves ~270 ms to plan in.
    POINTS_GRACE_S = 0.06

    def wait_frame(self, after=None, timeout=1.0, want_points=True):
        """Block until an observation frame OTHER than `after` arrives; return it.

        This is the TICK boundary of the whole stack: perception publishes at a
        fixed rate on the car and every consumer (planner, actuation, GUI render)
        advances on the same frame, so nobody re-plans on a frame it already saw
        and nobody skips one. Returns Frame(...) or None on timeout.

        Matching is "different id", not "greater id": the car's frame counter
        restarts at 1 with car-ros, so a `>` test would hang until the counter
        climbed back past the pre-restart value.
        """
        deadline = time.monotonic() + timeout
        with self._frame_cv:
            while True:
                snap = self._latest
                if snap is not None and snap[0] != after:
                    break
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self._frame_cv.wait(left)
            # The tick fired on the circles; give the matching points their few
            # hundred microseconds to land so the tick carries a COMPLETE sample.
            if want_points:
                fid = snap[0]
                grace = time.monotonic() + self.POINTS_GRACE_S
                while fid not in self._opts:
                    left = grace - time.monotonic()
                    if left <= 0:
                        break          # no points for this frame: report it honestly
                    self._frame_cv.wait(left)
        return self.observation()

    def observation(self):
        """One SAMPLE: Frame(frame_id, circles, points, age) -- circles and points
        guaranteed to be the same frame_id, i.e. the same lidar scan processed in
        the same cycle on the car.

        `points` is None when /obstacle_points has not arrived for the current
        circle frame (nothing subscribed yet, publish_points=false, or an older
        car build). It is never filled with a different frame's points -- drawing
        those against these circles is exactly the desync this method exists to
        avoid. Returns None if no circles have arrived at all.
        """
        with self._lock:
            snap = self._latest
            opts = self._opts.get(snap[0]) if snap is not None else None
        if snap is None:
            return None
        fid, circ, t = snap
        now = time.monotonic()
        pts = None if opts is None else opts[0]
        return Frame(fid, circ, pts, now - t)

    # -- raw /scan as base-frame points (no radius; for point-cloud viz) --
    def _on_scan(self, m):
        c, s = math.cos(self._lyaw), math.sin(self._lyaw)
        rmin, rmax = m.range_min, (m.range_max if m.range_max > 0.0 else 12.0)
        a, pts = m.angle_min, []
        for r in m.ranges:
            if r == r and rmin < r < rmax:            # r==r drops NaN; inf<rmax drops inf
                lx, ly = r * math.cos(a), r * math.sin(a)
                pts.append((c * lx - s * ly + self._lx, s * lx + c * ly + self._ly))
            a += m.angle_increment
        with self._lock:
            self._scan = (pts, time.monotonic())

    def scan_points(self):
        """Raw /scan as base-frame points -- requires CarClient(subscribe_scan=True).

        NOT synchronised with the circles: the car samples /scan on its own rate_hz
        timer, so the newest scan here is usually not the one the current circles
        came from. For drawing, use observation() instead.

        Latest /scan as base-frame points ScanPoints(pts=[(x,y),...], age), or
        None if none has arrived yet. Non-blocking. Same frame as obstacles()/the
        top-down view; there is no radius -- the caller chooses one."""
        if not self._scan_on:
            raise RuntimeError(
                "scan_points() needs CarClient(subscribe_scan=True); the raw /scan "
                "subscription is off by default (~43 KB/s of WiFi for data most "
                "callers do not want). For a point cloud that pairs with the "
                "obstacle circles, use observation() / obstacle_points() instead.")
        with self._lock:
            snap = self._scan
        if snap is None:
            return None
        pts, t = snap
        return ScanPoints(pts, time.monotonic() - t)

    # -- pose (/odom, wheel-integrated dead reckoning) --------------------
    def _on_odom(self, m):
        q = m.pose.pose.orientation
        # yaw from the quaternion (planar), no tf dependency
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                          time.monotonic())

    def pose(self):
        """Latest odom pose as Pose(x, y, yaw, age), or None. Non-blocking. Odom
        is wheel-integrated dead reckoning (drifts over time), fine as the local
        reference for a short (~5 s) go-to-B run."""
        with self._lock:
            snap = self._pose
        if snap is None:
            return None
        x, y, yaw, t = snap
        return Pose(x, y, yaw, time.monotonic() - t)

    def wait_pose(self, timeout=2.0):
        """Block for the next /odom message. Returns Pose or None on timeout."""
        try:
            m = rospy.wait_for_message("/odom", Odometry, timeout=timeout)
        except rospy.ROSException:
            return None
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return Pose(m.pose.pose.position.x, m.pose.pose.position.y, yaw, 0.0)

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
            mag = mag * self.diag_mult
        elif action in STRAFE:
            mag = mag * self.strafe_mult
        self._pub.publish(Float32MultiArray(data=[float(action), float(mag), float(dur)]))

    def stop(self):
        """Soft stop (halt the wheels now)."""
        self.drive(Action.STOP)

    def wheels(self, fl, rl, fr, rr):
        """Publish one /wheel_cmd [FL, RL, FR, RR] (forward-positive PWM, the car
        clips to +-100). Direct per-wheel drive for the variant-1 velocity loop.
        BYPASSES the drive_action harness: send a keep-alive stream (car brakes if
        /wheel_cmd goes quiet > ~1 s) and publish zeros to stop."""
        self._wheel_pub.publish(Float32MultiArray(data=[float(fl), float(rl),
                                                        float(fr), float(rr)]))

    def drive_wheels(self, fl, rl, fr, rr, duration):
        """Send one velocity PULSE [FL,RL,FR,RR,duration_s] to /drive_wheels. The
        car runs it with a LOCAL 20 Hz keep-alive for `duration`, then brakes -- so
        WiFi loss can't stall it and a burst can't wedge the serial. Refresh it each
        plan cycle. This is the robust actuation path for the variant-1 (v,w) MPC."""
        self._dwheels_pub.publish(Float32MultiArray(data=[
            float(fl), float(rl), float(fr), float(rr), float(duration)]))

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

    def restart_ros(self):
        """Restart car-ros on the board over SSH -- recover the stack after an
        E-STOP (which kills it). Non-blocking; the stack is back in ~15 s (single
        restart now that the rplidar STOP-before-launch fix is deployed)."""
        subprocess.Popen(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=4",
             "-i", self.ssh_key, "jetson@" + self.car_ip,
             "sudo systemctl restart car-ros"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # -- lifecycle passthroughs (so callers need not import rospy) ---------
    def sleep(self, seconds):
        rospy.sleep(seconds)

    def is_shutdown(self):
        return rospy.is_shutdown()

    def close(self):
        if getattr(self, "_dump_fh", None) is not None:
            try:
                self._dump_fh.close()
            except IOError:
                pass
        rospy.signal_shutdown("carclient closed")
