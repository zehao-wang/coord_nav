#!/usr/bin/env python3
"""PySide6 console for the Jetson mecanum car, over ROS1.

All ROS I/O goes through carclient.CarClient. This client only:
  - shows obstacles: reads /obstacles via CarClient and draws a top-down
    body-frame view (forward = up, car at centre).
  - sends drive commands: CarClient.drive(action, magnitude, duration); one
    click = one discrete step. Completion is reported on /drive_result.
  - E-STOP: CarClient.estop() SSHes estop.sh on the board (hard, ROS-independent).

Run through the ros1 env (ROS + rospy + PySide6 + carclient on the path):
  bash gui/run_gui.sh
"""

import os
import sys
import math
import html
import json
import time
import shutil
import threading
import subprocess
from datetime import datetime

from carclient import CarClient

# MPC baseline package (sibling mpc/ dir) -- policy execute panel.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mpc"))
from mpc_baseline import config as mpc_config
from mpc_baseline.registry import POLICY_REGISTRY, build_policy
from mpc_baseline.runner import PolicyRunner

from PySide6.QtCore import Qt, QObject, Signal, QTimer, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout, QVBoxLayout, QHBoxLayout,
    QFormLayout, QLabel, QPushButton, QPlainTextEdit, QGroupBox, QDoubleSpinBox,
    QCheckBox, QSizePolicy, QComboBox,
)

LOG_DUMP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "car_console.log")
# Each Execute run is saved under output/<YYYY-MM-DD_HH-MM-SS-mmm>/ : the observation
# window recorded as observation.mp4 (~3 Hz), a trajectory.png, and run.json.
RUNS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "output")

DIR_GRID = [  # (row, col, action_id, label) laid out as the 3x3 wheel
    (0, 0, 2, "↖"), (0, 1, 1, "↑"), (0, 2, 8, "↗"),
    (1, 0, 3, "←"), (1, 1, 0, "STOP"),  (1, 2, 7, "→"),
    (2, 0, 4, "↙"), (2, 1, 5, "↓"), (2, 2, 6, "↘"),
]

ACTION_NAMES = {0: "STOP", 1: "forward", 2: "fwd-left", 3: "left",
                4: "back-left", 5: "back", 6: "back-right", 7: "right",
                8: "fwd-right", 9: "rot-CCW", 10: "rot-CW"}


def nearest_edge(circles):
    """Nearest obstacle edge distance in metres, or None if there are none."""
    near = None
    for (x, y, r) in circles:
        d = max(0.0, math.hypot(x, y) - r)
        near = d if near is None else min(near, d)
    return near


# ROS I/O lives in CarClient; this relay hops the /drive_result callback from
# the rospy thread onto the Qt thread (queued signals are thread-safe).
class ResultRelay(QObject):
    result = Signal(object)   # drive_result dict


# Runs an MPC policy (A->B around obstacles) in a background thread so the Qt UI
# stays responsive. All updates come back as queued signals (thread-safe).
POSE_SOURCES = [("motor odom (gyro yaw + encoder xy)", "odom"),
                ("lidar (ICP)", "lidar"),
                ("dead-reckon (model)", "dead_reckon")]


class MPCController(QObject):
    stepped = Signal(dict)      # per-cycle plan info
    finished = Signal(dict)     # run summary
    logmsg = Signal(str, str)   # (kind, msg)

    def __init__(self, client):
        super().__init__()
        self.client = client
        self._runner = None
        self._thread = None

    def start(self, policy_key, magnitude, goal_x, goal_y, pose_source,
              collision_guard, step_duration, allow_rotation, execute_steps=1,
              tick_hz=None, run_dir=None):
        # build the selected policy backend from the registry (any Policy works).
        # build_policy validates the build() signature and that the policy's
        # action_space matches the registry entry -- a mismatch would bind the
        # wrong actuator (velocity -> /drive_wheels, discrete -> /drive_action).
        policy, cfg = build_policy(
            policy_key, magnitude, goal_x, goal_y=goal_y,
            step_duration=step_duration, allow_rotation=allow_rotation)
        live = mpc_config.LiveConfig()
        if tick_hz:                    # the ONE rate: same tick the view runs on
            live.tick.rate_hz = tick_hz
        live.magnitude = magnitude
        live.collision_abort = collision_guard
        live.execute_steps = execute_steps
        self._runner = PolicyRunner(
            policy, cfg, live, mpc_config.ObstacleConfig(), self.client,
            log=lambda m: self.logmsg.emit("SEND", m),
            pose_source=pose_source,
            on_step=lambda d: self.stepped.emit(d),
            collision_estop=False,     # GUI: soft-stop, don't kill car-ros
            tick_log_dir=run_dir)      # tick log lands beside the recording
        self._thread = threading.Thread(target=self._run, name="mpc-run", daemon=True)
        self._thread.start()

    def _run(self):
        try:
            summary = self._runner.run()
        except BaseException as exc:
            self.logmsg.emit("ERR", "MPC error: %s" % exc)
            summary = {"reason": "error: %s" % exc, "reached": False}
        self.finished.emit(summary)

    def stop(self):
        if self._runner is not None:
            self._runner.abort()

    def running(self):
        return self._thread is not None and self._thread.is_alive()


# ===========================================================================
# win0: top-down obstacle view. Forward = up, car at centre.
# ===========================================================================
class ObstacleView(QWidget):
    def __init__(self):
        super().__init__()
        self.frame_id = 0
        self.circles = []
        self.connected = False
        self.goal = None                 # (bx, by) goal B in the base frame, or None
        self.path = None                 # [(bx, by), ...] MPC predicted path, base frame
        self.points = []                 # /obstacle_points, base-frame [(x, y), ...]
        self.have_points = False         # points present for THIS frame_id?
        self.show_points = False         # draw the point cloud (blue)?
        self.point_radius = 0.02         # radius to draw each point (m; GUI-set)
        self.view_range = 3.0            # metres from centre to edge
        self.setMinimumSize(320, 320)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def render_frame(self, frame_id, circles, connected, points=None):
        """One SAMPLE, drawn atomically: `points` are /obstacle_points for this
        exact frame_id (None = not available for it). We never draw points from a
        different frame against these circles -- at 3 Hz publishing off a ~7.7 Hz
        lidar that mismatch is visible as the cloud rotating away from the circles."""
        self.frame_id = frame_id
        self.circles = circles
        self.connected = connected
        self.have_points = points is not None
        self.points = points or []
        self.update()

    def set_show_points(self, on):
        self.show_points = bool(on)
        self.update()

    def set_point_radius(self, r):
        self.point_radius = float(r)
        self.update()

    def set_goal(self, goal_base):
        """goal_base = (bx, by) of B in the base frame, or None to clear."""
        self.goal = goal_base
        self.update()

    def set_path(self, path_base):
        """path_base = [(bx, by), ...] MPC predicted trajectory in the base frame."""
        self.path = path_base
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor(18, 20, 24))
        cx, cy = w / 2.0, h / 2.0
        scale = (min(w, h) / 2.0 - 14) / self.view_range

        # range rings every 0.5 m
        p.setBrush(Qt.NoBrush)
        r = 0.5
        while r <= self.view_range + 1e-6:
            rr = r * scale
            p.setPen(QPen(QColor(55, 60, 68)))
            p.drawEllipse(QPointF(cx, cy), rr, rr)
            p.setPen(QPen(QColor(90, 96, 104)))
            p.drawText(int(cx + 3), int(cy - rr + 13), "%.1fm" % r)
            r += 0.5

        # point cloud (blue) under the circles -- same frame_id as them, or nothing
        if self.show_points and self.points:
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(70, 150, 255, 150))
            pr = max(1.5, self.point_radius * scale)
            for (x, y) in self.points:
                p.drawEllipse(QPointF(cx - y * scale, cy - x * scale), pr, pr)

        # obstacles (red). body x fwd -> screen up; body y left -> screen left
        p.setPen(QPen(QColor(255, 80, 60), 1))
        p.setBrush(QColor(255, 80, 60, 70))
        for (x, y, rad) in self.circles:
            p.drawEllipse(QPointF(cx - y * scale, cy - x * scale),
                          max(2.0, rad * scale), max(2.0, rad * scale))

        # MPC predicted path (yellow polyline), base frame; starts at the car
        if self.path:
            p.setPen(QPen(QColor(240, 210, 60), 2))
            prev = QPointF(cx, cy)
            for (bx, by) in self.path:
                pt = QPointF(cx - by * scale, cy - bx * scale)
                p.drawLine(prev, pt)
                prev = pt

        # goal B (green), base frame -- drawn under the car marker
        if self.goal is not None:
            gpx, gpy = cx - self.goal[1] * scale, cy - self.goal[0] * scale
            p.setPen(QPen(QColor(70, 220, 90), 1, Qt.DashLine))
            p.drawLine(QPointF(cx, cy), QPointF(gpx, gpy))
            p.setPen(QPen(QColor(70, 220, 90), 2))
            p.setBrush(QColor(70, 220, 90, 120))
            p.drawEllipse(QPointF(gpx, gpy), 8, 8)

        # car (blue) + heading line pointing up
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(80, 160, 255))
        p.drawEllipse(QPointF(cx, cy), 6, 6)
        p.setPen(QPen(QColor(80, 160, 255), 2))
        p.drawLine(QPointF(cx, cy), QPointF(cx, cy - 24))

        # axis hint so B (fwd x, left y) is easy to set: +x up (fwd), +y left
        axc = QColor(120, 205, 160)
        L = 42
        p.setPen(QPen(axc, 2))
        p.drawLine(QPointF(cx, cy), QPointF(cx, cy - L))               # +x up
        p.drawLine(QPointF(cx, cy - L), QPointF(cx - 4, cy - L + 8))
        p.drawLine(QPointF(cx, cy - L), QPointF(cx + 4, cy - L + 8))
        p.drawLine(QPointF(cx, cy), QPointF(cx - L, cy))               # +y left
        p.drawLine(QPointF(cx - L, cy), QPointF(cx - L + 8, cy - 4))
        p.drawLine(QPointF(cx - L, cy), QPointF(cx - L + 8, cy + 4))
        p.setFont(QFont("monospace", 9, QFont.Bold))
        p.drawText(int(cx + 6), int(cy - L + 12), "+x fwd")
        p.drawText(int(cx - L), int(cy - 7), "+y left")

        # overlay: frame · count · nearest ; orange when disconnected
        nearest = nearest_edge(self.circles)
        txt = "frame %d   obst %d   nearest %s" % (
            self.frame_id, len(self.circles),
            ("%.2fm" % nearest) if nearest is not None else "--")
        if self.show_points:
            # points are drawn only when they carry this frame_id, so say which
            txt += "   pts %s" % (("%d @frame %d" % (len(self.points), self.frame_id))
                                  if self.have_points else "-- (no /obstacle_points)")
        if not self.connected:
            txt = "! DISCONNECTED   " + txt
        p.setFont(QFont("monospace", 10))
        p.setPen(QColor(255, 150, 40) if not self.connected else QColor(210, 215, 220))
        p.drawText(12, 22, txt)


# ===========================================================================
# Main window
# ===========================================================================
class MainWindow(QMainWindow):
    # Poll this many times per tick and edge-detect the frame_id. Polling exactly
    # at the tick rate would alias against the car's publish instants (sometimes
    # two frames in one poll, sometimes none).
    TICK_OVERSAMPLE = 4

    def __init__(self):
        super().__init__()
        self.client = CarClient(init_node=True)
        self.relay = ResultRelay()
        self.client.on_result(lambda r: self.relay.result.emit(r))
        self.mpc = None                 # MPCController while a policy is running
        self._rec = None                # per-Execute run recorder state, or None
        self.reclog = ResultRelay()     # thread-safe (queued) recorder -> log bridge
        self.reclog.result.connect(lambda m: self.log("SEND", m))
        self.setWindowTitle("Car Console — mecanum / ROS1")
        self.resize(1120, 760)

        # --- widgets ---
        self.move_buttons = []          # steering/rotation buttons (disabled on link loss)
        self.view = ObstacleView()
        self.panel = self._build_panel()
        self.log_widget = QPlainTextEdit(readOnly=True)
        self.log_widget.setMaximumBlockCount(200)              # display cap; file keeps all
        self.log_widget.setLineWrapMode(QPlainTextEdit.NoWrap)  # long lines scroll, no wrap
        self.log_widget.setStyleSheet(
            "background:#141518; color:#cfd3d8; font-family:monospace; font-size:12px;")

        # per-category display filters (the file dump always keeps everything)
        self.filters = {}
        filt_row = QHBoxLayout()
        filt_row.addWidget(QLabel("show:"))
        for kind in ("GET", "SEND", "ERR"):
            cb = QCheckBox(kind)
            cb.setChecked(True)
            self.filters[kind] = cb
            filt_row.addWidget(cb)
        filt_row.addStretch(1)
        win2 = QGroupBox("win2  Log")
        v2 = QVBoxLayout(win2)
        v2.addLayout(filt_row)
        v2.addWidget(self.log_widget)

        # --- layout: top row 2:1, top:bottom 5:1 ---
        central = QWidget()
        grid = QGridLayout(central)
        grid.addWidget(self.view, 0, 0)
        grid.addWidget(self.panel, 0, 1)
        grid.addWidget(win2, 1, 0, 1, 2)
        grid.setColumnStretch(0, 2)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 2)
        grid.setRowStretch(1, 1)
        self.setCentralWidget(central)

        self.relay.result.connect(self._on_result)

        # poll timer: repaint + GET log at the user-set rate, off the latest frame
        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll_tick)
        self._apply_settings()

        QShortcut(QKeySequence(Qt.Key_Space), self, activated=self.on_estop)  # spacebar = E-STOP

        self.log("GET", "console started; master=%s" % os.environ.get("ROS_MASTER_URI", "?"))

    # ---- panel (win1) ----------------------------------------------------
    def _build_panel(self):
        box = QGroupBox("win1  Control Panel")
        v = QVBoxLayout(box)

        conn = QGroupBox("Connection & Params")   # transport is ROS, no IP/port
        form = QFormLayout(conn)
        master = QLabel(os.environ.get("ROS_MASTER_URI", "(unset)"))
        master.setStyleSheet("color:#8a9098;")
        # THE global tick rate. Must equal the car's perception rate (obstacle_circles
        # rate_hz in viz.launch) -- the runner and this view both advance one frame per
        # tick, so a different number here would just alias against the real clock.
        self.hz_spin = self._spin(0.5, 10.0, 0.5, mpc_config.TickConfig.rate_hz)
        # floor 20: below ~17.4 the planner's achievable yaw is 0 (build_live_cfg
        # raises), and 40 is the validated value.
        # floor 20 (below ~17.4 the planner's achievable yaw is 0 and build_live_cfg
        # raises); default 30 = LiveConfig.magnitude, the best-scoring conservative
        # value. 40 also works and is what the five 5 m on-car runs used.
        self.mag_spin = self._spin(20.0, 80.0, 5.0, mpc_config.LiveConfig.magnitude)
        self.dur_spin = self._spin(0.1, 3.0, 0.1, 0.5)      # step move = exact run time, no compensation
        self.diag_spin = self._spin(1.0, 4.0, 0.1, 1.6)     # diagonal magnitude multiplier
        self.strafe_spin = self._spin(1.0, 4.0, 0.1, 1.2)   # strafe magnitude multiplier
        form.addRow("ROS master", master)
        form.addRow("tick Hz (global)", self.hz_spin)
        form.addRow("magnitude (x)", self.mag_spin)
        form.addRow("step move(s)", self.dur_spin)
        form.addRow("diag mult", self.diag_spin)
        form.addRow("strafe mult", self.strafe_spin)
        apply_btn = QPushButton("apply")
        apply_btn.clicked.connect(self._apply_settings)
        form.addRow(apply_btn)
        v.addWidget(conn)

        # MCU link / battery health banner (updated each poll)
        self.link_lbl = QLabel("MCU: --")
        self.link_lbl.setAlignment(Qt.AlignCenter)
        self.link_lbl.setStyleSheet("padding:5px; font-weight:bold; background:#333; color:#ccc;")
        v.addWidget(self.link_lbl)

        # point cloud: /obstacle_points drawn (blue) in the top-down view. These are
        # the exact points the circles of the SAME frame_id were clustered from, so
        # cloud and circles are one sample (/scan would not be -- see carclient).
        pc = QGroupBox("Point cloud  (/obstacle_points, frame-synced)")
        pcl = QHBoxLayout(pc)
        self.pc_cb = QCheckBox("show (blue)")
        self.pc_cb.setChecked(False)
        self.pc_cb.toggled.connect(self.view.set_show_points)
        pcl.addWidget(self.pc_cb)
        pcl.addWidget(QLabel("radius m"))
        self.pc_radius = self._spin(0.005, 0.30, 0.005, 0.02)
        self.pc_radius.valueChanged.connect(self.view.set_point_radius)
        pcl.addWidget(self.pc_radius)
        pcl.addStretch(1)
        v.addWidget(pc)

        wheel = QGroupBox("Steering  (one click = one step)")
        g = QGridLayout(wheel)
        for (rw, cl, aid, lab) in DIR_GRID:
            b = QPushButton(lab)
            b.setMinimumSize(64, 52)
            # movement keys blue; the soft STOP (centre) stays a neutral grey
            b.setStyleSheet("font-size:18px; color:#eef2f6; " + (
                "background:#3a3f47;" if aid == 0 else "background:#2f5f9e;"))
            b.clicked.connect(lambda _=False, a=aid: self.on_action(a))
            if aid != 0:
                self.move_buttons.append(b)     # STOP stays enabled; directions gated
            g.addWidget(b, rw, cl)
        v.addWidget(wheel)

        rot = QHBoxLayout()
        for aid, lab in [(9, "↺"), (10, "↻")]:
            b = QPushButton(lab)
            b.setMinimumHeight(46)
            b.setStyleSheet("font-size:26px; color:#eef2f6; background:#2f5f9e;")
            b.setToolTip("rotate CCW" if aid == 9 else "rotate CW")
            b.clicked.connect(lambda _=False, a=aid: self.on_action(a))
            self.move_buttons.append(b)
            rot.addWidget(b)
        v.addLayout(rot)

        # --- Policy (A->B, MPC) ------------------------------------------
        pol = QGroupBox("Policy  (A->B, MPC)")
        pf = QFormLayout(pol)
        self.policy_combo = QComboBox()      # backends from the registry (switchable)
        for key, spec in POLICY_REGISTRY.items():
            self.policy_combo.addItem(spec["label"], key)
        # Default: continuous velocity-control policy (variant 1) -- validated go-around baseline.
        _vw = self.policy_combo.findData("mpc_vw")
        if _vw >= 0:
            self.policy_combo.setCurrentIndex(_vw)
        # B relative to start pose: forward x, left y (metres). Default = 3 m ahead.
        self.goal_x_spin = self._spin(-10.0, 10.0, 0.1, 3.0)
        self.goal_y_spin = self._spin(-10.0, 10.0, 0.1, 0.0)
        self.odom_combo = QComboBox()
        for label, val in POSE_SOURCES:                        # default = motor odom
            self.odom_combo.addItem(label, val)
        self.collision_cb = QCheckBox("collision guard (soft-stop)")
        # It USED to be OFF: obstacle circles already carry margin + planner inflates, so
        # the guard looked redundant and false-tripped mid-go-around. E-STOP is the backstop.
        # ON by default. It was off, and run output/2026-07-25_20-01-44 drove 126 mm
        # of the 130 mm footprint into an obstacle at v_max for three ticks (1 s)
        # with nothing intervening -- the guard would have fired on all three.
        # collision_estop=False below makes this a SOFT stop, so a false trip costs
        # a stop, not a killed car-ros.
        self.collision_cb.setChecked(True)
        # planned steps to apply before re-planning (1 = tight closed loop)
        self.exec_steps_spin = self._spin(1.0, 10.0, 1.0, 1.0)
        self.exec_steps_spin.setDecimals(0)
        pf.addRow("policy", self.policy_combo)
        pf.addRow("B forward x (m)", self.goal_x_spin)
        pf.addRow("B left y (m)", self.goal_y_spin)
        pf.addRow("pose source", self.odom_combo)
        pf.addRow("execute steps", self.exec_steps_spin)
        pf.addRow(self.collision_cb)
        row = QHBoxLayout()
        self.exec_btn = QPushButton("Execute")
        self.exec_btn.setMinimumHeight(40)
        self.exec_btn.setStyleSheet("background:#1f8b4c; color:white; font-size:15px; font-weight:bold;")
        self.exec_btn.clicked.connect(self.on_execute)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setMinimumHeight(40)
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.on_stop_policy)
        row.addWidget(self.exec_btn)
        row.addWidget(self.stop_btn)
        pf.addRow(row)
        self.policy_status = QLabel("idle")
        self.policy_status.setStyleSheet("color:#8a9098;")
        pf.addRow("status", self.policy_status)
        v.addWidget(pol)

        estop = QPushButton("E-STOP  (hard kill)")
        estop.setMinimumHeight(60)
        estop.setStyleSheet(
            "background:#c01818; color:white; font-size:18px; font-weight:bold;")
        estop.clicked.connect(self.on_estop)
        v.addWidget(estop)

        self.restart_btn = QPushButton("Restart car-ros  (recover after E-STOP)")
        self.restart_btn.setMinimumHeight(38)
        self.restart_btn.setStyleSheet("background:#2a6f97; color:white; font-weight:bold;")
        self.restart_btn.clicked.connect(self.on_restart)
        v.addWidget(self.restart_btn)

        v.addStretch(1)
        return box

    def _update_mag_cap(self):
        # base magnitude x (largest multiplier) must not exceed 80 (car's cap)
        m = max(self.diag_spin.value(), self.strafe_spin.value())
        self.mag_spin.setMaximum(80.0 / max(m, 0.1))
        self.client.diag_mult = self.diag_spin.value()
        self.client.strafe_mult = self.strafe_spin.value()

    @staticmethod
    def _spin(lo, hi, step, val):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setValue(val)
        return s

    # ---- settings --------------------------------------------------------
    def _apply_settings(self):
        # diag mult is a calibration value -> only takes effect on apply, and the
        # magnitude ceiling (= 80 / diag_mult) is recomputed here too.
        hz = self.hz_spin.value()
        # Poll several times per tick and edge-detect the frame_id, so a tick is
        # picked up promptly however the car's publish instants drift against us.
        self.poll_timer.start(max(20, int(1000.0 / (hz * self.TICK_OVERSAMPLE))))
        self._update_mag_cap()
        self.log("SEND", "apply: tick=%.1fHz mag=%.0f move=%.2fs diag=%.1fx strafe=%.1fx"
                 % (hz, self.mag_spin.value(), self.dur_spin.value(),
                    self.diag_spin.value(), self.strafe_spin.value()))

    # ---- handlers --------------------------------------------------------
    def _on_result(self, r):
        # move completion belongs to the command lifecycle -> SEND, not GET
        self.log("SEND", "done: %s (id=%s) took=%sms"
                 % (r.get("reason"), r.get("action"), r.get("took_ms")))

    def _poll_tick(self):
        """Runs on the GLOBAL tick: the timer polls faster than the tick, but the
        observation/render/record work happens once per NEW frame_id -- the same
        tick boundary PolicyRunner advances on. A free-running refresh would either
        redraw frames it had already drawn or skip frames outright."""
        self._update_link()
        # ONE sample: circles + the points they were clustered from, same frame_id
        obs = self.client.observation()
        if obs is None:
            fid, circ, connected, pts = 0, [], False, None
        else:
            fid, circ, connected = obs.frame_id, obs.circles, obs.age < 1.5
            pts = obs.points
        if obs is not None and fid == getattr(self, "_last_fid", None):
            return                        # same tick as last poll: nothing new to do
        self._last_fid = fid
        # report obstacle disconnect/reconnect transitions (disconnect -> ERR)
        if getattr(self, "_obs_conn", True) != connected:
            self._obs_conn = connected
            self.log("ERR" if not connected else "GET",
                     "obstacles DISCONNECTED" if not connected else "obstacles reconnected")
        # Always hand over the frame's points; the view's show_points flag decides
        # whether to draw them. Gating here too would double-gate AND blank the
        # cloud for one poll period every time the checkbox is ticked.
        self.view.render_frame(fid, circ, connected, points=pts)
        self._rec_grab(fid)               # capture one video frame per new observation
        # circles are [x,y,r] in metres, base frame
        near = nearest_edge(circ)
        data = " ".join("(%.2f,%.2f,%.2f)" % c for c in circ)
        self.log("GET", "frame=%d obst=%d nearest=%s%s  data=[%s]"
                 % (fid, len(circ), ("%.2fm" % near) if near is not None else "--",
                    "" if connected else "  [DISCONNECTED]", data))

    def _update_link(self):
        # MCU link health from /battery (same serial as motors): 0 V / no data
        # => link wedged, motor commands may be lost. Warn + gate the wheel.
        v = self.client.battery()
        ok = self.client.link_ok()
        if v is None:
            self.link_lbl.setText("MCU LINK: no data — USE E-STOP")
            self.link_lbl.setStyleSheet("padding:5px; font-weight:bold; background:#c01818; color:white;")
        elif ok:
            self.link_lbl.setText("MCU link OK   %.1f V" % v)
            self.link_lbl.setStyleSheet("padding:5px; font-weight:bold; background:#204d24; color:#8ee69a;")
        else:
            self.link_lbl.setText("⚠ MCU LINK LOST (%.1f V) — USE E-STOP" % v)
            self.link_lbl.setStyleSheet("padding:5px; font-weight:bold; background:#c01818; color:white;")
        if getattr(self, "_link_ok", None) != ok:
            self._link_ok = ok
            running = self.mpc is not None and self.mpc.running()
            # never re-enable manual drive while a policy is driving (it would fight
            # the MPC's /wheel_cmd on the real robot) -- gate on link AND not-running
            for b in self.move_buttons:
                b.setEnabled(ok and not running)
            if hasattr(self, "exec_btn") and not running:
                self.exec_btn.setEnabled(ok)   # gate Execute too, unless mid-run
            self.log("ERR" if not ok else "SEND",
                     "MCU link %s" % ("LOST — drive disabled" if not ok else "OK — drive enabled"))

    def on_action(self, aid):
        # while a policy drives, the manual pad must not send /drive_action (it
        # conflicts with the MPC); route STOP to aborting the policy, ignore the rest
        if self.mpc is not None and self.mpc.running():
            if aid == 0:
                self.on_stop_policy()
            return
        mag = self.mag_spin.value()
        dur = self.dur_spin.value()
        self.client.drive(aid, mag, dur)
        if aid == 0:
            self.log("SEND", "STOP")
        else:
            self.log("SEND", "%s (id=%d) mag=%.0f move=%.2fs"
                     % (ACTION_NAMES.get(aid, "?"), aid, mag, dur))

    def on_estop(self):
        self.log("ERR", "E-STOP! estop.sh -> car-ros will be killed")
        self.client.estop()

    def on_restart(self):
        # car-ros restart restarts roscore too; rospy can't reconnect to a new
        # master, so relaunch a fresh GUI process once the stack is back (~15 s).
        if self.mpc is not None and self.mpc.running():
            # abort first: else the still-running runner sees /battery_v vanish
            # during the restart, fires its link-loss E-STOP, and kills the car-ros
            # we just restarted.
            self.mpc.stop()
            self.log("SEND", "aborting running policy before car-ros restart")
        self.log("SEND", "restart car-ros -- recovering (~20s), GUI will relaunch to reconnect")
        self.client.restart_ros()
        QTimer.singleShot(20000, self._relaunch_self)

    def _relaunch_self(self):
        run = os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_gui.sh")
        subprocess.Popen(["bash", run], stdin=subprocess.DEVNULL,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True, env=os.environ.copy())
        self.close()

    # ---- policy (A->B, MPC) ---------------------------------------------
    def on_execute(self):
        if getattr(self, "mpc", None) is not None and self.mpc.running():
            return
        if not self.client.link_ok():
            self.log("ERR", "MCU link not healthy -- not starting policy")
            return
        policy_key = self.policy_combo.currentData()
        gx, gy = self.goal_x_spin.value(), self.goal_y_spin.value()
        self.mpc = MPCController(self.client)
        self.mpc.stepped.connect(self._on_mpc_step)
        self.mpc.finished.connect(self._on_mpc_finished)
        self.mpc.logmsg.connect(self.log)
        self._set_policy_running(True)
        self.log("SEND", "EXECUTE %s mag=%.0f B=(fwd %.1f, left %.1f) pose=%s guard=%s"
                 % (policy_key, self.mag_spin.value(), gx, gy,
                    self.odom_combo.currentData(), self.collision_cb.isChecked()))
        self._rec_start(policy_key, gx, gy)     # record this run (observation video + plot)
        try:
            self.mpc.start(policy_key, self.mag_spin.value(), gx, gy,
                           self.odom_combo.currentData(), self.collision_cb.isChecked(),
                           self.dur_spin.value(), False,
                           int(self.exec_steps_spin.value()),
                           tick_hz=self.hz_spin.value(),
                           run_dir=(self._rec or {}).get("dir"))
        except Exception as exc:
            # policy build / runner construction runs synchronously here; if it
            # raises, the worker thread never starts and 'finished' never fires --
            # so recover the UI right here instead of wedging it.
            self.log("ERR", "policy start failed: %s" % exc)
            if self._rec is not None:
                shutil.rmtree(self._rec["dir"], ignore_errors=True)
            self._rec = None
            self._set_policy_running(False)

    def on_stop_policy(self):
        if getattr(self, "mpc", None) is not None:
            self.log("SEND", "STOP policy")
            self.mpc.stop()

    def _on_mpc_step(self, d):
        # transform B and the predicted path from the planning frame into the
        # current base frame (forward=up) for the top-down view
        px, py, pth = d["pose"]
        c, s = math.cos(pth), math.sin(pth)

        def to_base(wx, wy):
            dx, dy = wx - px, wy - py
            return (c * dx + s * dy, -s * dx + c * dy)

        self.view.set_goal(to_base(d["goal"][0], d["goal"][1]))
        traj = d.get("traj")
        self.view.set_path([to_base(pt[0], pt[1]) for pt in traj] if traj else None)
        act = d.get("action")
        extra = ("act=%s" % ACTION_NAMES.get(act, act)) if act is not None else \
                ("v=%.2f w=%+.2f" % (d.get("v") or 0.0, d.get("w") or 0.0))
        self.policy_status.setText("gd=%.2fm  %s" % (d["gd"], extra))
        self._rec_step(d)

    def _on_mpc_finished(self, summary):
        self._set_policy_running(False)
        self._rec_finish(summary)
        self.view.set_goal(None)
        self.view.set_path(None)
        self.policy_status.setText("done: %s (gd=%s)" % (
            summary.get("reason"), summary.get("final_goal_dist")))
        self.log("SEND", "policy DONE %s" % summary)

    def _set_policy_running(self, running):
        link = getattr(self, "_link_ok", True)
        self.exec_btn.setEnabled((not running) and link)   # keep link-gating too
        self.stop_btn.setEnabled(running)
        self.policy_combo.setEnabled(not running)
        self.goal_x_spin.setEnabled(not running)
        self.goal_y_spin.setEnabled(not running)
        self.odom_combo.setEnabled(not running)
        self.exec_steps_spin.setEnabled(not running)
        self.restart_btn.setEnabled(not running)           # don't restart car-ros mid-run
        for b in self.move_buttons:       # no manual steering while a policy drives
            b.setEnabled((not running) and link)

    # ---- per-run recorder: observation video (~3 Hz) + trajectory plot ---
    def _rec_start(self, policy_key, gx, gy):
        try:
            # millisecond precision so two Executes in the same second get distinct
            # dirs (else one run's encode thread would rmtree the other's frames)
            ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")[:-3]
            run_dir = os.path.join(RUNS_DIR, ts)
            frames_dir = os.path.join(run_dir, "_frames")
            os.makedirs(frames_dir, exist_ok=True)
            self._rec = {"dir": run_dir, "frames": frames_dir, "n": 0,
                         "last_fid": None, "steps": [], "ts": ts,
                         "policy": policy_key, "goal": [gx, gy]}
        except OSError as e:
            self._rec = None
            self.log("ERR", "could not start recording: %s" % e)

    def _rec_grab(self, fid):
        # one frame per NEW observation (perception ~3 Hz) -> the video is 3 fps
        rec = self._rec
        if rec is None or fid == rec["last_fid"]:
            return
        rec["last_fid"] = fid
        try:
            path = os.path.join(rec["frames"], "f%05d.png" % rec["n"])
            self.view.grab().save(path, "PNG")
            rec["n"] += 1
        except Exception:
            pass

    def _rec_step(self, d):
        rec = self._rec
        if rec is None:
            return
        o = self.client.obstacles()
        rec["steps"].append({
            "pose": d.get("pose"), "goal": d.get("goal"), "gd": d.get("gd"),
            "v": d.get("v"), "w": d.get("w"), "action": d.get("action"),
            "traj": d.get("traj"),
            "circles": list(o.circles) if o else [], "t": time.time()})

    def _rec_finish(self, summary):
        rec = self._rec
        self._rec = None
        if rec is None:
            return
        if rec["n"] == 0:                       # nothing captured -> don't leave an empty dir
            shutil.rmtree(rec["dir"], ignore_errors=True)
            return
        # encode off the UI thread (ffmpeg + matplotlib can take a second)
        threading.Thread(target=self._encode_run, args=(rec, dict(summary)),
                         name="rec-encode", daemon=True).start()

    def _encode_run(self, rec, summary):
        run_dir = rec["dir"]
        try:
            mp4 = os.path.join(run_dir, "observation.mp4")
            if rec["n"] >= 2 and shutil.which("ffmpeg"):
                subprocess.run(
                    ["ffmpeg", "-y", "-framerate", "3",
                     "-i", os.path.join(rec["frames"], "f%05d.png"),
                     "-pix_fmt", "yuv420p",
                     "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", mp4],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=90)
            json.dump({"summary": summary, "policy": rec["policy"],
                       "goal": rec["goal"], "steps": rec["steps"]},
                      open(os.path.join(run_dir, "run.json"), "w"))
            self._plot_run(run_dir, rec["steps"], summary)
            shutil.rmtree(rec["frames"], ignore_errors=True)
            self.reclog.result.emit("saved run -> %s (video %d frames, reached=%s)"
                                    % (run_dir, rec["n"], summary.get("reached")))
        except Exception as e:
            self.reclog.result.emit("run save FAILED: %s" % e)

    @staticmethod
    def _plot_run(run_dir, steps, summary):
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        P = np.array([s["pose"] for s in steps if s.get("pose")])
        if len(P) < 2:
            return
        x0, y0, th0 = P[0]
        c0, s0 = np.cos(-th0), np.sin(-th0)

        def to_start(x, y):
            dx, dy = x - x0, y - y0
            return np.array([c0 * dx - s0 * dy, s0 * dx + c0 * dy])

        traj = np.array([to_start(p[0], p[1]) for p in P])
        goal = to_start(*steps[0]["goal"])
        near = []
        for s in steps:
            if not s.get("pose"):
                continue
            rp = to_start(s["pose"][0], s["pose"][1])
            rth = s["pose"][2] - th0
            cc, ss = np.cos(rth), np.sin(rth)
            for bx, by, br in s.get("circles", []):
                q = rp + np.array([cc * bx - ss * by, ss * bx + cc * by])
                if np.min(np.hypot(traj[:, 0] - q[0], traj[:, 1] - q[1])) < 1.2:
                    near.append((q[0], q[1], br))
        plen = float(np.sum(np.hypot(np.diff(traj[:, 0]), np.diff(traj[:, 1]))))
        fig, a = plt.subplots(figsize=(7, 6))
        for x, y, rr in near:
            a.add_patch(plt.Circle((x, y), rr, color="tab:red", alpha=0.08))
        a.plot(traj[:, 0], traj[:, 1], "-o", ms=3, color="tab:blue", lw=2, label="car path")
        a.plot(0, 0, "ks", ms=10, label="start A")
        a.plot(goal[0], goal[1], "g*", ms=22, label="goal B")
        if traj[:, 0].ptp() < 0.2 or traj[:, 1].ptp() < 0.2:
            a.margins(0.25)
        else:
            a.set_aspect("equal")
        a.grid(alpha=0.3); a.legend(loc="upper left")
        a.set_xlabel("x forward (m)"); a.set_ylabel("y left (m)")
        a.set_title("reached=%s  path=%.2fm  final_gd=%s" % (
            summary.get("reached"), plen, summary.get("final_goal_dist")))
        fig.savefig(os.path.join(run_dir, "trajectory.png"), dpi=115, bbox_inches="tight")
        plt.close(fig)

    # ---- logging ---------------------------------------------------------
    def log(self, kind, msg):
        colors = {"GET": "#e8a33d", "SEND": "#4caf50", "ERR": "#e04030"}
        line = "[%s] %-4s %s" % (datetime.now().strftime("%H:%M:%S.%f")[:-3], kind, msg)
        try:
            # rotate at 5 MB so the dump file can't grow without bound
            if os.path.exists(LOG_DUMP) and os.path.getsize(LOG_DUMP) > 5 * 1024 * 1024:
                os.replace(LOG_DUMP, LOG_DUMP + ".1")
            with open(LOG_DUMP, "a") as fh:
                fh.write(line + "\n")
        except OSError:
            pass
        # file keeps everything; only show categories whose box is checked
        cb = self.filters.get(kind)
        if cb is not None and not cb.isChecked():
            return
        self.log_widget.appendHtml(
            '<span style="color:%s">%s</span>' % (colors.get(kind, "#ccc"), html.escape(line)))

    def closeEvent(self, e):
        if self.mpc is not None and self.mpc.running():
            self.mpc.stop()               # stop a running policy before shutdown
        self.client.close()
        e.accept()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
