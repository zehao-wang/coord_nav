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
from datetime import datetime

from carclient import CarClient

from PySide6.QtCore import Qt, QObject, Signal, QTimer, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout, QVBoxLayout, QHBoxLayout,
    QFormLayout, QLabel, QPushButton, QPlainTextEdit, QGroupBox, QDoubleSpinBox,
    QCheckBox, QSizePolicy,
)

LOG_DUMP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "car_console.log")

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


# ===========================================================================
# win0: top-down obstacle view. Forward = up, car at centre.
# ===========================================================================
class ObstacleView(QWidget):
    def __init__(self):
        super().__init__()
        self.frame_id = 0
        self.circles = []
        self.connected = False
        self.view_range = 3.0            # metres from centre to edge
        self.setMinimumSize(320, 320)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def render_frame(self, frame_id, circles, connected):
        self.frame_id = frame_id
        self.circles = circles
        self.connected = connected
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

        # obstacles (red). body x fwd -> screen up; body y left -> screen left
        p.setPen(QPen(QColor(255, 80, 60), 1))
        p.setBrush(QColor(255, 80, 60, 70))
        for (x, y, rad) in self.circles:
            p.drawEllipse(QPointF(cx - y * scale, cy - x * scale),
                          max(2.0, rad * scale), max(2.0, rad * scale))

        # car (blue) + heading line pointing up
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(80, 160, 255))
        p.drawEllipse(QPointF(cx, cy), 6, 6)
        p.setPen(QPen(QColor(80, 160, 255), 2))
        p.drawLine(QPointF(cx, cy), QPointF(cx, cy - 24))

        # overlay: frame · count · nearest ; orange when disconnected
        nearest = nearest_edge(self.circles)
        txt = "frame %d   obst %d   nearest %s" % (
            self.frame_id, len(self.circles),
            ("%.2fm" % nearest) if nearest is not None else "--")
        if not self.connected:
            txt = "! DISCONNECTED   " + txt
        p.setFont(QFont("monospace", 10))
        p.setPen(QColor(255, 150, 40) if not self.connected else QColor(210, 215, 220))
        p.drawText(12, 22, txt)


# ===========================================================================
# Main window
# ===========================================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.client = CarClient(init_node=True)
        self.relay = ResultRelay()
        self.client.on_result(lambda r: self.relay.result.emit(r))
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
        grid.setRowStretch(0, 2)     # win2 (log) doubled: 1/6 -> 1/3 of height
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
        self.hz_spin = self._spin(0.5, 3.0, 0.5, 3.0)     # capped at 3 Hz (read rate)
        self.mag_spin = self._spin(0.0, 80.0, 5.0, 40.0)    # base magnitude (cap 80)
        self.dur_spin = self._spin(0.1, 3.0, 0.1, 0.8)      # step move = exact run time, no compensation
        self.diag_spin = self._spin(1.0, 4.0, 0.1, 2.0)     # diagonal magnitude multiplier
        form.addRow("ROS master", master)
        form.addRow("refresh Hz", self.hz_spin)
        form.addRow("magnitude (x)", self.mag_spin)
        form.addRow("step move(s)", self.dur_spin)
        form.addRow("diag mult", self.diag_spin)
        apply_btn = QPushButton("apply")
        apply_btn.clicked.connect(self._apply_settings)
        form.addRow(apply_btn)
        v.addWidget(conn)

        # MCU link / battery health banner (updated each poll)
        self.link_lbl = QLabel("MCU: --")
        self.link_lbl.setAlignment(Qt.AlignCenter)
        self.link_lbl.setStyleSheet("padding:5px; font-weight:bold; background:#333; color:#ccc;")
        v.addWidget(self.link_lbl)

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

        estop = QPushButton("E-STOP  (hard kill)")
        estop.setMinimumHeight(60)
        estop.setStyleSheet(
            "background:#c01818; color:white; font-size:18px; font-weight:bold;")
        estop.clicked.connect(self.on_estop)
        v.addWidget(estop)
        v.addStretch(1)
        return box

    def _update_mag_cap(self):
        # base magnitude x diag_mult must not exceed 80 (car's applied cap);
        # keep the live multiplier in sync so the cap and behaviour always match
        self.mag_spin.setMaximum(80.0 / max(self.diag_spin.value(), 0.1))
        self.client.diag_mult = self.diag_spin.value()

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
        self.poll_timer.start(int(1000.0 / hz))
        self._update_mag_cap()
        self.log("SEND", "apply: refresh=%.1fHz mag=%.0f move=%.2fs diag=%.1fx"
                 % (hz, self.mag_spin.value(), self.dur_spin.value(), self.diag_spin.value()))

    # ---- handlers --------------------------------------------------------
    def _on_result(self, r):
        # move completion belongs to the command lifecycle -> SEND, not GET
        self.log("SEND", "done: %s (id=%s) took=%sms"
                 % (r.get("reason"), r.get("action"), r.get("took_ms")))

    def _poll_tick(self):
        self._update_link()
        obs = self.client.obstacles()
        if obs is None:
            fid, circ, connected = 0, [], False
        else:
            fid, circ, connected = obs.frame_id, obs.circles, obs.age < 1.5
        # report obstacle disconnect/reconnect transitions (disconnect -> ERR)
        if getattr(self, "_obs_conn", True) != connected:
            self._obs_conn = connected
            self.log("ERR" if not connected else "GET",
                     "obstacles DISCONNECTED" if not connected else "obstacles reconnected")
        self.view.render_frame(fid, circ, connected)
        # summary + the actual returned circles [x,y,r] (metres, base frame).
        # log() always dumps to file and applies the GET display filter.
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
            for b in self.move_buttons:
                b.setEnabled(ok)
            self.log("ERR" if not ok else "SEND",
                     "MCU link %s" % ("LOST — drive disabled" if not ok else "OK — drive enabled"))

    def on_action(self, aid):
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
        self.client.close()
        e.accept()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
