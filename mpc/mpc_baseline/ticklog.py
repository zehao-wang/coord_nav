"""Per-tick debug log -- one dedicated file per experiment, named by start time.

The point: someone who did NOT write the policy should be able to open this file
and see what happened on every tick, without re-running anything.

One line per tick, fixed columns so the eye can scan down one. Every line also
goes to a sibling .jsonl for plotting/greping. The FLAGS column is the thing to
skim: a healthy tick shows `.`, anything else is a named anomaly, so a 500-line
file reveals its problem without reading it.

    tick   t      dt     frame        pose                gd     obs        DISPATCH        PLAN->next      timing      FLAGS
    0007   2.17   0.334  1183         +0.31 +0.02  -12.4  0.291  76 n0.84   vel v0.200 w+0.39  v0.200 w+0.31  w6 p7 W23  .
    0008   2.50   0.327  1184         +0.38 +0.01  -21.7  0.214  76 n0.79   vel v0.200 w-0.67  v0.110 w-0.99  w5 p7 W21  .
    0009   2.84   0.340  1185         +0.44 -0.03  -35.1  0.190  77 n0.71   vel v0.110 w-0.99  v0.200 w-1.20  w6 p8 W22  WPIN
    0010   3.23   0.390  1186         +0.47 -0.09  -51.9  0.204  74 n0.66   vel v0.200 w-1.20  v0.106 w-0.96  w6 p9 W24  WPIN AWAY

Flags (see FLAGS below for the full list): WPIN = yaw command pinned at the cap,
AWAY = distance to B grew, ZERO = commanded a full stop, SKIP = perception frames
were missed, OVERRUN = the tick's work did not fit in the tick, HOLD = stale data,
NODISP = nothing was dispatched, NOPTS = /obstacle_points did not pair this frame.

Written from the control loop at 3 Hz. Formatting a line costs ~30 us against a
333 ms tick, and each line is flushed immediately so an E-STOP, a KeyboardInterrupt
or a crash still leaves a complete file -- those are exactly the runs worth reading.
"""

import os
import json
import time
from datetime import datetime

# where a run's files go when the caller does not supply a directory
DEFAULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "..", "output")

HEADER_COLS = ("tick   t      dt     frame     pose(fwd   left  yaw)   gd     "
               "obs/mem      DISPATCH             PLAN->next           real/cmd     "
               "timing        FLAGS")


class TickLog(object):
    """One file per experiment. `run_dir` is the directory to write into (the GUI
    already makes output/<timestamp>/ per Execute and passes it); when omitted a
    fresh output/<timestamp>/ is created so the CLIs behave the same way.

    Both files are named by the experiment START time, so a run is identifiable
    from the filename alone: tick_<YYYY-MM-DD_HH-MM-SS>.log / .jsonl
    """

    def __init__(self, run_dir=None, stamp=None, meta=None, enabled=True):
        self.enabled = enabled
        self.fh = self.jfh = None
        self.n = 0
        self.flag_counts = {}
        self.t0 = time.monotonic()
        if not enabled:
            return
        self.stamp = stamp or datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        run_dir = run_dir or os.path.join(DEFAULT_DIR, self.stamp)
        try:
            os.makedirs(run_dir, exist_ok=True)
            base = os.path.join(run_dir, "tick_%s" % self.stamp)
            self.fh = open(base + ".log", "w")
            self.jfh = open(base + ".jsonl", "w")
            self.path = os.path.normpath(base + ".log")
        except OSError:
            self.enabled = False       # logging must never take the run down
            return
        self._header(meta or {})

    # -- header / footer ---------------------------------------------------
    def _header(self, meta):
        w = self.fh.write
        w("# tick log  %s\n" % self.stamp)
        w("#\n")
        for k in sorted(meta):
            w("# %-18s %s\n" % (k, meta[k]))
        w("#\n")
        w("# units: metres, degrees for yaw (x fwd, y left, CCW+), m/s, rad/s, PWM\n")
        w("# t/dt seconds since run start / since previous tick.  gd = distance to B.\n")
        w("# obs/mem 'N nD mM': N circles THIS FRAME, nearest edge D m -- what the\n")
        w("#            COLLISION GUARD sees, since it reads this frame only -- and M m\n")
        w("#            to the nearest REMEMBERED obstacle, which is what the POLICY\n")
        w("#            plans against. D=-- with a small M means the guard cannot fire\n")
        w("#            on an obstacle the policy can see.\n")
        w("# real/cmd 'xA wB': measured / commanded motion this tick; 1.00 = the model\n")
        w("#            matches the car. Blank under pose_source=dead_reckon, where the\n")
        w("#            pose IS the integrated command and the ratio is a tautology.\n")
        w("# DISPATCH = what was SENT this tick (decided one tick earlier, and it\n")
        w("#            supersedes whatever the car was still running).\n")
        w("# PLAN->next = what this tick's observation produced, to be sent NEXT tick.\n")
        w("# timing 'wA pB WC' = A ms blocked waiting for the frame, B ms planning,\n")
        w("#            C ms of total in-tick work. Tick budget is one period.\n")
        w("#\n")
        w("# FLAGS   . = nothing notable\n")
        w("#   SKIP    perception frames were missed between this tick and the last\n")
        w("#   OVERRUN in-tick work exceeded the tick period (the next tick catches up)\n")
        w("#   HOLD    obstacles/pose too stale to act on: the car was stopped\n")
        w("#   NODISP  nothing was dispatched (buffer empty; car keeps its last command)\n")
        w("#   NOPTS   /obstacle_points did not pair with this frame_id\n")
        w("#   ZERO    commanded a full stop while not at B (the 'car froze' failure)\n")
        w("#   WPIN    yaw command pinned at +-w_max (usually means it cannot turn enough)\n")
        w("#   AWAY    distance to B grew vs the previous tick\n")
        w("#   NEAR    an obstacle edge is within one robot radius of the footprint\n")
        w("#   SAFETY  a safety interlock fired (see the line text)\n")
        w("#   NOFRAME no observation frame arrived in time (the car was stopped)\n")
        w("#   STALE / COLLIDE / LINKLOST / ABORT / TIMEOUT / REACHED: the tick took that\n")
        w("#            branch and ended there. EVERY tick emits exactly one line, so a\n")
        w("#            gap in the numbering means the loop stopped -- never that a\n")
        w("#            branch forgot to log.\n")
        w("#\n")
        w(HEADER_COLS + "\n")
        self.fh.flush()

    def close(self, summary=None, extra=None):
        """Footer: the verdict plus the counts that say whether the loop was healthy."""
        if not self.enabled or self.fh is None:
            return None
        try:
            w = self.fh.write
            w("\n# ---- summary ----\n")
            if summary:
                for k in sorted(summary):
                    w("# %-18s %s\n" % (k, summary[k]))
            w("# %-18s %d\n" % ("ticks", self.n))
            w("# %-18s %.2f s\n" % ("wall", time.monotonic() - self.t0))
            if self.flag_counts:
                w("# %-18s %s\n" % ("flags", "  ".join(
                    "%s=%d" % (k, v) for k, v in sorted(self.flag_counts.items()))))
            else:
                w("# %-18s (none)\n" % "flags")
            for k in sorted(extra or {}):
                w("# %-18s %s\n" % (k, extra[k]))
            self.fh.flush()
            path = self.path
        finally:
            for fh in (self.fh, self.jfh):
                try:
                    fh.close()
                except Exception:
                    pass
            self.fh = self.jfh = None
        return path

    # -- one tick ----------------------------------------------------------
    def tick(self, rec):
        """`rec` is a plain dict; see PolicyRunner for the field names. Missing
        fields render as '-' rather than raising -- a log must never break a run."""
        if not self.enabled or self.fh is None:
            return
        self.n += 1
        flags = rec.get("flags") or ["."]
        for f in flags:
            if f != ".":
                self.flag_counts[f] = self.flag_counts.get(f, 0) + 1
        try:
            self.fh.write(self._line(rec, flags) + "\n")
            self.fh.flush()            # survive an estop / Ctrl-C / crash
            self.jfh.write(json.dumps(rec, default=float) + "\n")
            self.jfh.flush()
        except (IOError, ValueError):
            pass                       # never let logging kill the control loop

    @staticmethod
    def _drift(m):
        """Measured/commanded motion this tick. 1.00 means the model matches the
        car; the calibration run measured 1.55x on speed and 0.80x on yaw before
        the plant constants were fixed, which is exactly the kind of drift that
        makes a planner ask for turns the car cannot make."""
        if not m:
            return "%-12s" % "--"
        # test for None, not truth: a ratio of exactly 0.00 means THE CAR DID NOT
        # MOVE, which is the single most important thing this column can say, and
        # `if m.get("r_xy")` rendered it as "-" (no data).
        rx = "-" if m.get("r_xy") is None else "%.2f" % m["r_xy"]
        rw = "-" if m.get("r_yaw") is None else "%.2f" % m["r_yaw"]
        return "%-12s" % ("x%s w%s" % (rx, rw))

    @staticmethod
    def _act(a):
        """One action rendered the same way whichever action space it came from."""
        if a is None:
            return "%-20s" % "--"
        if a.get("space") == "velocity":
            return "%-20s" % ("vel v%.3f w%+.2f" % (a.get("v", 0.0), a.get("w", 0.0)))
        return "%-20s" % ("act %-2s mag%.0f %.2fs" % (a.get("action_id", "?"),
                                                      a.get("magnitude", 0.0),
                                                      a.get("duration", 0.0)))

    def _line(self, r, flags):
        def f(x, spec, dash="  -  "):
            return dash if x is None else spec % x
        pose = r.get("pose") or [None, None, None]
        t = r.get("timing") or {}
        return ("%-6s %-6s %-6s %-9s %-6s %-6s %-6s %-6s %-12s %s %s %-12s %-13s %s" % (
            "%04d" % r.get("tick", 0),
            f(r.get("t_s"), "%.2f"),
            f(r.get("dt_s"), "%.3f"),
            f(r.get("frame_id"), "%d"),
            f(pose[0], "%+.2f"), f(pose[1], "%+.2f"), f(pose[2], "%+.1f"),
            f(r.get("gd"), "%.3f"),
            "%s n%s m%s" % (r.get("n_obs", "-"),
                            f(r.get("nearest"), "%.2f", "--"),
                            f(r.get("dmem"), "%.2f", "--")),
            self._act(r.get("dispatch")),
            self._act(r.get("plan")),
            self._drift(r.get("moved")),
            "w%-3s p%-3s W%-3s" % (f(t.get("wait_ms"), "%.0f", "-"),
                                   f(t.get("plan_ms"), "%.0f", "-"),
                                   f(t.get("work_ms"), "%.0f", "-")),
            " ".join(flags)))

    def note(self, text):
        """A free-text line between ticks (safety trips, mode changes, aborts)."""
        if not self.enabled or self.fh is None:
            return
        try:
            self.fh.write("       %s\n" % text)
            self.fh.flush()
        except IOError:
            pass
