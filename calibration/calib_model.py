#!/usr/bin/env python3
"""Calibrate the planner's PLANT MODEL against the real car.

The planner assumes `RobotConfig.pwm_per_mps` converts PWM to m/s and that a
commanded yaw rate `w` is actually achieved (it mixes with `wz_arm`, which the
variant-1 config sets equal to `Variant1Config.steer_arm`). Both were nominal
guesses. An audit of logged runs put the real values far off -- roughly 1.58x on
speed and ~3.6x on turn radius -- and on the car that shows up as variant 1
commanding max yaw for tick after tick and spiralling instead of turning.

This measures the two gains directly, in the regime the controller uses, and
prints the config values that make model == plant.

  roscar
  python calibration/calib_model.py --test yaw            # in-place, needs no space
  python calibration/calib_model.py --test linear         # needs ~1.5 m ahead
  python calibration/calib_model.py --all                 # everything (asks for space)

Reference is /odom: xy from the encoders (lidar-calibrated scales) and yaw from
the IMU gyro. Ctrl-C or any error hard-estops.
"""
import os
import sys
import time
import math
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mpc"))

import numpy as np

from carclient import CarClient
from mpc_baseline import config as C
from mpc_baseline.kinematics import wheels_to_body, velocity_to_wheel_pwm

TICK = C.TickConfig()


def _settle(car, s=1.2):
    car.stop()
    time.sleep(s)


def _pulse_for(car, wheels, secs):
    """Hold a wheel vector for `secs` by re-sending one pulse per tick, exactly the
    way the live actuator does (car-side keep-alive, next pulse supersedes)."""
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        car.drive_wheels(wheels[0], wheels[1], wheels[2], wheels[3],
                         TICK.action_duration)
        time.sleep(TICK.period)


def _measure(car, wheels, secs):
    """Run the wheel vector and return (fwd, dyaw, dt, dist) measured from /odom.

    Yaw is INTEGRATED from samples taken during the run, not differenced between
    the endpoints: /odom yaw is wrapped to (-pi, pi], so an endpoint difference
    aliases as soon as the car turns more than half a turn. That is not
    hypothetical -- a 2.8 s spin at PWM 60 wrapped a full revolution and measured
    as -3.4 deg, which made the arm come out 80x too large.
    """
    p0 = car.pose()
    t0 = time.monotonic()
    yaw_acc, last = 0.0, p0.yaw

    t_end = t0 + secs
    while time.monotonic() < t_end:
        car.drive_wheels(wheels[0], wheels[1], wheels[2], wheels[3],
                         TICK.action_duration)
        # sample several times per pulse so each increment is far below pi
        for _ in range(6):
            time.sleep(TICK.period / 6.0)
            p = car.pose()
            if p is not None:
                yaw_acc += math.atan2(math.sin(p.yaw - last), math.cos(p.yaw - last))
                last = p.yaw
    car.stop()
    dt = time.monotonic() - t0
    for _ in range(8):                    # keep integrating through the brake
        time.sleep(0.1)
        p = car.pose()
        if p is not None:
            yaw_acc += math.atan2(math.sin(p.yaw - last), math.cos(p.yaw - last))
            last = p.yaw
    p1 = car.pose()
    dx, dy = p1.x - p0.x, p1.y - p0.y
    fwd = dx * math.cos(p0.yaw) + dy * math.sin(p0.yaw)
    return fwd, yaw_acc, dt, math.hypot(dx, dy)


def _front_clear(car):
    o = car.observation()
    if o is None or not o.circles:
        return float("inf")
    ds = [math.hypot(x, y) - r for (x, y, r) in o.circles
          if abs(math.degrees(math.atan2(y, x))) <= 25.0]
    return min(ds) if ds else float("inf")


# --- tests ---------------------------------------------------------------
def test_linear(car, mags, secs):
    """Pure forward at a known PWM -> m/s. Gives pwm_per_mps."""
    print("\n=== LINEAR: commanded PWM -> actual m/s (pwm_per_mps) ===")
    need = 0.35 + secs * max(mags) / 120.0
    clear = _front_clear(car)
    print("  need ~%.2f m ahead, have %.2f m" % (need, clear))
    if clear < need:
        print("  NOT ENOUGH SPACE -- skipping linear test")
        return None
    rows = []
    for m in mags:
        fwd, dyaw, dt, dist = _measure(car, [m, m, m, m], secs)
        v = fwd / dt
        k = (m / v) if abs(v) > 1e-4 else float("nan")
        rows.append((m, v, k))
        print("   PWM %5.1f -> %.3f m/s over %.2fs (%.3f m, yaw drift %+.1f deg)"
              "  => pwm_per_mps %.0f" % (m, v, dt, fwd, math.degrees(dyaw), k))
        _settle(car)
    ks = [k for _, _, k in rows if k == k]
    return float(np.median(ks)) if ks else None


def test_yaw(car, mags, secs):
    """In-place rotation at a known PWM -> rad/s. Gives the effective arm.

    Left wheels -u, right wheels +u spins in place; the model says
    wz = u / arm, so arm_real = u / wz_measured.
    """
    print("\n=== YAW: in-place rotation, commanded PWM -> actual rad/s (arm) ===")
    rows = []
    for m in mags:
        fwd, dyaw, dt, dist = _measure(car, [-m, -m, m, m], secs)
        w = dyaw / dt
        u = max(0.0, m - C.RobotConfig.pwm_offset) / C.RobotConfig.pwm_per_mps
        arm = (u / abs(w)) if abs(w) > 1e-4 else float("nan")
        rows.append((m, w, arm))
        print("   PWM %5.1f -> %+.3f rad/s (%+.1f deg over %.2fs, drift %.3f m)"
              "  => arm %.3f m" % (m, w, math.degrees(dyaw), dt, dist, arm))
        _settle(car)
    arms = [a for _, _, a in rows if a == a]
    return float(np.median(arms)) if arms else None


def test_arc(car, mag, secs, cfg):
    """The regime the controller actually uses: v>0 with a yaw command, pushed
    through velocity_to_wheel_pwm so the deadzone bump and PWM cap are included."""
    print("\n=== ARC: end-to-end commanded (v,w) -> actual, deadzone included ===")
    clear = _front_clear(car)
    print("  front clearance %.2f m" % clear)
    if clear < 1.0:
        print("  NOT ENOUGH SPACE -- skipping arc test")
        return
    v = C._v_of_pwm(mag, cfg.robot)      # affine inverse, NOT mag/pwm_per_mps
    for frac in (0.0, 0.5, 1.0):
        wlim = min(cfg.w_max, (1.0 - cfg.min_inner_frac) * v / cfg.steer_arm)
        wc = wlim * frac
        pwm = velocity_to_wheel_pwm(v, 0.0, wc, cfg.robot)
        fwd, dyaw, dt, dist = _measure(car, list(pwm), secs)
        # Speed along the ARC, not the projection on the start heading. Projecting
        # makes a turning car look slow (it reported 53% at w=1.2) when it is only
        # travelling on a curve at full speed -- there is no v-w coupling, that
        # number was a measurement artefact. See test_couple.
        half = abs(dyaw) / 2.0
        arc = dist * (half / math.sin(half)) if half > 1e-3 else dist
        vr, wr = arc / dt, dyaw / dt
        print("   cmd v=%.3f w=%+.3f | pwm %s | REAL arc-v=%.3f (%.0f%%) w=%+.3f (%s)"
              % (v, wc, np.array2string(pwm, precision=0), vr, 100 * vr / v if v else 0,
                 wr, ("%.0f%%" % (100 * abs(wr / wc))) if abs(wc) > 1e-6 else "--"))
        _settle(car)


def test_couple(car, mag, secs, cfg, fracs=(0.0, 0.25, 0.5, 0.75, 1.0)):
    """v-w COUPLING: hold v and sweep |w|, measuring how much forward speed the
    turn costs.

    `rollout_unicycle` assumes forward speed is independent of yaw rate. A
    skid-steering chassis does not work that way -- the wheels scrub sideways in a
    turn and the car loses ground speed. Measured once at v=0.361: 99% straight,
    90% at w=0.6, 53% at w=1.2. That is the single largest remaining model error,
    so this sweeps it properly instead of fitting three points.

    The sign of w alternates so the car weaves around its start instead of
    spiralling away, and clearance is re-checked before every sample.
    """
    print("\n=== COUPLE: forward speed lost to turning (v held, |w| swept) ===")
    v = C._v_of_pwm(mag, cfg.robot)
    wlim = min(cfg.w_max, (1.0 - cfg.min_inner_frac) * v / cfg.steer_arm)
    rows, sign = [], 1.0
    print("   v held at %.3f m/s, |w| up to %.2f rad/s" % (v, wlim))
    for fr in fracs:
        clear = _front_clear(car)
        if clear < 0.9:
            print("   clearance %.2f m -- stopping the sweep here" % clear)
            break
        wc = sign * wlim * fr
        pwm = velocity_to_wheel_pwm(v, 0.0, wc, cfg.robot)
        fwd, dyaw, dt, dist = _measure(car, list(pwm), secs)
        # forward progress ALONG THE ARC: for a constant-curvature arc of turn
        # dyaw over chord `dist`, arc length = dist * (dyaw/2) / sin(dyaw/2)
        half = abs(dyaw) / 2.0
        arc = dist * (half / math.sin(half)) if half > 1e-3 else dist
        vr, wr = arc / dt, dyaw / dt
        rows.append((abs(wc), vr / v if v else 0.0, abs(wr / wc) if abs(wc) > 1e-6 else None))
        print("   |w| %.2f -> arc speed %.3f m/s (%3.0f%% of v)   yaw %+.3f (%s)"
              % (abs(wc), vr, 100 * vr / v, wr,
                 ("%.0f%%" % (100 * abs(wr / wc))) if abs(wc) > 1e-6 else "--"))
        sign = -sign
        _settle(car)
    if len(rows) >= 3:
        import numpy as _np
        W = _np.array([r[0] for r in rows]); R = _np.array([r[1] for r in rows])
        # fit r = 1 - k*w^2 (least squares through the w=0 intercept of 1)
        m = W > 1e-6
        k = float(_np.sum((1.0 - R[m]) * W[m] ** 2) / _np.sum(W[m] ** 4))
        pred = 1.0 - k * W ** 2
        print("   fit  v_eff = v * (1 - %.3f * w^2)   residuals %s"
              % (k, _np.array2string(R - pred, precision=3)))
        return k
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--test", choices=["linear", "yaw", "arc", "couple"], default=None)
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--secs", type=float, default=2.0, help="drive time per sample")
    ap.add_argument("--mags", default="40,60", help="PWM levels, comma separated")
    args = ap.parse_args()
    mags = [float(x) for x in args.mags.split(",")]

    car = CarClient()
    if car.wait_obstacles(timeout=8) is None:
        print("no /obstacles -- is perception running?"); sys.exit(1)
    if car.wait_pose(timeout=5) is None:
        print("no /odom"); sys.exit(1)
    # /battery_v needs a moment after construction -- poll like PolicyRunner does
    t0 = time.monotonic()
    while not car.link_ok() and time.monotonic() - t0 < 5.0:
        time.sleep(0.2)
    if not car.link_ok():
        print("MCU link not healthy -- refusing to drive"); sys.exit(1)
    volt = car.battery()
    print("battery %.1f V" % volt)
    if volt < 10.5:
        print("  !! WARNING: a healthy pack reads ~11-13 V. Motor speed at a given PWM\n"
              "     falls with voltage, so pwm_per_mps calibrated here will be WRONG for\n"
              "     a charged pack. Charge before trusting the linear number.\n"
              "     (The yaw/arm number is a RATIO of wheel speeds and is far less\n"
              "     voltage-sensitive, so the arm result is still usable.)")

    cfg = C.build_live_cfg("1", 40.0, 1.0)
    print("model now: m/s=(PWM-%.0f)/%.1f  wz_arm=%.3f  steer_arm=%.3f"
          % (cfg.robot.pwm_offset, cfg.robot.pwm_per_mps, cfg.robot.wz_arm,
             cfg.steer_arm))

    k_lin = arm = k_couple = None
    try:
        if args.all or args.test == "yaw":
            arm = test_yaw(car, mags, args.secs)
        if args.all or args.test == "linear":
            k_lin = test_linear(car, mags, args.secs)
        if args.all or args.test == "arc":
            test_arc(car, 40.0, args.secs, cfg)
        if args.all or args.test == "couple":
            k_couple = test_couple(car, 40.0, args.secs, cfg)
    except BaseException as exc:
        print("\nABORT (%s) -- estop" % type(exc).__name__)
        car.estop()
        raise
    finally:
        car.stop()

    print("\n=== RESULT ===")
    if k_lin:
        print("  pwm_per_mps : %.0f  (config has %.0f)  -> RobotConfig.pwm_per_mps = %.0f"
              % (k_lin, cfg.robot.pwm_per_mps, k_lin))
    if arm:
        print("  effective arm: %.3f m (config steer_arm/wz_arm = %.3f)"
              % (arm, cfg.steer_arm))
        print("  -> Variant1Config.steer_arm = %.3f   (live_config_v1 copies it to wz_arm)" % arm)
        print("     note this also TIGHTENS the inner-wheel limit |w| <= 0.9*v/steer_arm,")
        print("     which is correct: the old 0.10 let the planner ask for turns the car cannot do.")
    if k_couple:
        print("  v-w coupling: v_eff = v * (1 - %.3f * w^2)  -> RobotConfig.v_w_couple = %.3f"
              % (k_couple, k_couple))
    if not k_lin and not arm and not k_couple:
        print("  (nothing measured)")


if __name__ == "__main__":
    main()
