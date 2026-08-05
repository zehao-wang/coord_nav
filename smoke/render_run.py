#!/usr/bin/env python3
"""Render a recorded live run (smoke/results/<tag>.json) into a top-down video.

    python smoke/render_run.py smoke/results/dense_h12ema_gridt.json out.mp4

Draws, per recorded tick: the base-frame /obstacles circles transformed to the
run-start frame, the car footprint + heading + trail, goal B, and -- when the
run recorded them (any policy, live tracker) -- purple velocity arrows and the
dashed +1s/+2s constant-velocity prediction ghosts the time-aware policies plan
against. The same visual language as the TestField animations, but showing the
REAL car and REAL perception.
"""

import json
import math
import sys

import numpy as np


def main(path, out, fps=6):
    d = json.load(open(path))
    steps = [s for s in d["steps"] if s.get("pose") and s.get("circles") is not None]
    if not steps:
        raise SystemExit("no renderable steps in %s" % path)
    s0 = steps[0]["pose"]
    c0, sn0 = math.cos(s0[2]), math.sin(s0[2])

    def to_start(x, y):
        dx, dy = x - s0[0], y - s0[1]
        return (c0 * dx + sn0 * dy, -sn0 * dx + c0 * dy)

    goal = d.get("goal") or [0, 0]
    goal_s = to_start(goal[0], goal[1]) if d.get("goal") else None
    summ = d.get("summary", {})
    outcome = summ.get("reason", "?")
    label = summ.get("policy", "?")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation as manim

    # extent over the whole run
    trail = [to_start(*st["pose"][:2]) for st in steps]
    xs = [p[0] for p in trail] + ([goal_s[0]] if goal_s else [])
    ys = [p[1] for p in trail] + ([goal_s[1]] if goal_s else [])
    x0, x1 = min(xs) - 1.2, max(xs) + 1.2
    y0, y1 = min(ys) - 1.2, max(ys) + 1.2

    fig, ax = plt.subplots(figsize=(7, 7 * (y1 - y0) / (x1 - x0)))

    def draw(k):
        st = steps[k]
        px, py, pth = st["pose"]
        cp, sp = math.cos(pth), math.sin(pth)
        ax.clear()
        # obstacles: base -> odom -> start frame
        for (bx, by, r) in st["circles"]:
            ox, oy = px + cp * bx - sp * by, py + sp * bx + cp * by
            gx, gy = to_start(ox, oy)
            if not (x0 - 0.5 < gx < x1 + 0.5 and y0 - 0.5 < gy < y1 + 0.5):
                continue
            ax.add_patch(plt.Circle((gx, gy), r, color="tab:red", alpha=0.35))
        # tracker overlay (odom-frame tracks/pred recorded by the runner)
        for tr in (st.get("tracks") or []):
            wx, wy, r, vx, vy = tr
            gx, gy = to_start(wx, wy)
            gvx = c0 * vx + sn0 * vy
            gvy = -sn0 * vx + c0 * vy
            ax.arrow(gx, gy, gvx, gvy, head_width=0.06, color="tab:purple",
                     alpha=0.9, length_includes_head=True)
        for alpha, horizon in zip((0.45, 0.22), (st.get("pred") or [])):
            for (wx, wy, r) in horizon:
                gx, gy = to_start(wx, wy)
                ax.add_patch(plt.Circle((gx, gy), r, fill=False, ls="--",
                                        lw=1.2, color="tab:purple", alpha=alpha))
        # trail + car + goal
        tr = trail[:k + 1]
        ax.plot([p[0] for p in tr], [p[1] for p in tr], "-",
                color="tab:blue", lw=1, alpha=0.7)
        gx, gy = trail[k]
        ax.add_patch(plt.Circle((gx, gy), 0.13, color="tab:blue", alpha=0.6))
        hx = gx + 0.22 * math.cos(pth - s0[2])
        hy = gy + 0.22 * math.sin(pth - s0[2])
        ax.plot([gx, hx], [gy, hy], "-", color="k", lw=1.5)
        if goal_s:
            ax.plot(goal_s[0], goal_s[1], "g*", ms=16)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.25)
        t_rel = st["t"] - steps[0]["t"]
        ax.set_title("%s  t=%4.1fs  [%s]" % (label, t_rel, outcome))

    anim = manim.FuncAnimation(fig, draw, frames=len(steps))
    if str(out).endswith(".mp4"):
        anim.save(out, writer=manim.FFMpegWriter(fps=fps))
    else:
        anim.save(out, writer=manim.PillowWriter(fps=fps))
    plt.close(fig)
    print("wrote %s (%d frames)" % (out, len(steps)))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
