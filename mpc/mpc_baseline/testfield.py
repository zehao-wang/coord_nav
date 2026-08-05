"""TestField: a live-faithful offline proving ground for ANY plugged-in policy.

The point of this module is INPUT PARITY: a policy dropped into the field
receives the same thing PolicyRunner feeds it on the real car --

  * the same `carpolicy.Observation` contract: pose (odom frame, run-start
    anchored), goal B, current-frame base-frame circles (x fwd, y left), and the
    SAME `ObstacleField` odom-memory code the runner uses;
  * the same cadence and timing: 3 Hz ticks, the action decided at tick N
    dispatching at tick N+1 (buffered loop), planning from the dead-time
    compensated pose -- run_episode(buffered=True), i.e. the runner's loop shape;
  * the same execution: the measured DisturbanceConfig plant (yaw lag + speed
    noise, fit from 298 on-car ticks) at the real 1/3 s tick;
  * occlusion (NEW here): the lidar cannot see an obstacle whose centre ray is
    blocked by another obstacle -- the old sim saw through everything. The
    approximation: a circle is dropped when the ray from the car to its CENTRE
    passes through another circle (real clustering keeps partially visible
    obstacles, so this only hides mostly-shadowed ones);
  * knobs for the remaining perception imperfections (noise_xy / dropout /
    whole-frame loss are structural in KinematicSim/PerceptionConfig but their
    magnitudes are NOT yet fit from car data -- they default to 0. Frame skips
    measured 0 on healthy WiFi, 2026-08-04 A/B logs).

Known parity gaps, on purpose: pose is exact (no odom drift model) and the
scan/pose timestamp skew during fast yaw is not modelled.

Obstacles move: each dynamic obstacle is a pymunk rigid body (elastic, no
friction/damping) bouncing off the arena walls, the static obstacles and each
other, so randomly generated cases stay physically consistent. The car is NOT a
pymunk body -- it moves by the same calibrated kinematics the planner assumes
(KinematicSim.step), and contact is scored exactly like the static suite and the
live guard: obstacle edge reaching `robot_radius`, swept through sub-steps so a
crossing obstacle cannot tunnel through the footprint between ticks.

Plug in a policy: register it (mpc_baseline.registry.register) and pass its key
to run_case / run_field / scripts/run_field.py -- construction, seeding and dt
rules go through eval.resolve_policy, the same single path the benchmarks use.
"""

from collections import namedtuple
from dataclasses import dataclass

import numpy as np
import pymunk

from . import config as C
from .eval import resolve_policy
from .obstacles import ObstacleField
from .sim import KinematicSim, World, run_episode


def _visible(circles, pose, sense_range, occlusion):
    """Indices of world circles the lidar can currently see (range + occlusion,
    no stochastic knobs) -- the deterministic core of DynamicSim.sense, shared
    with the animation's tracker replay."""
    px, py, _ = pose
    out = []
    for i in range(len(circles)):
        wx, wy, r = circles[i]
        if np.hypot(wx - px, wy - py) - r > sense_range:
            continue
        if occlusion and _centre_occluded(circles, i, px, py):
            continue
        out.append(i)
    return out

# A test-field case. static: ((x, y, r), ...) fixed circles. dynamic:
# ((x, y, r, vx, vy), ...) pymunk bodies launched with that velocity. arena:
# (xmin, ymin, xmax, ymax) elastic walls for the OBSTACLES (the car ignores
# them). All coordinates in the world (= odom = run-start) frame, metres.
Case = namedtuple("Case", ["name", "start", "static", "dynamic", "arena"])


@dataclass
class PerceptionConfig:
    """What the fake /obstacles feed degrades. sense_range matches the live
    perception cap; occlusion is geometry (always faithful to switch on); the
    noise magnitudes are honest KNOBS, not measurements -- they default to 0
    until fitted from recorded run.json frames (policy_run records the real
    circles every tick, so the fit is possible; not done yet)."""
    sense_range: float = 3.0
    occlusion: bool = True
    noise_xy: float = 0.0             # gaussian std on sensed centres (m), UNFITTED
    dropout: float = 0.0              # per-obstacle miss probability, UNFITTED


def _centre_occluded(circles, i, px, py):
    """True if circles[i]'s centre ray from (px,py) passes through another circle."""
    cx, cy = circles[i][0], circles[i][1]
    ax, ay = cx - px, cy - py
    l2 = ax * ax + ay * ay
    if l2 < 1e-12:
        return False
    for j in range(len(circles)):
        if j == i:
            continue
        ox, oy, orr = circles[j][0], circles[j][1], circles[j][2]
        t = ((ox - px) * ax + (oy - py) * ay) / l2
        if t <= 0.0 or t >= 1.0:                  # not strictly between car and target
            continue
        ddx = px + t * ax - ox
        ddy = py + t * ay - oy
        if ddx * ddx + ddy * ddy < orr * orr:
            return True
    return False


class DynamicSim(KinematicSim):
    """KinematicSim + pymunk-driven obstacles + occlusion + swept contact.

    The car steps through KinematicSim.step (identical plant + disturbance
    semantics as every other eval, so field numbers are comparable); obstacles
    advance in `substeps` pymunk sub-steps per tick, and the clearance reported
    for the step is the MINIMUM over sub-steps with the car interpolated along
    its chord -- a crossing obstacle that grazes the footprint mid-tick counts,
    even if both have moved apart by the tick boundary. (This is sub-step
    SAMPLING, not a continuous sweep: at field speeds the clearance error is
    sub-mm, but a mover above ~1 m/s relative speed could slip between samples
    -- raise `substeps` if you add fast movers.)"""

    def __init__(self, case, sense_range=3.0, robot_radius=0.13, noise_xy=0.0,
                 dropout=0.0, seed=0, disturbance=None, occlusion=True, substeps=10):
        circles0 = [tuple(map(float, s)) for s in case.static] + \
                   [(float(d[0]), float(d[1]), float(d[2])) for d in case.dynamic]
        super().__init__(World(circles0, case.start, case.name),
                         sense_range=sense_range, robot_radius=robot_radius,
                         noise_xy=noise_xy, dropout=dropout, seed=seed,
                         disturbance=disturbance)
        self.case = case
        self.occlusion = occlusion
        self.substeps = max(1, int(substeps))
        self.n_static = len(case.static)
        self._swept_clr = None

        self.space = pymunk.Space()
        self.space.gravity = (0.0, 0.0)          # top-down world
        self._bodies = []
        for (x, y, r, vx, vy) in case.dynamic:
            body = pymunk.Body(1.0, float("inf"))    # no spin: circles, frictionless
            body.position = (float(x), float(y))
            body.velocity = (float(vx), float(vy))
            shape = pymunk.Circle(body, float(r))
            shape.elasticity = 1.0
            shape.friction = 0.0
            self.space.add(body, shape)
            self._bodies.append(body)
        for (x, y, r) in case.static:
            shape = pymunk.Circle(self.space.static_body, float(r),
                                  offset=(float(x), float(y)))
            shape.elasticity = 1.0
            shape.friction = 0.0
            self.space.add(shape)
        x0, y0, x1, y1 = case.arena
        for a, b in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
                     ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
            seg = pymunk.Segment(self.space.static_body, a, b, 0.02)
            seg.elasticity = 1.0
            seg.friction = 0.0
            self.space.add(seg)

        # (t, circles copy, pose copy) per tick -- what animate_case renders
        self.history = [(self.t, self.circles.copy(), self.pose.copy())]

    def _sync(self):
        for i, body in enumerate(self._bodies):
            self.circles[self.n_static + i, 0] = body.position.x
            self.circles[self.n_static + i, 1] = body.position.y

    def step(self, body_vel, dt):
        prev = self.pose.copy()
        out = super().step(body_vel, dt)         # car plant + disturbance, unchanged
        swept = np.inf
        for k in range(self.substeps):
            self.space.step(dt / self.substeps)
            self._sync()
            if len(self.circles):
                a = (k + 1.0) / self.substeps
                cx = prev[0] + a * (out[0] - prev[0])
                cy = prev[1] + a * (out[1] - prev[1])
                d = np.hypot(self.circles[:, 0] - cx,
                             self.circles[:, 1] - cy) - self.circles[:, 2]
                swept = min(swept, float(d.min()))
        self._swept_clr = swept
        self.history.append((self.t, self.circles.copy(), self.pose.copy()))
        return out

    def true_min_clearance(self):
        inst = super().true_min_clearance()
        if self._swept_clr is None:              # before the first step
            return inst
        return min(inst, float(self._swept_clr))

    def sense(self):
        """Fake /obstacles with the field's perception model: range cut, then
        occlusion (deterministic geometry), then the stochastic knobs. With
        occlusion ON and dropout/noise_xy nonzero, hidden circles skip their
        RNG draws, so the stream diverges from KinematicSim.sense -- seeds are
        comparable across field runs, not against the static-suite sim."""
        out = []
        px, py, pth = self.pose
        c, s = np.cos(-pth), np.sin(-pth)
        for i in _visible(self.circles, self.pose, self.sense_range,
                          self.occlusion):
            wx, wy, r = self.circles[i]
            if self.dropout and self.rng.random() < self.dropout:
                continue
            dx, dy = wx - px, wy - py
            bx = c * dx - s * dy
            by = s * dx + c * dy
            if self.noise_xy:
                bx += self.rng.normal(0.0, self.noise_xy)
                by += self.rng.normal(0.0, self.noise_xy)
            out.append((float(bx), float(by), float(r)))
        return out


# --- cases ----------------------------------------------------------------
def _check_goal_dist(goal_dist):
    """The field's cases are laid out for the REALISTIC regime (B >= 2 m).
    Below that the archetype movers spawn outside the arena (where they bounce
    off the wrong side of the walls and escape) and the random generator's
    corridor box collapses (uniform(0.7, goal_dist-0.4) inverts) -- both used to
    fail silently or with an opaque numpy error. Tight short-range worlds are
    default_scenarios' job, not the field's."""
    if goal_dist < 2.0:
        raise ValueError(
            "TestField cases need goal_dist >= 2.0 m (got %.2f): the archetype "
            "and random layouts assume the realistic regime. For short-range "
            "tight worlds use sim.default_scenarios." % goal_dist)


def archetype_cases(goal_dist=3.0):
    """Named single-mechanism cases, each isolating one dynamic interaction the
    static suites cannot produce. Speeds are pedestrian-scale (0.2-0.4 m/s)
    against the car's 0.36 m/s."""
    _check_goal_dist(goal_dist)
    A = (-0.8, -1.4, goal_dist + 0.8, 1.4)
    S = (0.0, 0.0, 0.0)
    return [
        # crosses the corridor around the time the car gets there
        Case("cross_slow", S, (), ((1.6, -1.0, 0.13, 0.0, 0.22),), A),
        # fast crosser from the left -- passes ahead if the car just keeps going
        Case("cross_fast", S, (), ((1.5, 1.1, 0.13, 0.0, -0.40),), A),
        # head-on down the A->B line, closing speed ~0.6 m/s
        Case("oncoming", S, (), ((goal_dist - 0.3, 0.06, 0.13, -0.22, 0.0),), A),
        # diagonal drift into the corridor from ahead-right
        Case("diagonal", S, (), ((2.3, -0.9, 0.13, -0.18, 0.25),), A),
        # occlusion exercise: an ONCOMING mover hidden dead behind the static box
        # on the A->B line -- invisible for the whole approach (the centre ray
        # runs through the box) and it pops into view at close range only once
        # the car commits to a side. Verified to occlude for multiple seconds,
        # unlike a crosser, which leaves the shadow almost immediately.
        Case("occluded_oncoming", S, ((1.3, 0.0, 0.13),),
             ((2.4, 0.0, 0.12, -0.25, 0.0),), A),
    ]


def random_cases(n=10, seed=0, goal_dist=3.0, n_static=(0, 2), n_dynamic=(1, 2),
                 radius=(0.11, 0.15), speed=(0.10, 0.35)):
    """Seeded random cases: obstacles spawn in the corridor box, clear of the
    start and goal discs and of each other; movers get a random heading and a
    pedestrian-scale speed. Same seed -> byte-identical cases."""
    _check_goal_dist(goal_dist)
    rng = np.random.default_rng(seed)
    A = (-0.8, -1.4, goal_dist + 0.8, 1.4)
    cases = []
    for i in range(n):
        placed = []

        def _place(r):
            for _ in range(200):
                x = float(rng.uniform(0.7, goal_dist - 0.4))
                y = float(rng.uniform(-1.0, 1.0))
                if np.hypot(x, y) < 0.55 + r:                    # start clear
                    continue
                if np.hypot(x - goal_dist, y) < 0.45 + r:        # goal clear
                    continue
                if any(np.hypot(x - qx, y - qy) < r + qr + 0.06
                       for qx, qy, qr in placed):
                    continue
                placed.append((x, y, r))
                return x, y
            raise RuntimeError("could not place an obstacle (case %d): the "
                               "corridor box is over-packed for these radii" % i)

        static = []
        for _ in range(int(rng.integers(n_static[0], n_static[1] + 1))):
            r = float(rng.uniform(*radius))
            x, y = _place(r)
            static.append((x, y, r))
        dynamic = []
        for _ in range(int(rng.integers(n_dynamic[0], n_dynamic[1] + 1))):
            r = float(rng.uniform(*radius))
            x, y = _place(r)
            sp = float(rng.uniform(*speed))
            th = float(rng.uniform(0.0, 2.0 * np.pi))
            dynamic.append((x, y, r, sp * np.cos(th), sp * np.sin(th)))
        cases.append(Case("rand%02d" % i, (0.0, 0.0, 0.0),
                          tuple(static), tuple(dynamic), A))
    return cases


def default_cases(goal_dist=3.0, n_random=10, random_seed=1000):
    """The field's standard battery: 5 archetypes + n_random seeded random cases."""
    return archetype_cases(goal_dist) + random_cases(n_random, seed=random_seed,
                                                     goal_dist=goal_dist)


# --- running --------------------------------------------------------------
def run_case(spec, case, live=True, goal_dist=3.0, goal_y=0.0, magnitude=40.0,
             step_duration=0.5, plan_dt=None, disturbed=True, seed=0,
             perception=None, obs_cfg=None, keep_history=False, max_steps=400,
             substeps=10, tweak=None):
    """One policy (registry key or built-in 1/2), one case. LIVE-FAITHFUL BY
    DEFAULT: disturbed=True is the field's normal mode -- pass disturbed=False
    only to inspect the idealized perfect-execution world -- and built-in 1/2
    specs get the LIVE profile (live=True: the magnitude-40 car, 0.361 m/s, like
    every registry entry), not benchmark.py's slower sim profile. Returns an
    EpisodeResult; keep_history=True returns (result, history) for animation."""
    p = perception or PerceptionConfig()
    policy, cfg, dt = resolve_policy(spec, live=live, goal_dist=goal_dist,
                                     goal_y=goal_y, magnitude=magnitude,
                                     step_duration=step_duration, plan_dt=plan_dt,
                                     disturbed=disturbed, seed=seed)
    if tweak is not None:
        # experiment hook: mutate the freshly built cfg (e.g. cost weights)
        # before the policy plans with it -- what the screening sweeps needed
        tweak(cfg)
    sim = DynamicSim(case, sense_range=p.sense_range,
                     robot_radius=cfg.robot.robot_radius,
                     noise_xy=p.noise_xy, dropout=p.dropout, seed=seed,
                     disturbance=(C.DisturbanceConfig() if disturbed else None),
                     occlusion=p.occlusion, substeps=substeps)
    res = run_episode(sim, policy, spec, obs_cfg or C.ObstacleConfig(), cfg.goal,
                      plan_dt=dt, robot_cfg=cfg.robot, buffered=disturbed,
                      max_steps=max_steps)
    return (res, sim.history) if keep_history else res


def run_field(specs=("mpc_grid", "mpc_vw"), cases=None, seeds=20, goal_dist=3.0,
              disturbed=True, perception=None, obs_cfg=None, **kw):
    """The full battery: every spec over cases x seeds. Returns {spec: [results]}
    ordered cases-major within each seed, so results[:len(cases)] is seed 0.
    Any policy registered in POLICY_REGISTRY plugs in by key."""
    cases = cases if cases is not None else default_cases(goal_dist)
    out = {}
    for spec in specs:
        rs = []
        for s in range(seeds):
            for case in cases:
                rs.append(run_case(spec, case, goal_dist=goal_dist,
                                   disturbed=disturbed, perception=perception,
                                   obs_cfg=obs_cfg, seed=s, **kw))
        out[str(spec)] = rs
    return out


# --- animation ------------------------------------------------------------
def animate_case(spec, case, path, seed=0, goal_dist=3.0, disturbed=True,
                 perception=None, fps=6, **kw):
    """Run one episode and render it frame-by-frame (one frame per tick, fps=6 is
    2x real time). `.mp4` uses ffmpeg, anything else Pillow (gif). Obstacles the
    car cannot currently see (range/occlusion) render hollow -- watching an
    occluded mover pop into view is the point of half these cases.

    For a TIME-AWARE policy (cfg.predict_obstacles) the frames also show what
    the planner is actually scoring against: a purple velocity arrow per
    tracked mover and dashed ghost circles at the tracker's +1 s / +2 s
    constant-velocity predictions. The tracker is REPLAYED from the recorded
    history through the same ObstacleField code (deterministic as long as the
    perception noise knobs are 0 -- with noise/dropout enabled the overlay is
    skipped rather than shown wrong). Returns (EpisodeResult, path)."""
    p = perception or PerceptionConfig()
    res, hist = run_case(spec, case, seed=seed, goal_dist=goal_dist,
                         disturbed=disturbed, perception=p, keep_history=True, **kw)

    snaps = None
    _, _cfg, _ = resolve_policy(spec, goal_dist=goal_dist, disturbed=disturbed,
                                magnitude=kw.get("magnitude", 40.0))
    if kw.get("tweak") is not None:
        # the overlay probe must see the same cfg the scored episode ran with
        # (a tweak can flip predict_obstacles either way)
        kw["tweak"](_cfg)
    if getattr(_cfg, "predict_obstacles", False) and not p.noise_xy and not p.dropout:
        clk = {"t": 0.0}
        field = ObstacleField(kw.get("obs_cfg") or C.ObstacleConfig(),
                              lambda: clk["t"])
        snaps = []
        for (t, circles, pose) in hist:
            clk["t"] = t
            px, py, pth = pose
            cth, sth = np.cos(-pth), np.sin(-pth)
            base = []
            for i in _visible(circles, pose, p.sense_range, p.occlusion):
                dx, dy = circles[i][0] - px, circles[i][1] - py
                base.append((cth * dx - sth * dy, sth * dx + cth * dy,
                             circles[i][2]))
            field.update(base, pose)
            snaps.append((field.circles(), field.velocities(),
                          field.predict([1.0, 2.0])))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation as manim

    x0, y0, x1, y1 = case.arena
    fig, ax = plt.subplots(figsize=(7, 7 * (y1 - y0 + 0.4) / (x1 - x0 + 0.4)))
    goal = (goal_dist, 0.0)
    outcome = "REACHED" if res.reached else ("COLLIDED" if res.collided else "timeout")

    def draw(k):
        t, circles, pose = hist[k]
        ax.clear()
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                                   ec="0.6", lw=1))
        for i in range(len(circles)):
            cx, cy, r = circles[i]
            seen = (np.hypot(cx - pose[0], cy - pose[1]) - r <= p.sense_range and
                    not (p.occlusion and _centre_occluded(circles, i, pose[0], pose[1])))
            dyn = i >= len(case.static)
            ax.add_patch(plt.Circle((cx, cy), r, color="tab:red" if dyn else "0.4",
                                    alpha=0.5 if seen else 0.15,
                                    fill=seen, lw=1.5))
        if snaps is not None:
            mem, vel, pred = snaps[k]
            for i in range(len(mem)):
                vx, vy = vel[i]
                if vx == 0.0 and vy == 0.0:
                    continue
                ax.arrow(mem[i, 0], mem[i, 1], vx, vy, head_width=0.06,
                         color="tab:purple", alpha=0.9,
                         length_includes_head=True)
                for j, a in ((0, 0.45), (1, 0.22)):     # +1 s / +2 s ghosts
                    ax.add_patch(plt.Circle(
                        (pred[j, i, 0], pred[j, i, 1]), pred[j, i, 2],
                        fill=False, ls="--", lw=1.2, color="tab:purple",
                        alpha=a))
        tr = res.traj[:k + 1]
        ax.plot(tr[:, 0], tr[:, 1], "-", color="tab:blue", lw=1, alpha=0.7)
        ax.add_patch(plt.Circle((pose[0], pose[1]), 0.13, color="tab:blue", alpha=0.6))
        hx = pose[0] + 0.2 * np.cos(pose[2])
        hy = pose[1] + 0.2 * np.sin(pose[2])
        ax.plot([pose[0], hx], [pose[1], hy], "-", color="k", lw=1.5)
        ax.plot(goal[0], goal[1], "g*", ms=16)
        ax.set_xlim(x0 - 0.2, x1 + 0.2)
        ax.set_ylim(y0 - 0.2, y1 + 0.2)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.25)
        ax.set_title("%s  %s  t=%4.1fs  clr=%.2f m  [%s]"
                     % (case.name, spec, t, res.min_clearance, outcome))

    anim = manim.FuncAnimation(fig, draw, frames=len(hist))
    if str(path).endswith(".mp4"):
        anim.save(path, writer=manim.FFMpegWriter(fps=fps))
    else:
        anim.save(path, writer=manim.PillowWriter(fps=fps))
    plt.close(fig)
    return res, path
