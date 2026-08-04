"""The two MPC baseline policies -- each an implementation of the Policy interface.

  Variant 2 (discrete grid-hop):    sampling / enumeration MPC over the mecanum
                                    action set -- THE default baseline policy.
  Variant 1 (continuous v, omega):  continuous (v,w) SAMPLING MPC / DWA -- the same
                                    core, continuous action space (pure numpy, no solver).

Both implement `Policy`: plan(Observation) -> Action (see carpolicy for the full
input/output contract). Swap in any other Policy the same way.
"""

import numpy as np

from carpolicy import Policy, Action
from .kinematics import rollout_unicycle, rollout_body, build_action_table
from .cost import total_cost_discrete, total_cost_velocity
from .mppi import enumerate_action_sequences, sample_action_sequences


class Variant1Policy(Policy):
    """Continuous (v, w) sampling MPC (DWA) -- same receding-horizon search as
    variant 2 over a continuous action space:

        sample K control sequences (v,w) around a warm-started nominal
          -> roll out the unicycle model
          -> score each against the goal + ALL obstacle circles + the A->B line
          -> take the argmin's first (v,w)      (argmin, not softmax: averaging
             "go left" and "go right" gives "go straight into it")
          -> time-shift the winner as next cycle's nominal (warm start).

    The argmin commits to one side of a dead-ahead obstacle and the warm start
    carries that choice forward. Every sampled sequence already satisfies the
    forward-differential-drive limit (inner wheel never reverses)."""

    action_space = "velocity"        # emits Action.velocity(v, w, traj)

    def __init__(self, cfg, seed=0):
        self.cfg = cfg
        self.rng = np.random.default_rng(seed)
        self._nominal = None                  # (H, 2) warm-start control sequence
        self._u_prev = None                   # control the car is currently executing
        self._A = None                        # episode start (the A of the A->B line)

    @staticmethod
    def _track_line(A, B):
        """(nx, ny, c) of the straight A->B line: cross-track(p) = nx*x+ny*y - c."""
        d = np.asarray(B, float) - np.asarray(A, float)
        n = float(np.hypot(d[0], d[1]))
        if n < 1e-6:
            return np.array([0.0, 1.0, A[1]])
        dh = d / n
        normal = np.array([-dh[1], dh[0]])    # unit normal to A->B
        return np.array([normal[0], normal[1], normal[0] * A[0] + normal[1] * A[1]])

    def _perturb(self, nom):
        """(K, H, 2) sampled [v, w] sequences around `nom`. The noise is SMOOTHED
        along the horizon (AR(1)), so a sample is a sustained manoeuvre (a real
        turn) rather than per-step jitter -- that is what covers smooth go-arounds.
        Clipped to the velocity box AND to the yaw the car can actually DELIVER,
        yaw_gain*((1-min_inner_frac)*v/steer_arm - yaw_deadband), capped at w_max
        (the mix limit keeps the inner wheel rolling forward -- see below)."""
        m = self.cfg.mppi
        H, K = m.horizon, m.samples
        arm = max(self.cfg.steer_arm, 1e-6)
        frac = self.cfg.min_inner_frac
        raw = self.rng.normal(0.0, (m.noise_v, m.noise_w), size=(K, H, 2))
        # beta from a TIME constant, so a sample is the same physical manoeuvre
        # whatever the step time: a per-step beta would halve the smoothing horizon
        # (in seconds) the moment dt halved.
        beta = float(np.exp(-m.dt / max(m.noise_tau, 1e-6)))
        noise = np.empty_like(raw)
        # step 0 seeded at full sigma (not the AR(1) stationary scale) -> after the
        # rescale below it is the noisiest step: extra exploration on the control we
        # execute this cycle, which helps tight go-arounds.
        noise[:, 0] = raw[:, 0]
        for h in range(1, H):                 # AR(1) low-pass -> temporally smooth
            noise[:, h] = beta * noise[:, h - 1] + (1.0 - beta) * raw[:, h]
        # rescale so the smoothed steps keep ~the input std: an AR(1) driven by
        # (1-beta)*e has stationary var (1-beta)/(1+beta)*sigma^2, so scale the std
        # back up by sqrt((1+beta)/(1-beta)).
        noise *= np.sqrt((1.0 + beta) / (1.0 - beta))
        seqs = nom[None] + noise
        seqs[0] = nom                         # always evaluate the nominal itself
        seqs[:, :, 0] = np.clip(seqs[:, :, 0], self.cfg.v_min, self.cfg.v_max)
        # Yaw limit = what the car can actually DELIVER, not what the mix will accept.
        # The mix limit (inner wheel stays forward) is (1-frac)*v/arm; the yaw
        # feedforward then has to add yaw_deadband on top to overcome roller scrub,
        # and IT is capped by the same mix limit. So clipping the policy to the mix
        # limit made the feedforward a no-op exactly at the limit: measured commanded
        # -> realised 46 % at magnitude 20 and 85 % at 30 (100 % only at 40, which is
        # the one magnitude the on-car test used). Asking only for the achievable
        # value makes commanded == realised at every magnitude.
        robot = self.cfg.robot
        raw = (1.0 - frac) * seqs[:, :, 0] / arm
        achievable = getattr(robot, "yaw_gain", 1.0) * np.maximum(
            0.0, raw - getattr(robot, "yaw_deadband", 0.0))
        wlim = np.maximum(0.0, np.minimum(self.cfg.w_max, achievable))
        seqs[:, :, 1] = np.clip(seqs[:, :, 1], -wlim, wlim)
        return seqs

    def plan(self, obs):
        pose = np.asarray(obs.pose, dtype=float)
        goal = np.asarray(obs.goal, dtype=float)[:2]
        if self._A is None:
            self._A = pose[:2].copy()         # first pose of the run = A
        line = self._track_line(self._A, goal)
        m = self.cfg.mppi
        if self._nominal is None:
            nom = np.zeros((m.horizon, 2))
            nom[:, 0] = self.cfg.v_max        # default plan: drive straight to B
        else:
            nom = self._nominal
        # refine: sample around the nominal, keep the argmin, resample around it
        best_seq, best_states = nom, None
        for _ in range(max(1, m.n_iters)):
            seqs = self._perturb(nom)                                  # (K, H, 2)
            states = rollout_unicycle(pose, seqs, m.dt)                # (K, H, 3)
            cost, _ = total_cost_velocity(states, seqs, goal, line, obs.field,
                                          self.cfg.robot, self.cfg.cost, m.dt,
                                          self._u_prev,
                                          predict=getattr(self.cfg,
                                                          "predict_obstacles",
                                                          False),
                                          pred_delay=getattr(self.cfg,
                                                             "pred_extra_delay_s",
                                                             0.0))
            b = int(np.argmin(cost))
            best_seq, best_states, nom = seqs[b], states[b], seqs[b]
        self._nominal = np.vstack([best_seq[1:], best_seq[-1:]])       # time-shift warm start
        self._u_prev = best_seq[0].copy()      # what the car will be running next tick
        return Action.velocity(float(best_seq[0, 0]), float(best_seq[0, 1]),
                               traj=best_states, controls=best_seq)    # full (H,2) horizon

    def reset(self):
        self._nominal = None
        self._A = None
        self._u_prev = None


class Variant2Policy(Policy):
    """Discrete sampling MPC: score action-index sequences by rollout, apply the
    first action of the lowest-cost sequence (argmin), re-plan each hop."""

    action_space = "discrete"        # emits Action.discrete(id, mag, dur, traj)

    def __init__(self, cfg, seed=0):
        self.cfg = cfg
        self.ids, self.table = build_action_table(
            cfg.actions, cfg.step_magnitude, cfg.robot)   # ids (n,), table (n,3)
        self.n = len(self.ids)
        self.rng = np.random.default_rng(seed)
        self._nominal = None                              # warm-start seq (indices)
        # unit translation direction per action (zero rows for rotate/stop):
        # feeds the direction-smoothness cost
        norms = np.hypot(self.table[:, 0], self.table[:, 1])
        self._dirs = np.zeros((self.n, 2))
        nz = norms > 1e-9
        self._dirs[nz, 0] = self.table[nz, 0] / norms[nz]
        self._dirs[nz, 1] = self.table[nz, 1] / norms[nz]
        self._prev_dir = None                             # direction the car is executing

    def _candidates(self):
        H = self.cfg.horizon
        if self.n ** H <= self.cfg.exhaustive_cap:
            return enumerate_action_sequences(self.n, H)
        return sample_action_sequences(self.n, H, self.cfg.samples, self.rng,
                                       self._nominal)

    def plan(self, obs):
        pose = np.asarray(obs.pose, dtype=float)
        goal = np.asarray(obs.goal, dtype=float)[:2]
        seqs = self._candidates()                         # (K, H) action indices
        body = self.table[seqs]                           # (K, H, 3) body velocities
        # Roll out at the EXECUTION step, not the hop's life: the runner dispatches a
        # hop and the next tick supersedes it, so only one tick of each hop actually
        # happens. Rolling out at step_duration=0.5 against a 0.333 s tick predicted
        # every hop 50% longer than the car performs.
        dt = getattr(self.cfg, "rollout_dt", None) or self.cfg.step_duration
        states = rollout_body(pose, body, dt)

        cost, _collided = total_cost_discrete(
            states, goal, obs.field, self.cfg.robot, self.cfg.cost, dt,
            predict=getattr(self.cfg, "predict_obstacles", False),
            pred_delay=getattr(self.cfg, "pred_extra_delay_s", 0.0),
            dirs=self._dirs[seqs], prev_dir=self._prev_dir)

        best = int(np.argmin(cost))
        best_seq = seqs[best]
        self._prev_dir = self._dirs[best_seq[0]].copy()   # what the car runs next tick
        # warm start next cycle: time-shift, repeat the last action
        self._nominal = np.concatenate([best_seq[1:], best_seq[-1:]])
        mag, dur = self.cfg.step_magnitude, self.cfg.step_duration
        horizon = [(int(self.ids[i]), mag, dur) for i in best_seq]   # full hop plan
        return Action.discrete(horizon[0][0], mag, dur, traj=states[best],
                               controls=horizon)

    def reset(self):
        self._nominal = None
        self._prev_dir = None


def make_policy(variant, cfg, **kw):
    """variant in {1, 2, '1', '2', 'vw', 'grid'} -> the matching policy instance."""
    v = str(variant).lower()
    if v in ("1", "vw", "v1"):
        return Variant1Policy(cfg, **kw)
    if v in ("2", "grid", "v2"):
        return Variant2Policy(cfg, **kw)
    raise ValueError("unknown variant %r (use 1 or 2)" % (variant,))
