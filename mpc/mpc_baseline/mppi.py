"""Action-sequence samplers for the discrete sampling MPC (variant 2, pure numpy).

Variant 2 is a receding-horizon search over the discrete mecanum action set: at
each cycle we score a batch of action-index sequences by rolling them out and
pick the best one's first action (argmin -- no softmax averaging, since averaging
discrete actions is meaningless). These helpers just produce the candidate
sequences, either exhaustively (short horizon, small action set) or by random
sampling warm-started around last cycle's plan.
"""

import numpy as np


def enumerate_action_sequences(n_actions, horizon):
    """All action-index sequences of length `horizon` over `n_actions` actions,
    as an (n_actions**horizon, horizon) int array. Use only when that count is
    small (see Variant2Config.exhaustive_cap)."""
    grids = np.meshgrid(*[np.arange(n_actions)] * horizon, indexing="ij")
    return np.stack([g.ravel() for g in grids], axis=1)


def sample_action_sequences(n_actions, horizon, samples, rng, nominal=None,
                            keep_bias=0.5, hold=0.75):
    """Random action-index sequences (samples, horizon), in two families:

    * RUN-STRUCTURED (all rows): each hop repeats the previous hop's action
      with probability `hold`, else redraws uniformly -- sequences are a few
      sustained segments (expected run length 1/(1-hold)), the discrete
      analogue of variant 1's AR(1)-smoothed noise. Per-hop-uniform sampling
      produced Brownian jitter that never held a strafe long enough to thread
      a narrow passage at longer horizons.
    * NOMINAL-MUTATED (half the rows, when a time-shifted `nominal` is given):
      elements kept from the nominal with probability keep_bias, else
      redrawn -- the warm start that concentrates candidates around last
      cycle's plan. Row 0 is exactly the nominal so it is always evaluated.
    """
    seqs = np.empty((samples, horizon), dtype=np.int64)
    seqs[:, 0] = rng.integers(0, n_actions, size=samples)
    for h in range(1, horizon):
        redraw = rng.random(samples) >= hold
        seqs[:, h] = np.where(redraw,
                              rng.integers(0, n_actions, size=samples),
                              seqs[:, h - 1])
    if nominal is not None:
        half = samples // 2
        keep = rng.random((half, horizon)) < keep_bias
        seqs[:half] = np.where(keep, nominal[None, :], seqs[:half])
        seqs[0] = nominal
    return seqs
