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
                            keep_bias=0.5):
    """Random action-index sequences (samples, horizon). If a `nominal` sequence
    (horizon,) is given, each element is kept from the (time-shifted) nominal with
    probability keep_bias and otherwise resampled uniformly -- a cheap warm start
    that concentrates candidates around last cycle's plan. Row 0 is exactly the
    nominal so the warm-started plan is always evaluated."""
    seqs = rng.integers(0, n_actions, size=(samples, horizon))
    if nominal is not None:
        keep = rng.random((samples, horizon)) < keep_bias
        seqs = np.where(keep, nominal[None, :], seqs)
        seqs[0] = nominal
    return seqs
