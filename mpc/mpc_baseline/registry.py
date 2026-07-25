"""Registry of selectable policy backends.

The GUI / CLI enumerate this to let the operator switch policies. Each entry:
  label        : human name for the dropdown
  action_space : "velocity" | "discrete"  (informational; the Policy declares it)
  build(magnitude, goal_x, goal_y, ...) -> (Policy, cfg)
                 constructs a ready policy plus its config for the given goal B
                 (goal_x forward, goal_y left, metres, relative to the start pose).

Add a new backend by registering one entry -- no GUI/runner changes needed, as
long as the policy implements the Policy interface (policy.py).
"""

from . import config
from .policies import make_policy


def _mpc_build(variant):
    def build(magnitude, goal_x, goal_y=0.0, step_duration=0.5, allow_rotation=False):
        cfg = config.build_live_cfg(variant, magnitude, goal_x, goal_y=goal_y,
                                    step_duration=step_duration,
                                    allow_rotation=allow_rotation)
        return make_policy(variant, cfg), cfg
    return build


POLICY_REGISTRY = {
    "mpc_grid": {"label": "MPC grid (variant 2, baseline)",
                 "action_space": "discrete", "build": _mpc_build(2)},
    "mpc_vw":   {"label": "MPC velocity v,omega (variant 1, sampling)",
                 "action_space": "velocity", "build": _mpc_build(1)},
}
