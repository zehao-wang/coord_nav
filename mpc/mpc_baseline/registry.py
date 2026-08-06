"""Registry of selectable policy backends.

The GUI / CLI / offline eval enumerate this to let the operator switch policies.
Each entry:
  label        : human name for the dropdown
  action_space : "velocity" | "discrete"  -- MUST equal the Policy's own
                 `action_space`; it decides which actuator the runner binds
                 (velocity -> /drive_wheels, discrete -> /drive_action)
  build(magnitude, goal_x, goal_y=0.0, step_duration=0.5, allow_rotation=False)
                 -> (Policy, cfg). The GUI/CLI always pass ALL of those keyword
                 arguments, so build() must accept all of them.

Add a new backend with `register(...)` -- no GUI/runner changes needed, as long
as the policy implements the Policy interface (see the carpolicy package):

    from mpc_baseline.registry import register
    register("my_model", "My model", "velocity", my_build)

`register` validates the signature immediately and `build_policy` re-checks that
the policy's declared action space matches the entry, so a typo fails at
registration or at build time instead of driving the wrong topic on the car.
Assigning into POLICY_REGISTRY directly still works; the build-time checks apply
either way because everything constructs policies through `build_policy`.
"""

import inspect

from . import config
from .policies import make_policy

ACTION_SPACES = ("velocity", "discrete")

# The exact keyword arguments the GUI/CLI pass to every build().
BUILD_KWARGS = {"goal_y": 0.0, "step_duration": 0.5, "allow_rotation": False}

POLICY_REGISTRY = {}


def _check_build_signature(key, build):
    """Fail now, not when the operator hits Execute."""
    if not callable(build):
        raise TypeError("policy %r: build must be callable, got %r" % (key, type(build)))
    try:
        inspect.signature(build).bind(40.0, 1.0, **BUILD_KWARGS)
    except TypeError as exc:
        raise TypeError(
            "policy %r: build() must accept (magnitude, goal_x, goal_y=..., "
            "step_duration=..., allow_rotation=...) -- the GUI/CLI always pass all of "
            "them. Got %s: %s" % (key, inspect.signature(build), exc))


def register(key, label, action_space, build):
    """Register a policy backend, validating the entry up front. Returns it."""
    if action_space not in ACTION_SPACES:
        raise ValueError("policy %r: action_space must be one of %s, got %r"
                         % (key, ACTION_SPACES, action_space))
    _check_build_signature(key, build)
    POLICY_REGISTRY[key] = {"label": label, "action_space": action_space,
                            "build": build}
    return POLICY_REGISTRY[key]


def build_policy(key, magnitude, goal_x, goal_y=0.0, step_duration=0.5,
                 allow_rotation=False):
    """Construct a registered policy: returns (Policy, cfg).

    THE single construction path for the GUI, the live CLIs and the offline eval,
    so the consistency checks below cannot be bypassed by one caller.
    """
    if key not in POLICY_REGISTRY:
        raise KeyError("unknown policy %r; registered: %s"
                       % (key, ", ".join(sorted(POLICY_REGISTRY)) or "(none)"))
    entry = POLICY_REGISTRY[key]
    _check_build_signature(key, entry["build"])
    policy, cfg = entry["build"](magnitude, goal_x, goal_y=goal_y,
                                step_duration=step_duration,
                                allow_rotation=allow_rotation)

    declared = entry.get("action_space")
    actual = getattr(policy, "action_space", None)
    if actual != declared:
        raise ValueError(
            "policy %r: registry says action_space=%r but %s.action_space=%r. "
            "The runner binds the actuator from the POLICY while the GUI dropdown "
            "shows the registry value, so a mismatch silently drives the wrong "
            "topic (velocity -> /drive_wheels, discrete -> /drive_action). Make "
            "them equal." % (key, declared, type(policy).__name__, actual))
    if actual not in ACTION_SPACES:
        raise ValueError("policy %r: %s.action_space must be one of %s, got %r"
                         % (key, type(policy).__name__, ACTION_SPACES, actual))
    return policy, cfg


def _mpc_build(variant, predict=False):
    def build(magnitude, goal_x, goal_y=0.0, step_duration=0.5, allow_rotation=False):
        cfg = config.build_live_cfg(variant, magnitude, goal_x, goal_y=goal_y,
                                    step_duration=step_duration,
                                    allow_rotation=allow_rotation)
        cfg.predict_obstacles = predict
        return make_policy(variant, cfg), cfg
    return build


register("mpc_grid", "MPC grid (variant 2, baseline)", "discrete", _mpc_build(2))
register("mpc_vw", "MPC velocity v,omega (variant 1, sampling)", "velocity", _mpc_build(1))
# The TIME-AWARE ablations: identical sampler / cost weights / plant -- the ONE
# difference is predict_obstacles=True, i.e. the obstacle cost scores rollout
# step h against the field's constant-velocity prediction at t+h*dt (the
# standard dynamic-obstacle MPC baseline) instead of the frozen current frame.
# In a static world the velocity gates read 0 and these degenerate EXACTLY to
# the plain variants (regression-tested), so one number isolates "considers
# obstacle motion" as the only experimental variable.
register("mpc_grid_t", "MPC grid + CV prediction (variant 2t)", "discrete",
         _mpc_build(2, predict=True))
register("mpc_vw_t", "MPC v,omega + CV prediction (variant 1t)", "velocity",
         _mpc_build(1, predict=True))


def _orca_build(magnitude, goal_x, goal_y=0.0, step_duration=0.5,
                allow_rotation=False):
    # imported lazily so mpc_baseline works without the rvo2 bindings unless
    # the ORCA baseline is actually built
    from .orca_policy import ORCAPolicy
    cfg = config.build_live_cfg(2, magnitude, goal_x, goal_y=goal_y,
                                step_duration=step_duration)
    return ORCAPolicy(cfg), cfg


register("orca", "ORCA / RVO2 (reactive crowd baseline)", "discrete", _orca_build)
