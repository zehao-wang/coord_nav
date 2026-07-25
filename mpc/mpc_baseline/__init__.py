"""mpc_baseline - MPC baseline policy for the Jetson mecanum car (workstation side).

Two variants of a receding-horizon go-around-the-obstacle policy that takes the
car straight ahead to a goal B (the pose 5 s of forward driving reaches) while
avoiding lidar obstacle circles:

  * Variant 2 (DEFAULT baseline, discrete grid-hop) -- sampling/enumeration MPC
    over the mecanum action set; actuates via /drive_action. This is the policy
    the car is mainly tested with.
  * Variant 1 (continuous v, omega) -- continuous (v,w) SAMPLING MPC (DWA): the
    same sampling core as variant 2, just a continuous action space; actuates via
    a per-cycle /drive_wheels velocity pulse (car-side keep-alive).

Both variants implement the shared `Policy` interface (the `carpolicy` package:
Observation -> Action) -- MPC is just one Policy; learning-based policies
implement the same interface and drop into the same runner/GUI. The
planner/cost/kinematics are PURE NUMPY (no solver dependency), so `sim` and
`eval` validate the exact planning code offline. The ROS glue lives in
`actuators`/`runner` and is only imported when you drive the real car.
"""

from carpolicy import Policy, Observation, Action

from . import config
from .config import (RobotConfig, MPPIConfig, CostConfig, GoalConfig,
                     Variant1Config, Variant2Config, ObstacleConfig, LiveConfig,
                     sim_config_v1, sim_config_v2, live_config_v1, live_config_v2,
                     build_live_cfg)
from .obstacles import ObstacleField
from .policies import Variant1Policy, Variant2Policy, make_policy
from .registry import POLICY_REGISTRY

__all__ = [
    "Policy", "Observation", "Action",
    "config", "RobotConfig", "MPPIConfig", "CostConfig", "GoalConfig",
    "Variant1Config", "Variant2Config", "ObstacleConfig", "LiveConfig",
    "sim_config_v1", "sim_config_v2", "live_config_v1", "live_config_v2",
    "build_live_cfg", "ObstacleField", "Variant1Policy", "Variant2Policy",
    "make_policy", "POLICY_REGISTRY",
]
