# mpc — 麦轮小车 A→B 绕障 policy（工作站侧）

小车从 **A 到 B**、绕开路上的障碍。跑在 **workstation**（`ros1` 环境），只读雷达碰撞球、发驱动指令；小车侧不用改。

- **任务**：B = 从当前位置**正前方几米**的一点（默认 3m，相对起点的坐标 `(前 x, 左 y)`）。直线上可能有障碍，要绕过去到 B。
- **observation**：`/obstacles` 的碰撞圆 —— 每个障碍 `(x, y, r)`（米，base 系，x 前 y 左），就是 `carclient.obstacles().circles`，和 GUI 俯视图同一份数据，**~3Hz**。
- **两个内置 policy**（都连续操控、到点前不停）：
  | key | 名字 | 控制 | 走哪个话题 |
  |---|---|---|---|
  | `mpc_grid` | 变种2（离散 baseline） | 离散"跳格子"（8 个麦轮平移方向采样，取代价最小） | `/drive_action` |
  | `mpc_vw` | 变种1（连续速度，**GUI 默认**） | 连续 `(v, ω)` **采样 MPC / DWA** | `/drive_wheels` 速度脉冲 |

> 两者是**同一个采样内核**，只是动作空间不同（离散动作 vs 连续 (v,ω)）：采样 K 条控制序列 → rollout → 对**所有障碍圆**+目标+A→B 直线打分 → argmin → 暖启动。无 NLP、无兜底规划器。

---

## 快速开始

### 离线（不用车，现在就能跑）
```bash
roscar && cd mpc
python scripts/run_sim.py --variant 1 --scenario slalom --plot /tmp/v1.png   # 单场景+画图
python scripts/benchmark.py --json /tmp/bench.json                            # 全场景对比两变种
```

当前基线（6 个内置场景，`sim_config`，seed 0）：

| | 成功 | 碰撞 | 失败场景 |
|---|---|---|---|
| 变种1 `mpc_vw` | 5/6 (0.833) | 0 | `wall_inline` |
| 变种2 `mpc_grid` | 5/6 (0.833) | 0 | `slalom` |

> ⚠️ 这套 benchmark **不等于实车**，两个已知口径差：
> ① `eval.py` 用 `plan_dt = MPPIConfig.dt = 0.6s` 跑闭环，而实车 runner 是 `1/plan_rate = 0.25s`
>   —— 按实车节奏 + live 配置重跑，变种1 掉到 4/6；
> ② 仿真的"真值"就是 planner 自己用的积分器（`rollout_body`），无噪声、无延迟、无死区畸变
>   （`KinematicSim` 的 `noise_xy` / `dropout` 参数 `eval.run_variant` 从不设置）。
> 所以这里的数字只说明"规划逻辑自洽"，不说明实车成功率。

### GUI（推荐，实车）
```bash
bash gui/run_gui.sh          # 连车的 master，用当前 $DISPLAY（先按顶层 README 完成首次配置）
```
选 policy（下拉，默认 **velocity**）→ 设 B 前向 x / 左 y（米）→ (可选) **execute steps**（默认 1，见下）→ **Execute**。俯视图画障碍球 + MPC 预测轨迹 + 目标 + 车。**每次跑完**自动存到 `output/<日期_时间>/`：`observation.mp4`（3Hz 观测视频）+ `trajectory.png` + `run.json`。空格 = E-STOP。

### 命令行实车 / 单次实验
```bash
roscar
python ../smoke/policy_run.py --variant 1 --bx 3 --by 0 --pose odom --mag 40   # 跑一次+存图+陀螺校验
python scripts/run_live.py --variant 2 --magnitude 20                          # 变种2 baseline
```

---

## 加你自己的模型（3 步，自动出现在 GUI 里）

policy 接口是通用的（`carpolicy` 包），**不是 MPC 专用**——学习型/脚本型/遥操作都能实现同一个基类。

**① 写一个 Policy**（`carpolicy.Policy` 子类，放你自己的模块或直接写进 `mpc_baseline/policies.py`；就两件事：声明动作空间 + 实现 `plan`。字段类型见 `carpolicy/__init__.py` 顶部）：
```python
from carpolicy import Policy, Action

class MyPolicy(Policy):
    action_space = "velocity"            # 或 "discrete"
    def __init__(self, cfg):
        self.cfg = cfg

    def plan(self, obs):
        # obs.pose=(x,y,yaw) 相对起点；obs.goal=(x,y)；
        # obs.circles=[(x,y,r),...] 当前帧障碍(base系)；obs.field=odom系障碍记忆(可选)
        v, w = my_model(obs)             # ← 你的模型
        return Action.velocity(v, w)     # velocity: v 前进 m/s, w 偏航 rad/s
        # 离散则: return Action.discrete(action_id 0..10, magnitude PWM, duration s)

    def reset(self):                     # 清 per-episode 状态（warm start 等）
        pass
```
> ⚠️ **`reset()` 目前没有任何调用方**（`PolicyRunner.run()` 不调，GUI/CLI 也不调）。现在能正常工作
> 是因为 GUI 和各脚本**每次跑都通过 registry 的 `build()` 新建一个 policy 实例**。所以：**不要把
> 同一个 policy 实例复用于多次 run**，否则上一轮的 warm start / A→B 直线锚点会带进下一轮。
> （已记为待修：应由 runner 在 `run()` 开头调用 `policy.reset()`。）

**② 在 `mpc_baseline/registry.py` 注册一行**。`build(magnitude, goal_x, goal_y=0.0, step_duration=0.5, allow_rotation=False)` 是 GUI/CLI 调用的固定签名，返回 `(policy, cfg)`。注册项的 `action_space` **必须和你的 `Policy.action_space` 一致**（它决定 runner 绑哪个执行器：velocity→`/drive_wheels`、discrete→`/drive_action`）：
```python
from mpc_baseline.policies import MyPolicy   # 或 from your_module import MyPolicy

def _my_build(magnitude, goal_x, goal_y=0.0, step_duration=0.5, allow_rotation=False):
    # build_live_cfg 的第一个参数选变体的默认 cfg："1"=velocity(Variant1Config)/"2"=discrete；按你的 action_space 选
    cfg = config.build_live_cfg("1", magnitude, goal_x, goal_y=goal_y)
    return MyPolicy(cfg), cfg

POLICY_REGISTRY["my_model"] = {
    "label": "My model", "action_space": "velocity", "build": _my_build,
}
```

**③ 完事**——GUI 下拉框自动多出 "My model"，选中即用；`smoke/policy_run.py` / GUI 的录制、可视化、安全都照旧。

> 脚本里直接用也行：`PolicyRunner(MyPolicy(cfg), cfg, live, obs_cfg, client, pose_source="odom").run()`（runner 接受任何 Policy 实例，见 `runner.py`）。

**多步执行（可选，和 MPC 一样）**：`plan` 除了返回第一步，还可带上**整段 horizon**——`controls` 是**完整计划**（`controls[0]` 就是那个 `(v,w)`），runner 执行 `controls[0..N-1]`：
```python
return Action.velocity(v, w, controls=best_seq)   # best_seq=[(v,w),...] 完整 horizon（第一项=(v,w)）
# 离散：Action.discrete(id, mag, dur, controls=[(id,mag,dur), ...])
```
runner **默认只执行第一步再重规划**（`execute_steps=1`，紧闭环 = 标准 MPC）；调大就**开环执行 N 步再重规划**(每步仍做碰撞/链路/到点安全检查；N 超过 horizon 长度就提前重规划)。三处可设：GUI 面板的 **execute steps** 框 / `smoke/policy_run.py --exec-steps N` / `LiveConfig.execute_steps`。不带 `controls` 就是普通单步策略。

---

## 关键参数（`mpc_baseline/config.py`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `GoalConfig.goal_dist / goal_y` | 1.0 / 0.0 | B 相对起点：前 x、左 y（米）；GUI/实验默认用 3.0 |
| `LiveConfig.magnitude` | 20 | 实车 PWM 幅值（<30 有死区；GUI 默认 **40**，实车验证过的也是 40）|
| `LiveConfig.collision_abort` | True | 逼近碰撞软停（GUI/smoke 默认**关**，障碍圆自带 margin）|
| `Variant1Config.w_max` | 1.2 | 偏航上限。最小转弯半径 = v/w，**随 magnitude 变**：mag 40 → 0.17m；mag 20 → 0.11m（此时 w 被差速约束先卡在 0.9）|
| `MPPIConfig.horizon / dt` | 16 / 0.6 | 变种1 前视 = H·dt·v，**随 magnitude 变**：mag 40 → ≈1.9m，mag 20 → ≈0.96m（差速靠转向，需比变种2 长；变种2 是 4）|

> ⚠️ **实车不要用 magnitude 20 跑变种1。** `kinematics.velocity_to_wheel_pwm` 的死区补偿是
> **逐轮独立**抬到 `deadzone_pwm=30` 的，会把转向差分抹平。实测指令 vs 实际偏航角速度：
>
> | magnitude（v_max） | w 指令 0.23 | 0.45 | 0.68 | 0.90 |
> |---|---|---|---|---|
> | **20**（0.10 m/s） | 0.00 (0%) | 0.00 (0%) | 0.09 (13%) | 0.15 (17%) |
> | **40**（0.20 m/s） | 0.23 (100%) | 0.45 (100%) | 0.59 (87%) | 0.70 (78%) |
>
> mag 20 时 |w|≲0.45 四个轮子全被抬到 30 PWM → 只会直行（而且 vx 从 0.10 被抬成 0.15）。
> 这是已知待修的模型/执行不一致（planner 不知道这个畸变），修好前实车用 **40**。
| `CostConfig.w_track` | 2.5 | 回归 A→B 直线的拉力（太大→绕不开线上的宽墙）|
| `CostConfig.extra_margin / obs_buffer` | 0.10 / 0.15 | 障碍膨胀额外边距 / 软墙作用范围 |

---

## 安全（小车 MCU 对电机无超时，丢"停"会跑飞）

- **变种2** 走 `/drive_action`，复用车端定时脉冲安全 harness（本地喂看门狗、持续刹车、硬时长上限）。
- **变种1** 走 `/drive_wheels` 速度脉冲：每规划周期发一发，**车端本地 20Hz 保活**（抗 WiFi 丢包），不再流式 `/wheel_cmd`（曾 burst 卡死串口）。
- **runner 层**：MCU 链路掉 → estop；任何异常 / Ctrl-C → estop。GUI 有 **Restart car-ros** 键（E-STOP 后恢复）。

---

## 位姿 / odom（重要）

- **用 `pose_source="odom"`**（编码器位移 + **IMU 陀螺仪 yaw**）。yaw 走陀螺不是编码器差分——差速前进+缓转时编码器 yaw 会严重少报（车实际转 45° odom 只读 7.6°）。
- **不要用 `pose_source="lidar"`**：增量 ICP 在旋转时丢 yaw，是最不准的。
- WiFi 必须低延迟（省电模式会让延迟飙到 3s，闭环失稳）——已做成车的系统默认关闭。
- 校验工具：`smoke/calib_gyro.py`（原地转，对比 编码器 vs 陀螺 vs 激光）。

---

## 文件

| 路径 | 作用 |
|---|---|
| `carpolicy/`（同级包）| **通用 Policy 接口**：`Policy` 基类 + `Observation` + `Action`。加模型只依赖它 |
| `mpc_baseline/policies.py` | `Variant1Policy`（连续采样 MPC）/ `Variant2Policy`（离散）|
| `mpc_baseline/registry.py` | `POLICY_REGISTRY`：GUI/CLI 从这里列出可选 policy（**加模型在这注册**）|
| `mpc_baseline/cost.py` | 共用代价：到 B + 全障碍软墙 + cross-track + 平滑 |
| `mpc_baseline/kinematics.py` / `obstacles.py` / `mppi.py` | 麦轮混合 & rollout / odom 障碍记忆 & 间隙查询 / 采样 |
| `mpc_baseline/runner.py` | 实车闭环 `PolicyRunner`：驱动**任意** Policy，规划/驱动/安全 |
| `mpc_baseline/sim.py` / `eval.py` | 离线运动学仿真 + 场景 / 指标汇总对比 |
| `mpc_baseline/actuators.py` | `VelocityPulseActuator`(变种1) / `DriveActionActuator`(变种2) |
| `scripts/` `../smoke/` | `run_sim`·`benchmark`·`run_live`·`calibrate_goal` / `policy_run`·`calib_gyro` |

依赖 `carclient`、`carpolicy`（同仓库）。**在仓库根目录**装：

```bash
conda activate ros1 && pip install -e carclient -e carpolicy -e mpc   # -e 每个路径前都要写
```

装完别在仓库根目录用 `python -c` 导包（根目录的 `carclient/`、`carpolicy/` 同名目录会遮蔽已装的包）；
把脚本当文件跑（`python mpc/scripts/run_sim.py`）不受影响。详见顶层 README 排查表。
