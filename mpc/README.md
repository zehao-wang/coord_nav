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

当前基线（**20 seeds × 6 场景 = 120 局**，标定后的被控对象模型）：

| | 成功 | 碰撞 |
|---|---|---|
| 变种2 `mpc_grid` | 0.833 | 0.000 |
| 变种1 `mpc_vw`（内置**紧**场景） | **0.983** | **0.000** |
| 变种1 `mpc_vw`（**真实**场景 B=3m、障碍 1.2–1.5m） | **1.000** | 0.000 |

> **别只看 seed 0。** 单 seed 会给出 5/6 这种数字,而 120 局的真实值完全不同 —— 这个坑我们踩过。
>
> 变种1 在内置紧场景（障碍 0.5m、B=1m）失败**不是退化**：那些场景要求的转弯半径超出车的物理能力。
> 把 `steer_arm` 改回标定前的虚构值 0.10 就能复现旧的 0.750,说明旧数字是仿真在给它做真车做不到的急转。
> 在实际驱动的场景里标定后是 1.000 / 0 碰撞。
>
> ⚠️ 仿真的"真值"仍然是 planner 自己用的积分器（`rollout_body`），**无噪声、无延迟**
> （`KinematicSim` 的 `noise_xy` / `dropout` 参数 `eval.run_variant` 从不设置）。
> 控制周期已经和实车统一（都是一个 tick），但这里的数字仍只说明"规划逻辑自洽 + 模型已标定"。

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
from mpc_baseline.registry import register
from mpc_baseline.policies import MyPolicy   # 或 from your_module import MyPolicy

def _my_build(magnitude, goal_x, goal_y=0.0, step_duration=0.5, allow_rotation=False):
    # build_live_cfg 的第一个参数选变体的默认 cfg："1"=velocity(Variant1Config)/"2"=discrete；按你的 action_space 选
    cfg = config.build_live_cfg("1", magnitude, goal_x, goal_y=goal_y)
    return MyPolicy(cfg), cfg

register("my_model", "My model", "velocity", _my_build)
```
`register()` 当场校验 `build` 签名和 `action_space` 取值；`build_policy()`（GUI/CLI/eval 唯一的构造入口）
再校验**你的 `Policy.action_space` 和注册项一致**——写反了会立刻报错，而不是在实车上驱动错的话题。

**③ 完事**——GUI 下拉框自动多出 "My model"，选中即用；`smoke/policy_run.py` / GUI 的录制、可视化、安全都照旧。
**离线也能直接跑你的模型**（上车前先在这验证）：
```bash
python scripts/run_sim.py --policy my_model --all                # 全场景表
python scripts/run_sim.py --policy my_model --scenario slalom --plot /tmp/m.png
python scripts/benchmark.py --policy my_model                    # 和两个 baseline 同表对比
python scripts/benchmark.py --policy my_model --plan-dt 0.25     # 按实车节奏比
```
```python
from mpc_baseline import eval as E
rs = E.run_policy("my_model", magnitude=40)     # -> [EpisodeResult]，可喂 print_table / to_json
```
> `--policy` 用 registry 建的 **live** 配置，`--variant 1|2` 用 **sim** 配置，两者数字不可直接比；
> 要对齐加 `--live-profile`。另外 `run_variant()` 只接受 `1`/`2`/registry key，其它一律报错
> （以前会静默跑成 variant 2，给你别人的分数）。

> **discrete policy 的一个额外要求**：离线 `run_episode` 需要一个 `RobotConfig` 才能把
> `(action_id, magnitude)` 换算成机体速度。走 `run_policy` / `--policy` 会自动带上；自己直接调
> `run_episode` 时传 `robot_cfg=cfg.robot`。velocity policy 不需要。

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
| `TickConfig.rate_hz` | 3.0 | **全局唯一频率**，必须等于车端感知率（`viz.launch` 的 `obstacle_circles rate_hz`）|
| `TickConfig.action_ticks` | 1.5 | 一条指令在车上的存活时长（tick 数）。>1 使单次丢包不抖动，<2 使连丢两次后刹车 |
| `GoalConfig.goal_dist / goal_y` | 1.0 / 0.0 | B 相对起点：前 x、左 y（米）；GUI/实验默认用 3.0 |
| `LiveConfig.magnitude` | 20 | 实车 PWM 幅值。**摩擦阈值 14 PWM**，所以 mag 20 只有 v_max≈0.08 m/s（几乎不动）；**实用值 40**（≈0.36 m/s），GUI 默认也是 40 |
| `LiveConfig.collision_abort` | True | 逼近碰撞软停。**GUI 现在默认开**——曾默认关，结果一次实车跑把 130mm footprint 的 126mm 开进了障碍圆里 1 秒。软停不杀 car-ros |
| `RobotConfig.pwm_per_mps / pwm_offset` | 72.1 / 14.0 | **实测**仿射被控对象 `轮速=(PWM−14)/72.1`。见下面的标定小节 |
| `RobotConfig.wz_arm`, `Variant1Config.steer_arm` | 0.196 | **实测**偏航臂（米），使指令 w == 实际偏航率 |
| `Variant1Config.w_max` | 1.2 | 偏航上限 → 最小转弯半径 = v/w：mag 40 → **0.30 m** |
| `MPPIConfig.horizon / dt` | 29 / 1/3 | `dt` **就是一个 tick**（执行的正是第一步）；H·dt ≈ 9.7 s 前视，mag 40 下 ≈3.5 m |
| `MPPIConfig.noise_tau` | 0.8425 | 采样噪声的 AR(1) 时间常数（**秒**，不是每步），改 tick 率时机动时长不变 |
| `CostConfig.w_cont` | **0.0(关)** | 惩罚首步与「车正在执行的指令」的差（`w_smooth` 只管 horizon 内部）。离线看着免费，**实车 0/2 到达**（见 CHANGELOG 0.9.3）——仿真无扰动，迟钝不付代价 |
| `CostConfig.w_track` | 4.17 | 回归 A→B 直线的拉力（太大→绕不开线上的宽墙）|
| `CostConfig.extra_margin / obs_buffer` | 0.10 / 0.15 | 障碍膨胀额外边距 / 软墙作用范围 |

> 代价里的 running 项都是**积分**（`权重 × Σ × dt`），所以改 horizon / dt 不会悄悄改变各项配比。
> 权重值是按旧 `dt=0.6` 折算过的，旧时序能逐位复现旧结果。

---

## 标定被控对象模型

planner 的模型必须和真车一致,否则闭环性能靠运气。**被控对象是仿射的,不是比例的**:

```
轮速 (m/s) = (PWM − pwm_offset) / pwm_per_mps        实测 (PWM − 14.0) / 72.1
```

偏航轴上还有**第二个死区**(麦轮偏航要横向刮擦滚子),由 `kinematics.yaw_feedforward` 补偿。

**方法、工具、实测数据、三个坑,全在 [`calibration/README.md`](../calibration/README.md)。**
发指令车不动、或 tick log 的 `real/cmd` 列持续偏离 1.00,就去那里重新标定。

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
- 校验工具：`calibration/calib_gyro.py`（原地转，对比 编码器 vs 陀螺 vs 激光）。

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
