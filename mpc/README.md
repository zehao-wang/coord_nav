# mpc — 麦轮小车 A→B 绕障 policy（工作站侧）

小车从 **A 到 B**、绕开路上的障碍。跑在 **workstation**（`ros1` 环境），只读雷达碰撞球、发驱动指令；小车侧不用改。

- **任务**：B = 从当前位置**正前方几米**的一点（默认 3m，相对起点的坐标 `(前 x, 左 y)`）。直线上可能有障碍，要绕过去到 B。
- **observation**：`/obstacles` 的碰撞圆 —— 每个障碍 `(x, y, r)`（米，base 系，x 前 y 左），就是 `carclient.obstacles().circles`，和 GUI 俯视图同一份数据，**~3Hz**。
- **四个内置 policy**（都连续操控、到点前不停）：
  | key | 名字 | 控制 | 走哪个话题 |
  |---|---|---|---|
  | `mpc_grid` | 变种2（离散 baseline，**冻结世界**） | 离散"跳格子"（8 个麦轮平移方向采样，取代价最小） | `/drive_action` |
  | `mpc_vw` | 变种1（连续速度，**GUI 默认**，冻结世界） | 连续 `(v, ω)` **采样 MPC / DWA** | `/drive_wheels` 速度脉冲 |
  | `mpc_grid_t` | 变种2t（**时序**：恒速预测） | 同变种2，障碍按估计速度外推 | `/drive_action` |
  | `mpc_vw_t` | 变种1t（时序：恒速预测） | 同变种1，障碍按估计速度外推 | `/drive_wheels` 速度脉冲 |

> 四个是**同一个采样内核**，两个维度各二选一：动作空间（离散 vs 连续 (v,ω)）×
> 障碍模型（**冻结当前帧** vs **`_t` 时序版**——`ObstacleField` 从连续帧差分出每个障碍的
> 恒速估计,rollout 第 h 步对着障碍在 t+h·dt 的预测位置打分,即动态障碍 MPC 文献里的标准
> CV-prediction 基线）。`_t` 与原版**只差这一个变量**（静态世界里逐字节等价,有回归测试），
> 是干净的受控消融。采样 K 条控制序列 → rollout → 打分 → argmin → 暖启动。无 NLP、无兜底规划器。

---

## 快速开始

### 离线（不用车，现在就能跑）
```bash
roscar && cd mpc
python scripts/run_sim.py --variant 1 --scenario slalom --plot /tmp/v1.png   # 单场景+画图
python scripts/benchmark.py --json /tmp/bench.json          # ↓ 下表紧套件两行，原样复现
python scripts/benchmark.py --suite realistic --disturbed   # 真实机制 + 执行扰动
python scripts/benchmark_table.py                           # 更新正式基准榜（见下）
```

> **正式基准榜在 [`BENCHMARKS.md`](BENCHMARKS.md)**（`benchmarks.json` 为机器可读源）：
> 固定 eval protocol + 固定分级 set（L1 静态-常规 → L4 动态-复合,同 seed、全部实车忠实
> 模式、registry 即插即用契约构造）,**现在和未来的每个 policy 都评在这一张表里**。
> 协议带版本号,行记 commit/日期;全程确定性 → `benchmark_table.py --check KEY` 即回归
> 检测。本 README 下面的表是历史演进的说明性数字（sim profile / 分套件）,横向比较以榜为准。

当前基线（标定后的被控对象模型；**上面的命令原样复现下表** —— `--seeds` 默认 20，
紧套件 20×6=120 局、真实套件 20×4=80 局；`--seeds 1` 才是单 seed 快看）：

| 套件 | 模式 | 变种2 `mpc_grid` 成功/碰撞 | 变种1 `mpc_vw` 成功/碰撞 |
|---|---|---|---|
| 紧（内置） | 默认 | 0.833 / **0.000** | 0.717 / 0.283 |
| 紧（内置） | `--disturbed` | **1.000 / 0.000** | 0.442 / **0.533** |
| 真实 `--suite realistic` | 默认 | **1.000 / 0.000** | **1.000 / 0.000** |
| 真实 `--suite realistic` | `--disturbed` | **1.000 / 0.000** | 0.938 / 0.037 |

> **碰撞 = 障碍表面碰到车身足迹**（0.9.15 起），和实车 guard 同一判据。旧表用的是
> "足迹**圆心**进圈"才算碰 —— 障碍表面嵌进车身 12cm 仍记"成功"，v1 紧套件 0.9.15 前的
> 0.992/0.008、0.767/0.167 换成诚实的尺子并叠上 0.9.16 刹车语义、0.9.18 w_track_l1
> 之后就是上表这一列（不是退化，是尺子和默认参数的演进,CHANGELOG 有逐步数字）；
> v2 在两种判据下都是 0 接触，结论反而更硬。

> **`--disturbed` 是实车忠实评估**:实测执行扰动(偏航一阶滞后 τ=0.48s + 速度噪声,
> 从 298 个实车 tick 拟合)+ runner 的缓冲 tick 循环,离散策略也按实车 1/3s tick
> 执行与 rollout(0.9.15 前这条路径误用 0.5s 的 2Hz 节奏,车上从不存在)。**任何平滑性/
> 稳健性调参必须用它筛** —— 完美执行的默认仿真曾把 w_cont 误判为免费(实车 0/2)。扰动下
> 变种1 因偏航滞后在紧场景剐障碍(0.533),这是"偏航跟踪 0.6"的可测代价;全向平移的
> 变种2 不吃这个滞后,是扰动下更稳健的 baseline。

> **v2 的"默认"两行是确定性的**:动作集穷举(8⁴ ≤ cap)不抽 RNG、无扰动仿真无噪声,
> 20 个 seed 逐字节相同 —— 紧套件 0.833 的有效样本量是 6(wall_inline 恒超时,其余恒成功),
> 别当成 120 局的统计量;`--disturbed` 行才真正随 seed 变。v1 两种模式都随 seed 变。
> **别只看 seed 0**(`--seeds 1`):单 seed 和 120 局的数字可以差很远,这个坑踩过。
>
> 变种1 在内置紧场景（障碍 0.5m、B=1m）失败**不是退化**：那些场景要求的转弯半径超出车的
> 物理能力,套件是刻意的压力测试。**真实驾驶机制**(B=3m、障碍 1.2–1.5m)即
> `--suite realistic`,0.9.15 起已提交进仓库(`sim.realistic_scenarios` —— 此前这一行
> 出自从未提交的临时场景,别人无法复现)：足迹判据下仍是 1.000 / 0 接触,扰动下 0.938/0.037。
>
> ⚠️ **默认**（不带 `--disturbed`）仍是完美执行的世界:planner 的积分器就是"真值",
> 无执行噪声、无缓冲延迟 —— 这些数字只说明"规划逻辑自洽 + 模型已标定"。
> 要评估稳健性,**用 `--disturbed`**(见上)。感知噪声(`noise_xy`/`dropout`)目前
> 两种模式都未启用,是扰动模型的已知余项。
>
> 回归测试:`cd mpc && python -m pytest tests/ -q`(seed 贯通、足迹碰撞判据、disturbed
> 的 tick 对齐 —— 每一条都是踩过一次的坑)。

### 测试场 TestField（动态障碍 + 遮挡，输入与真机一致）
```bash
python scripts/run_field.py                     # 两个冻结世界 baseline：5 原型 + 10 随机 × 20 seeds
python scripts/run_field.py --policy mpc_grid_t --policy mpc_vw_t   # 加时序版同表对比
python scripts/run_field.py --policy my_model   # 你的模型即插即用（registry 注册后）
python scripts/run_field.py --anim /tmp/anims   # 每 case 逐帧动画（gif；--anim-fmt mp4）
python scripts/run_field.py --mem 0             # 只用当前帧规划（也会关掉 _t 的速度估计）
```

`mpc_baseline/testfield.py`。场地**默认实车忠实**（和 benchmark 默认完美执行相反）：
喂给 policy 的输入和 `PolicyRunner` 实车同构 —— 同一个 `Observation` 契约与
`ObstacleField` 记忆代码、3Hz tick、缓冲一拍派发 + 死区补偿、实测执行扰动，外加
**遮挡**（到障碍圆心的射线被别的障碍挡住就看不见 —— 旧仿真是透视的）。动态障碍是
pymunk 刚体（弹性互撞/撞墙反弹，随机 case 物理自洽、seed 可复现）；车不进物理引擎，
仍走标定的运动学。碰撞判据与静态套件/实车 guard 相同。`--perfect-exec` 关扰动（只用于
调试）；感知噪声 `--noise-xy/--dropout` 是**未标定旋钮**，默认 0（可从 run.json 的实录
帧拟合，待做）。已知与真机的差距：位姿无漂移、无扫描-位姿时间戳偏斜。

四个 baseline 在测试场（15 cases × 20 seeds = 300 局/策略，实车忠实模式，
`--seeds/--random/--rand-seed` 默认即协议，成功/碰撞；下表是 0.9.17 时点的
说明性数字 —— 0.9.20 的感知加固闸门略微改变 `_t` 行为，**现行数字以
[`BENCHMARKS.md`](BENCHMARKS.md) 为准**）：

| | 冻结世界 | **`_t` 时序（恒速预测）** |
|---|---|---|
| 变种2 grid | 0.740 / 0.260 | **0.977 / 0.023** |
| 变种1 vw | 0.553 / 0.410 | **0.873 / 0.110** |

> 逐 case 看：`mpc_grid_t` 在 **15 个 case 里 13 个零碰撞**（拦截型 `cross_slow`
> 0.85→0.00、`diagonal` 0.65→0.00、`occluded_oncoming` 0.20→0.00 —— coast 预测让
> 被遮挡的 mover 在记忆里继续走,穿遮挡跟踪）。残余碰撞（rand03 0.20、rand07 0.15）
> 全是 mover **中途弹墙**的 case —— 恒速模型的固有边界,如实保留。
>
> 三个结论：**(1) 冻结世界的 baseline 对拦截型动态障碍不稳健**（`cross_slow` v2 碰
> 0.85、v1 全灭）——这是静态套件盖住的能力缺口。**(2) 有了速度跟踪,1.5s 障碍记忆从
> 负资产变回正资产**：`_t` + `--mem 0` 退化回原版数字（无记忆→无法跨帧差分速度;
> grid_t 带记忆碰 0.023 vs 无记忆 0.163）——0.9.16 发现的"记忆尾迹"问题由 coast 关联
> 根治,不再需要牺牲记忆。**(3) 时间线必须对齐**：缓冲派发差一拍 + EMA 位置滞后一拍,
> 两个"一拍"曾吃掉大半收益（`pred_extra_delay_s` + raw 观测外推修正,见 CHANGELOG
> 0.9.17）。学习型 policy 若要超过 `_t`,得赢在恒速模型失效处（弹墙、变速、意图）。

### GUI（推荐，实车）
```bash
bash gui/run_gui.sh          # 连车的 master，用当前 $DISPLAY（先按顶层 README 完成首次配置）
```
选 policy（下拉，默认 **velocity**）→ 设 B 前向 x / 左 y（米）→ (可选) **execute steps**（默认 1，见下）→ **Execute**。俯视图画障碍球 + MPC 预测轨迹 + 目标 + 车。**每次跑完**自动存到 `output/<日期_时间>/`：`observation.mp4`（3Hz 观测视频）+ `trajectory.png` + `run.json`。空格 = E-STOP。

### 命令行实车 / 单次实验
```bash
roscar
python ../smoke/policy_run.py --variant 1 --bx 3 --by 0 --pose odom --mag 40   # 跑一次+存图+陀螺校验
python ../smoke/policy_run.py --variant mpc_grid_t --bx 3 --mag 40             # --variant 接受任意 registry key
python scripts/run_live.py --variant 2 --magnitude 40                          # 变种2 baseline
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
python scripts/benchmark.py --policy my_model --plan-dt 0.333    # 按实车节奏比
python scripts/run_field.py --policy my_model                    # 动态障碍测试场（见上）
python scripts/benchmark_table.py --policy my_model              # 上正式基准榜（BENCHMARKS.md）
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
| `LiveConfig.magnitude` | **40** | 实车 PWM 幅值 → v_max 0.361 m/s、可用偏航到满 1.2 rad/s、转弯半径 0.30 m。**实车验证过的就是 40**（五次 5m 跑全部到达）。**摩擦阈值 14 PWM**，**17.4 以下偏航恒为 0**（`build_live_cfg` 会拒绝）。紧场景 20 seeds：mag 20 → **0.333**（全是超时，太慢转不动）／30 → 0.992／40 → 0.975（幅值扫描是 0.9.15 前的圆心碰撞判据，横向比较仍有效，别当安全数字引用） |
| `LiveConfig.collision_abort` | True | 逼近碰撞软停。**GUI 和 `smoke/policy_run.py` 现在都默认开**（policy_run 曾漏在"默认关"——README 推荐的实车命令悄悄复现事故配置，0.9.15 修正；`--no-guard` 显式关）——曾默认关，结果一次实车跑把 130mm footprint 的 126mm 开进了障碍圆里 1 秒。软停不杀 car-ros |
| `RobotConfig.pwm_per_mps / pwm_offset` | 72.1 / 14.0 | **实测**仿射被控对象 `轮速=(PWM−14)/72.1`。见下面的标定小节 |
| `RobotConfig.wz_arm`, `Variant1Config.steer_arm` | 0.196 | **实测**偏航臂（米），使指令 w == 实际偏航率 |
| `Variant1Config.w_max` | 1.2 | 偏航上限 → 最小转弯半径 = v/w：mag 40 → **0.30 m** |
| `MPPIConfig.horizon / dt` | 29 / 1/3 | `dt` **就是一个 tick**（执行的正是第一步）；H·dt ≈ 9.7 s 前视，mag 40 下 ≈3.5 m |
| `MPPIConfig.noise_tau` | 0.8425 | 采样噪声的 AR(1) 时间常数（**秒**，不是每步），改 tick 率时机动时长不变 |
| `CostConfig.w_cont` | **0.0(关)** | 惩罚首步与「车正在执行的指令」的差（`w_smooth` 只管 horizon 内部）。离线看着免费，**实车 0/2 到达**（见 CHANGELOG 0.9.3）——仿真无扰动，迟钝不付代价 |
| `CostConfig.w_track` | 4.17 | 回归 A→B 直线的**二次**拉力（太大→绕不开线上的宽墙）|
| `CostConfig.w_track_l1` | **1.0** | **线性**回线拉力：二次项在线附近梯度趋零,障碍走后车在线边"游荡"收不了尾;L1 恒定拉力治这个,远处又比二次弱、不跟宽绕行打架。0.9.18 筛选:测试场尾段离线距 −15/−20%,realistic 不变,紧压力套件 ~1pp 代价 |
| `CostConfig.w_dir_hist` | 0.0 | 离散变体"首跳 vs 正在执行方向"的 (1−cos) 转向罚。**默认 0** 保消融干净;`mpc_grid_t` 实车部署可设 **0.1**(转角 22.9°→16.0°,0.980/0.020,紧套件守门不动)。**别给冻结世界的 `mpc_grid` 开**——它的折回是对跳变世界的应急纠错,罚掉直接 −7pp 成功率(w_dir_seq 更是任何权重都没过筛,弃用) |
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
| `mpc_baseline/sim.py` / `eval.py` | 离线运动学仿真 + 场景 / 指标汇总对比（`resolve_policy` 是唯一的 policy 构造入口）|
| `mpc_baseline/testfield.py` | **测试场**：pymunk 动态障碍 + 遮挡 + 实车忠实闭环，任意注册 policy 即插即用 |
| `scripts/run_field.py` | 测试场 CLI：battery / 随机 case / `--mem` 消融 / 逐帧动画 |
| `mpc_baseline/benchtable.py` / `scripts/benchmark_table.py` | **正式基准榜**：固定协议 + 分级 set，`BENCHMARKS.md`/`benchmarks.json`，`--check` = 回归检测 |
| `mpc_baseline/actuators.py` | `VelocityPulseActuator`(变种1) / `DriveActionActuator`(变种2) |
| `scripts/` `../smoke/` | `run_sim`·`benchmark`·`run_live`·`calibrate_goal` / `policy_run`·`calib_gyro` |

依赖 `carclient`、`carpolicy`（同仓库）。**在仓库根目录**装：

```bash
conda activate ros1 && pip install -e carclient -e carpolicy -e mpc   # -e 每个路径前都要写
```

装完别在仓库根目录用 `python -c` 导包（根目录的 `carclient/`、`carpolicy/` 同名目录会遮蔽已装的包）；
把脚本当文件跑（`python mpc/scripts/run_sim.py`）不受影响。详见顶层 README 排查表。
