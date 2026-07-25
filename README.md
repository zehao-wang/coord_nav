# coord_nav — Coord_Nav 项目的中间件

小车是一块 **Jetson Nano**（Ubuntu 18.04 / ROS **Melodic**），跑 roscore + 感知/控制,**感知与底层控制都在车上算**。这台 **workstation**（Ubuntu 22.04 / ROS **Noetic**,conda 环境 `ros1`）是纯 ROS 客户端。

```
  Workstation 10.42.0.1 ──WiFi coord_nav──> Car 10.42.0.187 (jetson-desktop)
  ROS Noetic client                          ROS Melodic + roscore + 传感器/控制
```

| 项 | 值 |
|---|---|
| 热点 | `coord_nav`（开放 2.4G），网关=workstation `10.42.0.1` |
| 小车 | 固定 IP `10.42.0.187`，主机名 `jetson-desktop`，SSH `jetson@`（已装免密公钥）|
| ROS master | 在**小车**上 `http://10.42.0.187:11311` |

---

## 首次配置（一次性）

1. **建 ROS 环境**（工作站用 conda 装 ROS Noetic，RoboStack）：
   ```bash
   conda create -n ros1 python=3.11 -y && conda activate ros1
   conda install -c conda-forge -c robostack-staging ros-noetic-desktop -y
   ```
2. **改 `car_env.sh` 顶部 config 块**为本机：`CONDA_ROOT`、`SELF_IP`、`CAR_IP`。
3. **装 Python 包**——在**仓库根目录**执行：
   ```bash
   conda activate ros1 && pip install -e carclient -e carpolicy -e mpc
   ```
   > ⚠️ **`-e` 每个路径前都要写一次**，否则只会装上第一个。
   > 自检（不要在仓库根目录跑）：`cd /tmp && python -c "import carclient, carpolicy, mpc_baseline"`
4. **`/etc/hosts` 加一行**（少了会"能 list 但收不到数据"）：`10.42.0.187 jetson-desktop`
5. *（可选）* `~/.bashrc` 里加 `roscar(){ source /绝对路径/car_env.sh; }`。

## 每次使用

```bash
sudo nmcli connection up jetson-ap   # 1) 工作站开热点；小车上电连 coord_nav
ping 10.42.0.187                     #    小车在线？
source car_env.sh                    # 2) 进 ros1 + 指向小车 master（= roscar，每个新终端一次）
rostopic list                        #    应看到 /obstacles /odom /scan ...
bash gui/run_gui.sh                  # 3) 开 GUI（用当前 $DISPLAY）
```

连不上先 `bash smoke/smoke_test.sh` 分层自检，再看 [排查](docs/troubleshooting.md)。

---

## 我该看哪个文档

| 我想… | 去 |
|---|---|
| **写自己的 A→B 绕障模型** | [`mpc/README.md`](mpc/README.md) — 实现 `carpolicy.Policy`、注册一行、自动出现在 GUI 里 |
| **读感知 / 发动作**（裸接口） | [`carclient/README.md`](carclient/README.md) — API、话题、动作 id |
| **标定小车模型** | [`calibration/README.md`](calibration/README.md) — PWM↔速度、偏航臂、死区；**发指令车不动先看这里** |
| **改感知算法** | [`obstacle_perception/README.md`](obstacle_perception/README.md) — C++ 节点 + 一键部署 |
| **车端服务 / 安全 / 网络** | [`docs/car-setup.md`](docs/car-setup.md) |
| **出问题了** | [`docs/troubleshooting.md`](docs/troubleshooting.md) |
| **为什么是现在这样** | [`CHANGELOG.md`](CHANGELOG.md) — 记录每个决定背后的实测数据 |

---

## 目录

| 路径 | 作用 |
|---|---|
| `car_env.sh` | ROS 环境 + 网络变量（被 `roscar` 引用）|
| `carclient/` | **Python API 包**：`CarClient` / `Action` |
| `carpolicy/` | **通用 Policy 接口包**：`Policy` / `Observation` / `Action` |
| `mpc/` | **A→B 绕障 policy 框架 + 两个 MPC baseline**（包名 `mpc_baseline`）|
| `calibration/` | 标定被控对象模型的工具、方法、历史记录 |
| `gui/` | PySide6 上位机（`run_gui.sh` 启动）|
| `obstacle_perception/` | C++ 障碍感知 ROS 包（部署到 Nano）|
| `car_ros/` | 车端在用的源码副本（部署到 `car_base/`）|
| `smoke/` | 连通性 / 单轮测试 + 实车实验 `policy_run.py` |
| `docs/` | 车端配置、排查 |
| `output/` | GUI 每次 Execute 的录制 + tick log（gitignored）|

---

## 三件必须知道的事

1. **MCU 对电机没有超时。** 丢一条"停止"就可能锁存前进跑飞,ROS 层停不住 —— 只有 `estop.sh` 能停。所有驱动路径都带车端保活 + 到时刹车,**空格键 = E-STOP**。细节见 [`docs/car-setup.md`](docs/car-setup.md)。
2. **电机有 ~14 PWM 的静摩擦阈值,17.4 以下完全不能转向。** 默认 `magnitude` 是 **40**(v_max 0.36 m/s、转弯半径 0.30 m),这是实车验证过的值;20 只有 0.33 的成功率。见 [`calibration/`](calibration/README.md)。
3. **全局只有一个频率**:`TickConfig.rate_hz = 3.0`,等于车端感知率。一帧观测 = 一个 tick,规划、执行、GUI 渲染全部跟着它走。
