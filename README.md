# coord_nav — Coord_Nav 项目的中间件

小车是一块 **Jetson Nano**（Ubuntu 18.04 / ROS **Melodic**），跑 roscore + 感知/控制，**所有计算在车上**。这台 **workstation**（Ubuntu 22.04 / ROS **Noetic**，conda 环境 `ros1`）是纯 ROS 客户端：只**读碰撞球**、**发离散动作**。

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
2. **改 `car_env.sh` 顶部 config 块**为本机：`CONDA_ROOT`（你的 conda 路径）、`SELF_IP`（本机在热点上的 IP）、`CAR_IP`（小车 IP）。
3. **装 Python 包**（脚本 / 接模型 / GUI 都需要）——在**仓库根目录**执行：
   ```bash
   conda activate ros1 && pip install -e carclient -e carpolicy -e mpc
   ```
   > ⚠️ `-e` 每个路径前都要写一次。写成 `pip install -e carclient carpolicy mpc` 只会装上
   > `carclient`，后两个会被当成 PyPI 包名去查然后报
   > `ERROR: Could not find a version that satisfies the requirement carpolicy`。
   >
   > 装完自检（**不要在仓库根目录跑**，见下）：
   > ```bash
   > cd /tmp && python -c "import carclient, carpolicy, mpc_baseline; print('ok')"
   > ```
4. **`/etc/hosts` 加一行**（客户端靠主机名收数据，少了会"能 list 但收不到数据"）：`10.42.0.187 jetson-desktop`
5. *（可选）* `roscar` 快捷方式：`~/.bashrc` 里加 `roscar(){ source /绝对路径/car_env.sh; }`，等价于直接 `source car_env.sh`。

## 每次使用

```bash
sudo nmcli connection up jetson-ap   # 1) 工作站开热点；小车上电连 coord_nav
ping 10.42.0.187                     #    小车在线？
source car_env.sh                    # 2) 进 ros1 环境+指向小车 master（每个新终端一次；= roscar）
rostopic list                        #    应看到 /obstacles /odom /scan ...
bash gui/run_gui.sh                  # 3) 开 GUI（用当前 $DISPLAY）
```

`source car_env.sh` = `conda activate ros1` + `ROS_MASTER_URI=小车` + `ROS_IP=本机`。连不上先 `bash smoke/smoke_test.sh` 分层自检。

---

## API（对接模型/脚本）

两件事：**拿碰撞球** 和 **发动作**。推荐用 `carclient`，也可直接用 ROS 话题。

### `carclient`（推荐，简洁鲁棒）

```python
from carclient import CarClient, Action

car = CarClient()                       # 用 roscar 的 ROS 环境连接
car.wait_obstacles(timeout=5)           # 等第一帧

while not car.is_shutdown():            # 典型模型循环
    obs = car.obstacles()               # 非阻塞取最新帧
    if obs and obs.age < 1.0:           # obs.frame_id, obs.circles=[(x,y,r),...], obs.age(秒)
        action = my_model(obs.circles)  # 0..10
        car.drive(action)               # 发一个离散动作（车端闭 20Hz 喂看门狗、到时停）
    car.sleep(1/3.0)                    # ~3Hz
```

- `car.obstacles()` → `Obstacles(frame_id, circles, age)`；`circles` 每个 `(x, y, r)` 米、base 系（x 前 y 左）。非阻塞、线程安全缓存。
- `car.observation()` → `Frame(frame_id, circles, points, age)` —— **一个采样**：`circles` 和 `points` 保证是同一个 `frame_id`（车上同一次处理、同一帧 scan）。**画图用这个**。`points` 为 `None` 表示这帧还没配上点（不会拿别帧的点顶替）。
- `car.scan_points()` → 原始 `/scan` 转 base 系的点，**和圆不同步**（车端按 `rate_hz` 定时取最新 scan，约 60% 的 scan 没参与生成圆）。只在你要原始雷达数据时用。
- `car.drive(action, magnitude=None, duration=None)` → 幅值/时长省略用默认（40 / 0.5s）。
- `car.on_result(cb)` / `car.last_result()` → 每步完成回报 `{"action","reason","took_ms",...}`。
- `car.stop()` 软停；`car.estop()` 硬急停（SSH 跑 estop.sh，杀 car-ros）；`car.connected()` 判活。

### 直接用 ROS 话题（等价）

**读碰撞球** — 订阅 `/obstacles`（`std_msgs/Float32MultiArray`）：
```python
m = rospy.wait_for_message("/obstacles", Float32MultiArray)
frame_id = int(m.data[0])                                   # 第几个采样
circles  = [(m.data[i], m.data[i+1], m.data[i+2]) for i in range(1, len(m.data)-2, 3)]
```
> `data = [frame_id, x0,y0,r0, x1,y1,r1, ...]`。第一个是采样帧号，其后每 3 个一组 `(x,y,r)`。

**发动作** — 发布 `/drive_action`（`std_msgs/Float32MultiArray`）`data=[action_id, magnitude, duration_s]`，完成回报在 `/drive_result`（`std_msgs/String` JSON）。

### 动作 id（标准麦轮逆解，车端查表 × 幅值）

| id | 动作 | id | 动作 |
|---|---|---|---|
| 0 | STOP | 6 | 右后 ↘ |
| 1 | 前进 ↑ | 7 | 右移 → |
| 2 | 左前 ↖ | 8 | 右前 ↗ |
| 3 | 左移 ← | 9 | 原地左转 ↺ |
| 4 | 左后 ↙ | 10 | 原地右转 ↻ |
| 5 | 后退 ↓ | | |

`magnitude` 是轮速幅值（PWM 档，~<30 死区、50+ 稳转）；`duration_s` 是这一步持续时间（开环定时脉冲，非闭环到点）。

### 更高层：A→B 绕障 Policy 框架（写自己的模型看这里）

上面是"取圆 / 发一步"的裸接口。要做一个**完整的 A→B 绕障模型**（自带闭环 runner、可视化、录制、安全、GUI 下拉切换），用 **`carpolicy` + `mpc/`** 框架：实现一个 `carpolicy.Policy`（`plan(obs)→Action`）、在 `mpc_baseline/registry.py` 注册一行，就自动出现在 GUI 里可选。**3 步示例 + 内置两个 baseline（离散跳格 / 连续速度）见 [`mpc/README.md`](mpc/README.md)。**

---

## GUI（`gui/car_console.py`）

`bash gui/run_gui.sh` 打开（用当前 $DISPLAY）。三区：**win0** 障碍俯视图（前=上、车在中心、红障碍圆 + 绿目标 B + 黄 MPC 预测轨迹 + 断线变橙）｜ **win1** 控制面板（**policy 下拉 + B 坐标 + execute steps + Execute/Stop** 跑 A→B；下方 3×3 方向盘 + `↺↻` 手动单步 + 红 E-STOP）｜ **win2** 日志（GET/SEND/ERR 三色过滤，全量 dump 到 `gui/car_console.log`）。**每次 Execute 跑完**自动存 `output/<日期_时间>/`：`observation.mp4`（3Hz 观测视频）+ `trajectory.png` + `run.json`。全部 ROS I/O 走 `carclient`。

---

## 小车话题一览

| 话题 | 类型 | 说明 |
|---|---|---|
| `/obstacles` | `std_msgs/Float32MultiArray` | **碰撞球**：`[frame_id, x,y,r, ...]`，米，base 系。远程主要读这个 |
| `/obstacle_points` | `std_msgs/Float32MultiArray` | **这帧圆的来源点**：`[frame_id, x,y, x,y, ...]`，同 base 系、**同 frame_id**。可视化用它才能点云与圆同源（直接订 `/scan` 不同步，见下）。有订阅者才发 |
| `/drive_action` | `std_msgs/Float32MultiArray` | **发离散动作**：`[action_id, magnitude, duration_s]` |
| `/drive_result` | `std_msgs/String` | 每步完成回报（JSON）|
| `/battery_v` | `std_msgs/Float32` | MCU 电压（链路健康信号；`/battery` 跨版本不兼容故转发）|
| `/wheel_cmd` | `std_msgs/Float32MultiArray` | 单轮直控 `[FL,RL,FR,RR]`（-100~100，正=正转）。调试/底层用 |
| `/obstacles_viz` | `visualization_msgs/MarkerArray` | 碰撞球的 rviz 可视化（Fixed Frame=base_footprint）|
| `/scan` `/odom` `/imu/data_raw` `/battery` `/tf` | — | 雷达 / 里程计 / IMU / 电量 / 变换 |

---

## 车端服务与开机自启

- **`car-ros.service`（systemd，已 enable）是唯一自启项**，跑 `roslaunch car_base viz.launch`，包含全部必需节点：`rplidar`（/scan）、`car_base`（/cmd_vel、/wheel_cmd、/odom）、`base_to_laser`（tf）、**`obstacle_circles`**（/obstacles，3Hz）、**`drive_action`**（/drive_action）。重启小车即全部就绪。
- 手动开停：`sh ~/ros.sh --enable | --disable | --restart | --status`（或 `sudo systemctl ... car-ros`）。
- **老的 TCP 服务已下线**：`perception_server.py`(:9871 get_info/execute) 本就不自启；`scan_bridge`(:9870) 已从 `viz.launch` 移除；`scan_api.py`(:8080) 未用。客户端纯走 ROS。
- **`rplidar-watchdog.service`（自启，独立于 car-ros）**：监测 `/scan`，丢失 >12s 自动恢复雷达——先 `pkill -x rplidarNode`（`viz.launch` 里 rplidar 设了 `respawn="true"`，roslaunch 立即拉回，不打断 master/驱动/GUI），不行再退到重启 car-ros。**绝不碰 USB unbind**（车 WiFi 也是 USB，unbind 会掉网）。
- **Nano 无桌面（headless）**：`get-default` = `multi-user.target`，gdm 禁用、packagekit 屏蔽（省 ~300MB + CPU/GPU）。要桌面：`sudo systemctl set-default graphical.target && sudo systemctl enable --now gdm`。⚠️ `coord_nav` 连接已改**系统级**（原来是 `user:jetson`，headless 无登录时不会自动连 → 会丢车），别改回用户级。

### 安全 harness（防电机失控）

MCU 固件对 `set_motor` **无超时**，串口不稳时"停止"命令可能丢失 → 车锁存前进**跑飞**，ROS 层停不住，只有 `estop.sh`（新串口直灌 0）能停。所以：
- 车端电压转发为 `/battery_v`（`std_msgs/Float32`，因为 `sensor_msgs/BatteryState` 跨 Melodic/Noetic md5 不兼容，客户端订不到 `/battery`）。
- `CarClient.link_ok()` / GUI 顶部横幅显示 MCU 链路健康；**断链自动禁用方向键**；**空格键 = E-STOP**。
- 保守默认：幅值 40、时长 0.5s；车端 `drive_action` 硬上限 `max_duration_s=3s`。

### 障碍感知节点（`obstacle_perception/`，C++）

`/scan` → DBSCAN 聚类 → 外接圆 → 去冗余 → `/obstacles`。**~3.4ms/帧**（旧 Python 版 ~80ms，快 ~20×），天花板 = 雷达 ~7.7Hz，默认发 **3Hz**（`viz.launch` 里 `obstacle_circles` 的 `rate_hz`；`0`=跑满）。**改完一键同步到小车**：`bash obstacle_perception/deploy_to_car.sh`（上传→车上编译→重启→验证；见 `obstacle_perception/README.md`）。`lidar_yaw/x/y` 由 launch 传入，自动与 `base_footprint→laser` 静态变换一致。

---

## 常见问题排查

| 症状 | 解决 |
|---|---|
| `rostopic list` / 连接卡住 | 小车没开机 / 没连 `coord_nav` / roscore 没起。先 `ping 10.42.0.187` |
| 能 list 但收不到数据 | 查 `/etc/hosts` 有 `10.42.0.187 jetson-desktop`；当前终端 `roscar` 过 |
| 找不到 `rostopic` / `carclient` | 忘了 `roscar`（没激活 conda `ros1`）|
| `ImportError: cannot import name 'Policy' from 'carpolicy' (unknown location)` | **在仓库根目录**用 `python -c` / `python -m` / 交互式 REPL 导包了。根目录下的 `carclient/`、`carpolicy/` 目录会把已装的同名包遮蔽成空 namespace package（`__file__` 为 `None`）。换个目录（`cd /tmp`）或直接把脚本当文件跑（`python mpc/scripts/run_sim.py`，此时仓库根不在 `sys.path`，不受影响）|
| `ModuleNotFoundError: No module named 'mpc_baseline'` | `mpc` 没装上——多半是踩了上面那个 `pip install -e` 语法坑。重跑首次配置第 3 步 |
| `/scan` 空、`/obstacles` 空 | rplidar 数据链路 wedge（电机还转、CP2102 卡）。`rplidar-watchdog` 会自动恢复；顽固时 `sudo systemctl restart car-ros` 等 ~15-20s，或 `sudo reboot`。**别用 USB unbind**（会掉网）|
| 客户端读不到 `/battery`/某车端话题 | 跨 Melodic/Noetic msg md5 不兼容（如 BatteryState）。车端改用 std_msgs 类型转发（如 `/battery_v`）|
| 热点不见了 | `sudo nmcli connection up jetson-ap` |

---

## 相关文件

| 路径 | 作用 |
|---|---|
| `car_env.sh` | ROS 环境 + 网络变量（被 `roscar` 引用）|
| `carclient/` | **Python API 包**（`pip install -e`）：`CarClient` / `Action` |
| `carpolicy/` | **通用 Policy 接口包**（`pip install -e`）：`Policy` / `Observation` / `Action` |
| `mpc/` | **A→B 绕障 policy 框架 + 两个 MPC baseline**（`pip install -e`，包名 `mpc_baseline`）。见 [`mpc/README.md`](mpc/README.md) |
| `gui/car_console.py` `gui/run_gui.sh` | PySide6 上位机 |
| `obstacle_perception/` | C++ 障碍感知 ROS 包（部署到 Nano `~/catkin_ws/src/`）|
| `car_ros/car_base_node.py` `car_ros/drive_action_node.py` `car_ros/viz.launch` | 车端在用的源码副本（部署到 `car_base/`）|
| `smoke/` | 连通性/单轮测试（`smoke_test.sh`、`wheel_test.*`、`wheel_diag.py`）+ 实车实验 `policy_run.py`、`calib_gyro.py` |
| `output/` | GUI 每次 Execute 的录制（gitignored）|

> `rplidar_watchdog.py` **只在车上**（`car_base/scripts/` + `rplidar-watchdog.service`），源码未纳入本仓库。

## 基础设施备忘（非日常）

- **热点**：NM 连接 `jetson-ap`，接口 TP-Link `wlxb4b02447290a`（RTL8811AU，DKMS 模块 `8821au`）。开：`sudo nmcli connection up jetson-ap`（**不开机自启**）。内核升级后网卡消失：`sudo dkms install rtl8821au/5.12.5.2 -k $(uname -r) && sudo modprobe 8821au`。
- **小车固定 IP**：`/etc/NetworkManager/dnsmasq-shared.d/car-static.conf` 按 MAC `64:79:f0:79:24:30` 绑定。workstation 自己用内置网卡 `wlp15s0` 上网；此热点**只做局域网**，不给小车外网。
- **小车侧改动**（均留 `*.bak.<时间戳>` 备份）：`car_base/scripts/car_base_node.py`（`/wheel_cmd`，后轮 M2/M4 接反 `rl→M4,rr→M2` + 极性取负）；`car_base/scripts/drive_action_node.py`（离散动作 + `/battery_v` 转发）；`car_base/scripts/rplidar_watchdog.py` + `/etc/systemd/system/rplidar-watchdog.service`（自启看门狗）；`car_base/launch/viz.launch`（obstacle_circles + drive_action，rplidar `respawn=true`，去 scan_bridge）；`obstacle_perception/`（C++ 包）。
- **headless 相关改动**：`set-default multi-user.target`、`gdm` disabled、`packagekit` masked、`coord_nav` 连接改系统级（`connection.permissions ""`）。别把 coord_nav 改回 `user:jetson`，否则 headless 重启丢车。
- **⚠️ 禁忌**：不要在车上 USB unbind/rebind（WiFi 是 USB，会掉网）。跨版本话题优先用 std_msgs 类型（BatteryState 等 md5 不兼容）。
