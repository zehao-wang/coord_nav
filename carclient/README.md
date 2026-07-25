# carclient — 和小车通信的 Python API

ROS 传输。用 `roscar` 把 `ROS_MASTER_URI` 指到车上,然后:

```python
from carclient import CarClient, Action

car = CarClient()
car.wait_obstacles(timeout=5)

while not car.is_shutdown():
    obs = car.obstacles()
    if obs and obs.age < 1.0:
        car.drive(my_model(obs.circles))   # 0..10
    car.sleep(1/3.0)
```

## 读感知

| 方法 | 返回 | 说明 |
|---|---|---|
| `obstacles()` | `Obstacles(frame_id, circles, age)` | 最新碰撞圆。`circles` 每个 `(x,y,r)` 米、base 系(x 前 y 左)。非阻塞、线程安全缓存 |
| **`observation()`** | `Frame(frame_id, circles, points, age)` | **一个采样**:圆和点保证同一个 `frame_id`(车上同一次处理、同一帧 scan)。**画图用这个**。`points=None` 表示这帧还没配上点 —— 不会拿别帧的点顶替 |
| **`wait_frame(after, timeout)`** | `Frame` 或 `None` | **阻塞到新的一帧**。这是整个栈的 **tick 边界**:一帧观测 = 一个 tick,不重复、不跳帧。按"id 不同"而非"id 更大"匹配,所以车端计数器重启不会把它挂死 |
| `obstacle_points(frame_id)` | `ObsPoints(frame_id, pts, age)` | 指定帧的来源点 |
| `scan_points()` | `ScanPoints(pts, age)` | 原始 `/scan`,**和圆不同步**。需要 `CarClient(subscribe_scan=True)`,默认关(实测 43 KB/s,而这条链路的延迟曾是闭环失稳的根因)。画图请用 `observation()` |
| `pose()` | `Pose(x, y, yaw, age)` | `/odom`,xy 来自编码器、yaw 来自 IMU 陀螺 |

## 发动作

| 方法 | 说明 |
|---|---|
| `drive(action, magnitude=None, duration=None)` | 一个离散动作。省略则用默认(40 / 0.5s) |
| `drive_wheels(fl, rl, fr, rr, duration)` | 速度脉冲,车端本地 20Hz 保活 + 到时刹车 |
| `stop()` / `estop()` | 软停 / 硬急停(SSH 跑 estop.sh) |
| `on_result(cb)` / `last_result()` | 每步完成回报 `{"action","reason","took_ms",...}` |
| `link_ok()` / `connected()` | MCU 链路健康 / 数据在不在 |

> `on_result` 是**单槽**回调,GUI 已经占用。别的地方要结果请**轮询** `last_result()`。

### 动作 id(标准麦轮逆解,车端查表 × 幅值)

| id | 动作 | id | 动作 |
|---|---|---|---|
| 0 | STOP | 6 | 右后 ↘ |
| 1 | 前进 ↑ | 7 | 右移 → |
| 2 | 左前 ↖ | 8 | 右前 ↗ |
| 3 | 左移 ← | 9 | 原地左转 ↺ |
| 4 | 左后 ↙ | 10 | 原地右转 ↻ |
| 5 | 后退 ↓ | | |

`magnitude` 是轮速幅值(PWM 档);`duration_s` 是这一步持续时间(**开环定时脉冲,不是闭环到点**)。

> **实测**:电机静摩擦阈值 **~14 PWM**,之上线性 `轮速 = (PWM−14)/72.1` m/s。
> 所以 **mag 20 只比阈值高 6,车几乎不动**;mag 40 ≈ 0.36 m/s。见 [`calibration/`](../calibration/README.md)。

## 直接用 ROS 话题(等价)

```python
m = rospy.wait_for_message("/obstacles", Float32MultiArray)
frame_id = int(m.data[0])
circles  = [(m.data[i], m.data[i+1], m.data[i+2]) for i in range(1, len(m.data)-2, 3)]
```
发动作:`/drive_action` `data=[action_id, magnitude, duration_s]`,回报在 `/drive_result`(JSON)。

### 话题一览

| 话题 | 类型 | 说明 |
|---|---|---|
| `/obstacles` | `Float32MultiArray` | **碰撞球** `[frame_id, x,y,r, ...]`,米,base 系 |
| `/obstacle_points` | `Float32MultiArray` | **这帧圆的来源点** `[frame_id, x,y, ...]`,**同 frame_id**。是聚类的**输入**(含 DBSCAN 噪声点,圆另经 EMA),保证同一采样而非逐点对应。有订阅者才发 |
| `/drive_action` | `Float32MultiArray` | 离散动作 `[id, magnitude, duration_s]` |
| `/drive_wheels` | `Float32MultiArray` | 速度脉冲 `[FL,RL,FR,RR,duration]`,车端保活 |
| `/drive_result` | `String` | 每步完成回报(JSON) |
| `/battery_v` | `Float32` | MCU 电压(链路健康信号;`/battery` 跨版本不兼容故转发) |
| `/wheel_cmd` | `Float32MultiArray` | 单轮直控 `[FL,RL,FR,RR]`(−100~100)。**绕过全部安全 harness**,仅调试用 |
| `/obstacles_viz` | `MarkerArray` | rviz 可视化(Fixed Frame=base_footprint) |
| `/scan` `/odom` `/imu/data_raw` `/tf` | — | 雷达 / 里程计 / IMU / 变换 |

## ⚠️ 包遮蔽

**不要在仓库根目录**用 `python -c` / `python -m` / REPL 导包 —— 根目录下的 `carclient/`、`carpolicy/` 目录会把已装的同名包遮蔽成空 namespace package。把脚本当文件跑不受影响。详见顶层 README 的排查表。
