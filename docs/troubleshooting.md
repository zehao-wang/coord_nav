# 排查

## 连不上 / 收不到数据

| 症状 | 解决 |
|---|---|
| `rostopic list` / 连接卡住 | 小车没开机 / 没连 `coord_nav` / roscore 没起。先 `ping 10.42.0.187` |
| 能 list 但收不到数据 | 查 `/etc/hosts` 有 `10.42.0.187 jetson-desktop`;当前终端 `roscar` 过 |
| 找不到 `rostopic` / `carclient` | 忘了 `roscar`(没激活 conda `ros1`) |
| 热点不见了 | `sudo nmcli connection up jetson-ap` |
| **延迟很大但话题都在** | WiFi 省电模式。用 `ping` 量(`rostopic list` 只碰 master,看起来一切正常);`iwconfig apwifi0 \| grep 'Power Man'`。这曾是闭环失稳的根因,见 [`car-setup.md`](car-setup.md) |
| `rostopic hz`/`echo` 挂住不理 `timeout` | 已知。用有界的 python `wait_for_message` 代替 |

## 装包 / 导包

| 症状 | 解决 |
|---|---|
| `ImportError: cannot import name 'Policy' from 'carpolicy' (unknown location)` | **在仓库根目录**用 `python -c` / `python -m` / REPL 导包了。根目录下的 `carclient/`、`carpolicy/` 目录会把已装的同名包遮蔽成空 namespace package(`__file__` 为 `None`)。换个目录(`cd /tmp`),或把脚本当文件跑(`python mpc/scripts/run_sim.py`,此时仓库根不在 `sys.path`) |
| `ModuleNotFoundError: No module named 'mpc_baseline'` | `mpc` 没装上。**`-e` 每个路径前都要写一次**:`pip install -e carclient -e carpolicy -e mpc`。写成 `pip install -e carclient carpolicy mpc` 只会装第一个,后两个被当成 PyPI 包名 |

## 传感器 / 车

| 症状 | 解决 |
|---|---|
| `/scan` 空、`/obstacles` 空 | rplidar 数据链路 wedge(电机还转、CP2102 卡)。`rplidar-watchdog` 会自动恢复;顽固时 `sudo systemctl restart car-ros` 等 ~15-20s,或 `sudo reboot`。**别用 USB unbind**(会掉网) |
| 客户端读不到 `/battery` 或某车端话题 | 跨 Melodic/Noetic msg md5 不兼容(如 `BatteryState`)。车端改用 std_msgs 类型转发(如 `/battery_v`) |
| 重启 car-ros 后客户端收不到数据了 | 正常:roscore 一起重启,订阅注册失效,客户端必须**重连**。GUI 会自行重启 |
| 发了指令但车不动 | 幅值低于摩擦阈值(~14 PWM)。**mag 20 几乎不动**,实用值 40。见 [`calibration/`](../calibration/README.md) |
| `MCU link not healthy -- refusing to drive` | `/battery_v` 还没到(客户端刚建、等一两秒),或电压真的低。`link_ok()` 阈值 6.0 V |
| `MCU sensors are not live -- refusing to drive` | **MCU 串口读通道卡死**:写还通、读冻结。所有 MCU 话题定在同一个值,陀螺卡住 → odom yaw 静止时也会以 ~60°/s 爬升,而 `link_ok()` 照样返回 True(它只看电压是否合理,一个冻住的 10.00 V 也合理)。**轮子仍会响应指令**,车会在原地转。**断电重启**,重启 car-ros 不一定能清掉。自查:`CarClient.sensors_live()`(启动时)/ `imu_frozen()`(runner 每 tick 自动查,冻结即急停,tick log 记 `WEDGE`)|

## 控制表现不对

先看 **tick log**(`output/<启动时间>/tick_*.log`)的 `FLAGS` 列 —— 健康的 tick 是 `.`:

| flag | 含义 |
|---|---|
| `WPIN` | 偏航指令钉在上限 = 转不过来。多半是模型/实车失配,重新标定 |
| `AWAY` | 离目标越来越远 |
| `ZERO` | 命令全停但没到终点 = 车僵住了 |
| `SKIP` | 丢了观测帧 |
| `OVERRUN` | tick 内的工作超出了周期 |
| `NOPTS` | `/obstacle_points` 没和这帧配上 |
| `STALE` / `NOFRAME` | 数据过期 / 没等到新帧,车已停 |

`real/cmd` 列是实测/指令比,持续偏离 1.00 就该重新标定,见 [`calibration/`](../calibration/README.md)。

## 其它

- `pkill -f <x>` 会匹配到运行 pkill 的那个 shell(命令行里含 `<x>`)。用 `pkill -x <procname>` 或按 pid 杀。
- 启动 GUI 前**不要** `pkill -f car_console.py` —— 会自杀。
