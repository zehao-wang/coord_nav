# 车端服务、安全 harness、基础设施备忘

非日常操作。日常使用看顶层 [`README.md`](../README.md)。

## 车端服务与开机自启

- **`car-ros.service`(systemd,已 enable)是唯一自启项**,跑 `roslaunch car_base viz.launch`,包含全部必需节点:`rplidar`(/scan)、`car_base`(/cmd_vel、/wheel_cmd、/odom)、`base_to_laser`(tf)、**`obstacle_circles`**(/obstacles,3Hz)、**`drive_action`**(/drive_action)。重启小车即全部就绪。
- 手动开停:`sh ~/ros.sh --enable | --disable | --restart | --status`(或 `sudo systemctl ... car-ros`)。
- **老的 TCP 服务已下线**:`perception_server.py`(:9871) 本就不自启;`scan_bridge`(:9870) 已从 `viz.launch` 移除;`scan_api.py`(:8080) 未用。客户端纯走 ROS。
- **`rplidar-watchdog.service`(自启,独立于 car-ros)**:监测 `/scan`,丢失 >12s 自动恢复雷达 —— 先 `pkill -x rplidarNode`(`viz.launch` 里 rplidar 设了 `respawn="true"`,roslaunch 立即拉回,不打断 master/驱动/GUI),不行再退到重启 car-ros。**绝不碰 USB unbind**(车 WiFi 也是 USB,unbind 会掉网)。
- **Nano 无桌面(headless)**:`get-default` = `multi-user.target`,gdm 禁用、packagekit 屏蔽(省 ~300MB + CPU/GPU)。要桌面:`sudo systemctl set-default graphical.target && sudo systemctl enable --now gdm`。

> ⚠️ `coord_nav` 连接已改**系统级**(原来是 `user:jetson`,headless 无登录时不会自动连 → 会丢车),别改回用户级。

> ⚠️ **重启 car-ros 会连 roscore 一起重启**,已有客户端的订阅注册随之失效,必须重连(GUI 会自行重启)。

## 安全 harness(防电机失控)

**MCU 固件对 `set_motor` 无超时**,串口不稳时"停止"命令可能丢失 → 车锁存前进**跑飞**,ROS 层停不住,只有 `estop.sh`(新串口直灌 0)能停。所以:

- 车端电压转发为 `/battery_v`(`std_msgs/Float32`,因为 `sensor_msgs/BatteryState` 跨 Melodic/Noetic md5 不兼容,客户端订不到 `/battery`)。
- `CarClient.link_ok()` / GUI 顶部横幅显示 MCU 链路健康;**断链自动禁用方向键**;**空格键 = E-STOP**。
- 车端 `drive_action` 硬上限 `max_duration_s=3s`,到时**持续刹车** `stop_hold_s=1.5s`。
- 新指令会**顶掉**正在跑的那条(`superseded`)并立即生效,同时取消刹车 —— 背靠背的指令之间不会插入刹车,这是连续运动的基础。
- `PolicyRunner` 不确认 MCU 链路健康就拒绝启动(这样断链急停才是**已武装**的);任何异常 / Ctrl-C / 断链都硬急停。

## 障碍感知节点(`obstacle_perception/`,C++)

`/scan` → DBSCAN 聚类 → 外接圆 → 去冗余 → 时域滤波 → `/obstacles`。**~3.4ms/帧**(旧 Python 版 ~80ms),天花板 = 雷达 ~7.7Hz,默认发 **3Hz**(`viz.launch` 里 `obstacle_circles` 的 `rate_hz`;`0`=跑满)。

**改完一键同步到小车**:`bash obstacle_perception/deploy_to_car.sh`(上传→车上编译→重启→验证)。详见 [`obstacle_perception/README.md`](../obstacle_perception/README.md)。

`lidar_yaw/x/y` 由 launch 传入,自动与 `base_footprint→laser` 静态变换一致。

## 基础设施备忘

- **热点**:NM 连接 `jetson-ap`,接口 TP-Link `wlxb4b02447290a`(RTL8811AU,DKMS 模块 `8821au`)。开:`sudo nmcli connection up jetson-ap`(**不开机自启**)。内核升级后网卡消失:`sudo dkms install rtl8821au/5.12.5.2 -k $(uname -r) && sudo modprobe 8821au`。
- **小车固定 IP**:`/etc/NetworkManager/dnsmasq-shared.d/car-static.conf` 按 MAC `64:79:f0:79:24:30` 绑定。workstation 自己用内置网卡 `wlp15s0` 上网;此热点**只做局域网**,不给小车外网。
- **WiFi 省电必须关**:开着会让链路延迟从 ~3ms 飙到 **3+ 秒**,曾经是闭环失稳(车螺旋/撞障碍)的**根因**。已做成车的系统默认。诊断用 `ping`(不是 `rostopic list`,后者只碰 master,看起来一切正常)。
- **小车侧改动**(均留 `*.bak.<时间戳>` 备份):`car_base/scripts/car_base_node.py`(`/wheel_cmd`,后轮 M2/M4 接反 `rl→M4,rr→M2` + 极性取负);`car_base/scripts/drive_action_node.py`(离散动作 + `/battery_v` 转发);`car_base/scripts/rplidar_watchdog.py` + 对应 service;`car_base/launch/viz.launch`;`obstacle_perception/`(C++ 包)。
- **headless 相关改动**:`set-default multi-user.target`、`gdm` disabled、`packagekit` masked、`coord_nav` 连接改系统级(`connection.permissions ""`)。
- **⚠️ 禁忌**:不要在车上 USB unbind/rebind(WiFi 是 USB,会掉网)。跨版本话题优先用 std_msgs 类型(BatteryState 等 md5 不兼容)。

> `rplidar_watchdog.py` **只在车上**(`car_base/scripts/` + `rplidar-watchdog.service`),源码未纳入本仓库。
