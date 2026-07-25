# obstacle_perception — 障碍感知（C++，跑在小车上）

`/scan` → DBSCAN 聚类 → 外接圆 → 去冗余 → **时域滤波** → **`/obstacles`**（`[frame_id, x,y,r, ...]`，米，base 系，默认 3Hz）。~3.4ms/帧。是所有 policy 的 observation 来源。

同一次 `process()` 还发 **`/obstacle_points`**（`[frame_id, x,y, x,y, ...]`，同 base 系）——**这帧圆是从这批点算出来的**，带**同一个 frame_id**。可视化端据此把点云和圆当成同一个采样来画。
注意这是聚类的**输入**：里面也包含 DBSCAN 判为噪声（不属于任何圆）的点（实测几个百分点），而且圆还额外过了 `temporalFilter` 的跨帧 EMA。保证的是**同一个采样**，不是逐点对应。
直接订 `/scan` 做不到同步：节点是按 `rate_hz` 定时器取"最新一帧 scan"，3Hz 对雷达 ~7.7Hz 意味着**约 60% 的 scan 没参与生成圆**，客户端手上最新的 `/scan` 通常不是圆的来源（实测偏差 −0.04 ~ +0.22 s）。
只在有订阅者时才发（实测 646 点/帧 ≈ 5.0 KB/帧 → 3Hz 下 15.1 KB/s）。参数：`publish_points`（默认 true）、`points_stride`（>1 抽稀）。

- 源码：`src/obstacle_circles_node.cpp`（改算法改这里）
- 部署在小车：`~/catkin_ws/src/obstacle_perception/`，随 `car-ros`（`viz.launch` 的 `obstacle_circles` 节点）自启。
- **时域滤波**：订阅 `/odom` 做自我运动补偿——把上一帧的圆按车的位姿增量推到当前 base 系应在的位置，再和新测量关联、对**残差**做 EMA。这样车移动/转弯时圆**不滞后**，只消掉"聚类点族每帧不同"造成的圆心/半径抖动。新障碍不延迟、消失的圆立即丢。参数 `filter_alpha`（0=关，越大越平滑，默认 0.5）、`filter_assoc`（帧间关联门限 m，默认 0.4）。
- 参数（`margin=0.02` 等）在 **`viz.launch`** 里（覆盖 C++ 默认）；改完由 `deploy_to_car.sh` 一起同步。

## 改完一键同步到小车

在**工作站**上，改完 `src/` 后跑：

```bash
bash obstacle_perception/deploy_to_car.sh
```

一条命令做完 **上传 → 车上编译 → 重启 car-ros → 验证 `/obstacles`**，小车直接用上新算法。

**脚本能捕获什么**（每种失败都非 0 退出、直接把错误打给你看）：
- **编译失败**（语法错 / 链接错 / CMake 错）→ 自动摘出并高亮编译器错误行；**不重启小车**，旧的可用版本继续跑。
- **运行时崩溃**（编过但节点跑起来挂 / 不发 `/obstacles`）→ 重启后轮询验证 `/obstacles`，超时则自动拉出 car-ros 日志里的报错（segfault / exception / assert 等）。
- **管不了的**：编过、跑起来、也在发，但**算出的圆是错的**（逻辑 bug）——这属于语义正确性，脚本无法判断。看 **GUI 俯视图** 或某次 Execute 存下的 `output/*/observation.mp4` 目视确认。
- 走 SSH（`jetson@10.42.0.187`，key `~/.ssh/id_ed25519`）；车/路径变了可用环境变量覆盖：`CAR_IP=... CAR_SSH_KEY=... bash obstacle_perception/deploy_to_car.sh`。
- 前提：小车已上电、连上 `coord_nav` 热点（`sudo nmcli connection up jetson-ap` 开热点）。

> 只想手动：`rsync` 源码到车上，再 `ssh` 里 `cd ~/catkin_ws && source /opt/ros/melodic/setup.bash && catkin_make --pkg obstacle_perception && sudo systemctl restart car-ros`。脚本就是把这串封装 + 加了可达性/编译/验证检查。
