# sentry_nav_bringup

哨兵机器人导航总入口包。汇集所有 launch 脚本、Nav2 参数、地图、PCD、RViz 配置与 Nav2 内置行为树 XML。

---

## 包功能概述

`sentry_nav_bringup` 承担以下职责：

- 提供实车 **launch 入口**
- 存放 `config/reality/` **Nav2 参数**
- 存放地图（`map/`）、先验点云（`pcd/`）、RViz 布局（`rviz/`）
- 存放 `bt_navigator` 使用的 **Nav2 内置 BT XML**（与 `sentry_behavior` 状态机决策不同）

---

## 核心设计

**自旋底盘 + 虚拟惯性系**：底盘持续自旋，`gimbal_yaw_fake` 与 `gimbal_yaw` 反向旋转，
使 Nav2 规划器始终在稳定的惯性系中工作。`fake_vel_transform` 在执行端把 Nav2 输出旋回真实系
并叠加自旋速度分量，再发往底盘。

### TF 树

```
map → odom → base_footprint → chassis → gimbal_yaw → gimbal_pitch → front_mid360
                                             ↓
                                       gimbal_yaw_fake   (Nav2 规划用虚拟系)
```

### 速度指令链路

从 `navigation_launch.py` remappings 直接确认的话题名称：

```
controller_server     →  cmd_vel_controller   (remap: cmd_vel → cmd_vel_controller)
                                ↓
velocity_smoother     订阅 cmd_vel_controller  (remap 输入)
                      发布 cmd_vel_nav2_result (remap: cmd_vel_smoothed → cmd_vel_nav2_result)
                                ↓
fake_vel_transform    订阅 cmd_vel_nav2_result (input_cmd_vel_topic)
                      发布 cmd_vel_chassis     (output_cmd_vel_topic)
                                ↓
        实车: rm_serial_driver 订阅 /cmd_vel_chassis
```

> `enable_stamped_cmd_vel: true` 在 `controller_server`、`behavior_server`、`velocity_smoother`
> 三处**必须一致**，否则类型不匹配导致链路断开。

---

## Launch 入口

### 一览表

| 脚本 | 用途 | 关键参数与默认值 |
|---|---|---|
| `rm_navigation_reality_launch.py` | **实车主入口**（不含串口驱动） | 见下表 |
| `rm_sentry_launch.py` | **实车一键**（导航 + 串口 + 录包 + 可选状态机决策） | 见下表 |
| `bringup_launch.py` | 内部聚合，被以上入口 include | `slam`, `map`, `prior_pcd_file`, `scan_context_db_file`, `use_composition`(`True`), `log_level`(`info`) |
| `slam_launch.py` | SLAM 模式（point_lio + slam_toolbox + pointcloud_to_laserscan） | `namespace`, `params_file`, `use_sim_time`, `autostart`, `use_respawn`, `log_level` |
| `localization_launch.py` | 定位模式（point_lio + map_server + small_gicp_relocalization） | `namespace`, `map`, `prior_pcd_file`, `scan_context_db_file`, `use_composition`(`False`), `container_name`(`nav2_container`) |
| `navigation_launch.py` | Nav2 lifecycle 节点群 + terrain + odom_bridge + fake_vel_transform | `namespace`, `params_file`, `use_composition`(`False`), `container_name`(`nav2_container`) |
| `robot_state_publisher_launch.py` | URDF + TF（仅在导航模块独立运行时使用） | `namespace`, `use_sim_time`, `robot_name`(`sentry_robot`) |
| `rviz_launch.py` | RViz2 可视化（退出 RViz 触发整体 Shutdown） | `namespace`, `rviz_config`(`rviz/nav2_default_view.rviz`) |

### rm_navigation_reality_launch.py 参数表

| 参数 | 默认值 | 说明 |
|---|---|---|
| `namespace` | `""` | 机器人命名空间（空串 = 无前缀） |
| `slam` | `False` | `True` = SLAM 建图；`False` = 重定位 |
| `world` | `204` | 场地名，决定 `map/reality/<world>.yaml` 与 `pcd/reality/<world>.pcd` 路径 |
| `map` | `map/reality/<world>.yaml` | 2D 地图 |
| `prior_pcd_file` | `pcd/reality/<world>.pcd` | small_gicp 先验点云 |
| `scan_context_db_file` | `pcd/reality/<world>.scdb` | Scan Context 数据库 |
| `use_sim_time` | `False` | 使用系统时钟 |
| `params_file` | `config/reality/nav2_params.yaml` | Nav2 参数文件 |
| `autostart` | `true` | 自动激活 lifecycle 节点 |
| `use_composition` | `True` | 使用 component container |
| `use_respawn` | `False` | 节点崩溃后自动重启 |
| `use_robot_state_pub` | `True` | 是否启动 robot_state_publisher（导航独立运行时需要） |
| `use_rviz` | `True` | 启动 RViz2 |
| `use_foxglove` | `False` | 启动 foxglove_bridge (port 8765) |

### rm_sentry_launch.py 参数表

| 参数 | 默认值 | 说明 |
|---|---|---|
| `namespace` | `""` | 机器人命名空间 |
| `slam` | `False` | SLAM 模式开关 |
| `world` | `204` | 场地名 |
| `use_rviz` | `True` | 启动 RViz2 |
| `use_foxglove` | `False` | 启动 foxglove_bridge |
| `enable_recorder` | `True` | 启动 sentry_match_recorder（延迟 5s，game_progress=4 时自动录包） |
| `enable_behavior` | `False` | 启动 sentry_behavior 战术决策（延迟 8s） |
| `strategy` | `rmuc_defend` | 传给 sentry_behavior_node 的状态机策略名（rmuc_defend / a / b） |

---

## 启动方式

### 实车建图

```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py \
  slam:=True use_robot_state_pub:=True
```

### 实车导航（已有地图）

```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py \
  world:=<场地名> slam:=False use_robot_state_pub:=True
```

### 实车一键（导航 + 串口）

```bash
ros2 launch sentry_nav_bringup rm_sentry_launch.py

# 指定场地 + 开启状态机决策：
ros2 launch sentry_nav_bringup rm_sentry_launch.py \
  world:=rmul_2026 enable_behavior:=True strategy:=rmuc_defend
```

---

## 配置目录结构

```
config/
└── reality/
    ├── nav2_params.yaml           实车 Nav2 参数 (use_sim_time=false, controller_frequency=30Hz)
    └── mid360_user_config.json    Livox Mid360 驱动网络配置

map/                               2D 地图（用户建图后手动存放）
│   # 建议命名：map/reality/<world>.yaml

pcd/                               先验点云（用户建图后手动存放）
│   # 建议命名：pcd/reality/<world>.pcd
│   #           pcd/reality/<world>.scdb

rviz/
└── nav2_default_view.rviz         Nav2 调试可视化布局

behavior_trees/                    Nav2 bt_navigator 使用的内置 BT（非战术决策树）
├── navigate_to_pose_w_replanning_and_recovery.xml
└── navigate_through_poses_w_replanning_and_recovery.xml
```

> 战术决策树（`RMUC.xml` 等）在 `sentry_behavior/behavior_trees/`，与此目录无关。

---

## 节点与话题

### 节点依赖关系（bringup_launch → 子 launch）

```
bringup_launch.py
  ├── slam=True  → slam_launch.py
  │     ├── point_lio                    (建图里程计，prior_pcd.enable=False)
  │     ├── slam_toolbox                 (sync_slam_toolbox_node，scan_topic=obstacle_scan)
  │     ├── pointcloud_to_laserscan      (in: terrain_map_ext → out: obstacle_scan)
  │     ├── map_saver_server
  │     └── static_transform_publisher   (map → odom，固定零变换)
  │
  ├── slam=False → localization_launch.py
  │     ├── point_lio                    (定位里程计)
  │     ├── map_server                   (加载 2D 地图)
  │     └── small_gicp_relocalization    (发布 map→odom TF)
  │
  └── 始终 → navigation_launch.py
        ├── terrain_analysis
        ├── terrain_analysis_ext
        ├── odom_bridge             (发布 odom→base_footprint TF + odometry 话题)
        ├── fake_vel_transform      (速度坐标旋转 + 自旋叠加)
        ├── controller_server       → cmd_vel_controller
        ├── velocity_smoother       订阅 cmd_vel_controller → 发布 cmd_vel_nav2_result
        ├── planner_server
        ├── behavior_server
        ├── bt_navigator
        ├── waypoint_follower
        └── lifecycle_manager_navigation
```

### 关键话题

| 话题 | 类型 | 方向 | 节点 |
|---|---|---|---|
| `cmd_vel_controller` | `TwistStamped` | 发布 | `controller_server`（remap from `cmd_vel`） |
| `cmd_vel_nav2_result` | `TwistStamped` | 发布 | `velocity_smoother`（remap from `cmd_vel_smoothed`） |
| `cmd_vel_chassis` | `Twist` | 发布 | `fake_vel_transform` |
| `cmd_spin` | `std_msgs/Float32` | 订阅 | `fake_vel_transform`（动态设置自旋速度） |
| `odometry` | `nav_msgs/Odometry` | 发布 | `odom_bridge` |
| `aft_mapped_to_init` | `nav_msgs/Odometry` | 订阅 | `odom_bridge`（来自 point_lio） |
| `cloud_registered` | `sensor_msgs/PointCloud2` | 订阅 | `odom_bridge`（已配准点云） |
| `terrain_map` | `sensor_msgs/PointCloud2` | 发布 | `terrain_analysis` → local_costmap 观测源 |
| `terrain_map_ext` | `sensor_msgs/PointCloud2` | 发布 | `terrain_analysis_ext` → global_costmap + pointcloud_to_laserscan |
| `obstacle_scan` | `sensor_msgs/LaserScan` | 发布 | `pointcloud_to_laserscan`（供 slam_toolbox） |
| `/initialpose` | `PoseWithCovarianceStamped` | 订阅 | `small_gicp_relocalization`（RViz 重定位输入） |
| `/navigate_to_pose` | Action | server | `bt_navigator` |

> 带命名空间前缀的话题由 `bringup_launch.py` 中的 `PushRosNamespace` 自动添加；实车默认无前缀。

---

## 关键约束

- `controller_frequency` **必须等于** `smoothing_frequency`，两者都不能超过 CPU 实际吞吐上限
- `enable_stamped_cmd_vel: true` 在 `controller_server`、`behavior_server`、`velocity_smoother` 三处必须一致
- `enable_periodic_relocalization: true` 必须开启，否则 small_gicp 不持续周期纠偏
- PCD 先验地图与 2D 地图**必须在同一坐标系、同一起点**建图，不可混用不同 session 的产物
- 实车入口（含 `rm_sentry_launch.py`）`world` 默认 `204`，需根据实际地图文件名覆盖
- `small_gicp_relocalization.init_pose` 必须对应启动底盘在地图中的位姿。公共参数默认零位姿；`localization_params_file:=auto` 按地图 YAML 文件名加载 `config/reality/localization/<地图名>.yaml`，0927 也使用建图起点零初值，不再把某次运行中估计的朝向固化为启动配置。显式传入其他 YAML 可以覆盖；`localization_params_file:=none` 关闭地图专属覆盖。
- 起点位置改变时，用 RViz **2D Pose Estimate** 或定位 profile 指定初值。`initial_search_all_yaws: true` 在初值位置 1 m 范围内搜索全部朝向，要求至少 95% 三维重合、截断距离均方根 <7.5 cm，并拒绝相隔 >20° 或 >40 cm 且分数相近的竞争解。它解决启动朝向超出局部搜索范围的问题，不保证未知位置或对称场地中的全局定位；关闭该选项恢复窄朝向先验。
- 重定位同时检查实际三维点到先验地图的距离：至少 80% 的采样墙面点落在 15 cm 内才提交结果。优化器收敛或低分数本身不再代表对齐。`/localization_valid` 在初值未验证或连续 3 秒没有新的有效匹配时为 false；该状态本身不等同于底盘制动。
- 点云累积使用最近 15 帧的滑动窗口，定位周期为 1 秒，避免拿几秒前的观测修正当前 TF。
- 冷启动和周期定位在 GICP 粗配准后，用保留高度的三维最近邻距离精修 x/y/yaw，并比较从初值直接精修的结果。这样减少 XY 体素划分随里程计坐标朝向变化引入的配准偏差；仍需通过纠偏幅度和三维重叠率检查。
- SLAM 模式的二维地图固定在 odom 下；Point-LIO 的周期 PCD 和退出保存 PCD 均转换到 odom。没有收到坐标变换时，不输出可供导航使用的原始坐标 PCD；退出时只保存明确命名的 `scans_unregistered_lidar_odom.pcd` 供恢复，不能直接与二维地图配对。

详细调优推导见 [`src/docs/TUNING_GUIDE.md`](../docs/TUNING_GUIDE.md)。

---

## 点云对齐回归

先 `source install/setup.bash`。离线测试必须使用隔离的 ROS domain，避免把测试位姿发给实车：

```bash
ROS_DOMAIN_ID=174 python3 tests/pointcloud_alignment_regression.py
ROS_DOMAIN_ID=175 python3 tests/map_profile_regression.py
ROS_DOMAIN_ID=176 python3 tests/odom_geometry_regression.py
ROS_DOMAIN_ID=173 python3 tests/localization_regression.py
ROS_DOMAIN_ID=173 python3 tests/odom_scan_timestamp_regression.py
ROS_DOMAIN_ID=173 python3 tests/navigation_visibility_regression.py
```

`pointcloud_alignment_regression.py --real-dir <采样目录>` 还可回放真实点云，需要 `prior.npy`、`before/registered_scan.npy` 与 `reference_pose.json`。可用 `tests/capture_alignment_fixture.py --pcd <先验PCD> --out <采样目录>` 采集；存入工作区日志目录以免重启后丢失。合成场景检查已知位姿误差、多个地图和朝向、噪声/缺失点、错误地图/朝向/坐标系、断流恢复、手动初值和观测更新时效。真实回放不是实车移动测试。加 `--heading-search --test-zero-seed` 可测试已知起点的全朝向搜索、对称歧义拒绝和真实扫描的零初值启动。

静止实机重启验收脚本会实际停止指定 launch，然后重复启动导航（决策关闭，不发送目标）。使用前保持机器人静止，确认指定 PID 正是要停止的导航 launch：

```bash
python3 tests/live_alignment_restart.py --world 0927 --stop-launch-pid <PID> \
  --restarts 5 --duration 45 --output-dir logs/live_alignment_restart
```

每轮至少 10 个有效采样窗口，首次定位后不得再次出现无效状态；每个窗口的三维墙面点到先验 PCD 的距离中位数 <7 cm、90 分位 <20 cm、15 cm 内占比 >80% 才通过。失败写入 `results.json`，超标窗口保存为 `run*_bad_*.npy`，并记录初始串口关节角、关节发布值与 odom 变换，不会丢弃坏窗口。该距离是与地图的一致性，不能替代外部测量的绝对定位精度；静止测试也不覆盖行驶、自旋、上坡或真实异地建图。

---

## 相关文档

- [系统架构详解](../docs/ARCHITECTURE.md)
- [快速部署](../docs/QUICKSTART.md)
- [运行模式说明](../docs/RUNNING_MODES.md)
- [参数调优](../docs/TUNING_GUIDE.md)
- [远程调试（Foxglove）](../docs/REMOTE_DEBUG.md)
- [sentry_behavior 说明](../sentry_behavior/README.md)
