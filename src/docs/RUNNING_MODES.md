# 运行模式与参数详解

> 本文档详细说明所有运行模式的启动方式、完整参数列表及调优建议。
> 快速上手请参阅 [快速部署指南](QUICKSTART.md)，系统架构请参阅 [架构详解](ARCHITECTURE.md)。

---

## 目录

- [1. 运行模式总览](#1-运行模式总览)
- [2. 实车导航模式](#2-实车导航模式)
- [3. SLAM 建图模式](#3-slam-建图模式)
- [4. 状态机决策系统](#4-状态机决策系统)
- [5. 辅助工具](#5-辅助工具)
- [6. Nav2 导航参数详解](#6-nav2-导航参数详解)
- [7. 定位模块参数详解](#7-定位模块参数详解)
- [8. 地形分析参数详解](#8-地形分析参数详解)
- [9. 串口通信参数](#9-串口通信参数)
- [10. 常见调参场景](#10-常见调参场景)

---

## 1. 运行模式总览

系统支持以下运行模式，通过不同的 launch 文件和参数组合实现切换：

| 模式 | 启动文件 | 核心参数 | 用途 |
|:---|:---|:---|:---|
| 实车导航 | `rm_navigation_reality_launch.py` | `slam:=False` | 物理机器人的自主导航 |
| 实车建图 | `rm_navigation_reality_launch.py` | `slam:=True` | 物理机器人构建地图 |
| 状态机决策 | `sentry_behavior_launch.py` | `strategy` | 比赛战术逻辑（独立启动） |
| 手柄遥控 | 内嵌于导航 launch | 默认开启 | PS4 手柄控制 |

### Launch 层级关系

```
实车模式:
  rm_navigation_reality_launch.py
  ├── livox_ros_driver2 (实车激光雷达驱动)
  ├── robot_state_publisher_launch.py (可选)
  ├── bringup_launch.py
  │   ├── [slam=True]  → slam_launch.py
  │   ├── [slam=False] → localization_launch.py
  │   └── navigation_launch.py (始终启动)
  ├── joy_teleop_launch.py
  └── rviz_launch.py (可选)

状态机决策 (独立):
  sentry_behavior_launch.py
  └── sentry_behavior_node (自研分层状态机, 发布 /goal_pose)
```

### 两层决策/导航架构

系统将"决定去哪里"与"决定怎么去"分离：

```
sentry_behavior (自研分层状态机)
    │  决定 "去哪里"：按 strategy 的 guard 表选目标
    │  └─ 发布目标到 /goal_pose（去重 + 无订阅重发）
    v
Nav2 bt_navigator (Nav2 导航 BT)
    │  决定 "怎么去"
    │  ComputePathToPose → FollowPath
    v
omni_pid_pursuit_controller (局部控制)
```

- **战术决策层** (`sentry_behavior`): 自研分层状态机（无 BehaviorTree），裁判系统驱动，向 `/goal_pose` 发布目标点。
- **导航执行层** (`sentry_nav_bringup/behavior_trees/`): BTCPP_format="3"，Nav2 `bt_navigator` 标准导航行为树，处理路径规划、跟踪和恢复。

---

## 2. 实车导航模式

### 前置条件
- Livox MID360 激光雷达已连接并配置网络（默认 LiDAR IP: `192.168.1.150`，主机 IP: `192.168.1.50`）
- 串口设备已连接（默认 `/dev/ttyACM0`）
- 先验点云文件 (PCD) 已放置在 `sentry_nav_bringup/pcd/reality/` 目录
- 2D 栅格地图已放置在 `sentry_nav_bringup/map/reality/` 目录

### 启动命令

**导航模式（使用先验地图）：**
```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py \
  world:=rmul_2026 \
  slam:=False \
  use_robot_state_pub:=True
```

**建图模式：**
```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py \
  slam:=True \
  use_robot_state_pub:=True
```

### 完整参数列表

| 参数 | 类型 | 默认值 | 说明 |
|:---|:---|:---|:---|
| `namespace` | string | `""` (空) | 机器人命名空间（空串 = 无前缀） |
| `slam` | bool | `False` | `True`=SLAM 建图；`False`=定位导航 |
| `world` | string | `rmul_2026` | 场地名，决定 map/pcd 路径 |
| `map` | string | `map/reality/{world}.yaml` | 2D 地图路径 |
| `prior_pcd_file` | string | `pcd/reality/{world}.pcd` | 先验 PCD 文件路径 |
| `use_sim_time` | bool | `False` | 使用系统真实时钟 |
| `params_file` | string | `config/reality/nav2_params.yaml` | 实车专用参数 |
| `use_robot_state_pub` | bool | `False` | 是否启动 robot_state_publisher |
| `use_rviz` | bool | `True` | 是否启动 RViz 可视化 |

### 实车专有节点
- **livox_ros_driver2**: Livox MID360 激光雷达驱动。
- **robot_state_publisher**: 发布机器人 URDF/TF（需手动通过 `use_robot_state_pub:=True` 启用）。

### 网络配置

Livox MID360 网络配置文件位于 `sentry_nav_bringup/config/reality/mid360_user_config.json`：

| 配置项 | 默认值 | 说明 |
|:---|:---|:---|
| `host_net_info.cmd_data_ip` | `192.168.1.50` | 主机 PC 的 IP 地址 |
| `lidar_configs[0].ip` | `192.168.1.150` | LiDAR 的 IP 地址 |
| `lidar_configs[0].pcl_data_type` | `1` | 1=笛卡尔32位, 2=笛卡尔16位, 3=球坐标 |
| `lidar_configs[0].pattern_mode` | `0` | 0=非重复扫描, 1=重复扫描, 2=重复低频扫描 |

> **部署提醒**：IP 地址必须根据实际网络环境修改。使用网线直连 LiDAR 时，确保主机网卡在 `192.168.1.x` 网段。

---

## 3. SLAM 建图模式

### 工作原理

SLAM 模式通过 `slam:=True` 参数触发。与导航模式的主要区别：

| 组件 | 导航模式 (slam=False) | 建图模式 (slam=True) |
|:---|:---|:---|
| Point-LIO `prior_pcd.enable` | 取决于配置 | `False`（不加载先验地图） |
| Point-LIO `pcd_save.pcd_save_en` | `False` | `True`（退出时保存 PCD） |
| small_gicp_relocalization | 启动 | **不启动** |
| slam_toolbox | 不启动 | **启动** |
| map_saver_server | 不启动 | **启动** |
| map→odom TF | 由 small_gicp 动态发布 | 静态恒等变换 |
| pointcloud_to_laserscan | 不启动 | **启动**（为 slam_toolbox 提供 2D 扫描） |

### 启动命令

```bash
# 实车建图
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True
```

### 建图后保存地图

```bash
# 实车模式（无命名空间）
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

保存后会生成两个文件：
- `my_map.pgm`: 栅格图像
- `my_map.yaml`: 地图元数据（分辨率、原点等）

> **提示**：Point-LIO 在 SLAM 模式下会自动保存 PCD 文件。该 PCD 可作为后续导航模式的先验点云使用。PCD 文件保存路径取决于 Point-LIO 的配置。

### SLAM Toolbox 关键参数

在 `nav2_params.yaml` 的 `slam_toolbox:` 段配置：

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `odom_frame` | `odom` | 里程计坐标系 |
| `map_frame` | `map` | 地图坐标系 |
| `base_frame` | `chassis` | 机器人基座坐标系 |
| `scan_topic` | `obstacle_scan` | 2D 扫描话题（由 pointcloud_to_laserscan 提供） |
| `resolution` | `0.05` | 地图分辨率 (m/像素) |
| `max_laser_range` | `10.0` | 最大激光有效距离 (m) |
| `minimum_travel_distance` | `0.5` | 触发地图更新的最小移动距离 (m) |
| `minimum_travel_heading` | `0.5` | 触发地图更新的最小旋转角度 (rad) |

---

## 4. 状态机决策系统

> **权威与完整说明见 [sentry_behavior/README.md](../../sentry_behavior/README.md)。** 本节为操作摘要。下方 4.x 中"服务端/客户端参数、可用行为树、Groot2、自定义节点、黑板"等小节为 **BehaviorTree 时代历史内容**,已整体被状态机取代(现已无 `sentry_behavior_server/client`、`target_tree`、Groot2、BT 节点/树 XML),仅作历史参考。

### 概述

`sentry_behavior_node`(自研分层状态机,脱离 BehaviorTree)订阅裁判系统话题,以 `tick_frequency`(默认 20Hz)在 `SingleThreadedExecutor` 上驱动 2 层状态机(生命周期 `WAIT_START↔IN_MATCH` × 战术 guard 表),向 `/goal_pose` 发布目标点驱动 Nav2。运行时由 `strategy` 参数选策略。

### 启动命令

```bash
# 实车环境(默认 strategy=rmuc_defend)
ros2 launch sentry_behavior sentry_behavior_launch.py use_sim_time:=False strategy:=rmuc_defend
```

### 节点参数(`params/sentry_behavior.yaml`)

| 参数 | 默认 | 说明 |
|:---|:---|:---|
| `strategy` | `rmuc_defend` | 策略名(`rmuc_defend` / `a` / `b`) |
| `tick_frequency` | `20.0` | 状态机主循环频率 (Hz) |
| `viz_enable` / `viz_port` | `true` / `1667` | 内嵌 TCP NDJSON 状态可视化协议(供独立 Rust/C 客户端) |
| `rmuc_hp_min`/`attack_hp_min`/`patrol_hp_min`/`ammo_min`/`outpost_hp_min` | `151/300/300/1/1` | 战术阈值 |

### 策略与坐标

- `rmuc_defend`:`hp>=151 && ammo>=1` → 守点 `(3.71,-0.61)`,否则补给 `(-0.27,-3.94)`。
- `a`:阈值 `hp>=300 && ammo>=1` → 攻击点 `(3.71,-0.61)`,否则补给。
- `b`:前哨存活 → `hp>=300 && ammo>=1` 巡逻 `(9.17,4.07)` / 补给;前哨倒 → 点3 `(3.27,-0.90)`。

### 切换策略

```bash
ros2 launch sentry_behavior sentry_behavior_launch.py strategy:=b
```

### 状态可视化

节点内嵌 TCP NDJSON server(`viz_port`,默认 1667),独立线程非阻塞推送 `graph`/`state`/`transition` 帧,供独立 Rust/C 客户端连接(无 ROS 依赖);`nc 127.0.0.1 1667` 可直接看。

### 回归测试

`tests/inv_smoke.sh` 注入裁判数据驱动 `strategy=rmuc_defend`,从 `/goal_pose` 抓坐标序列验证 INV-1~7(守点/补给坐标与策略同步)。

---

<!-- 以下为 BehaviorTree 时代历史内容,已被状态机取代,仅作参考 -->

### 服务端参数 (sentry_behavior_server)

配置文件：`sentry_behavior/params/sentry_behavior.yaml`

| 参数 | 类型 | 默认值 | 说明 |
|:---|:---|:---|:---|
| `action_name` | string | `sentry_behavior` | ROS2 Action 服务名 |
| `tick_frequency` | int | `2` | 行为树主循环频率 (Hz)。控制决策刷新速率 |
| `groot2_port` | int | `1667` | Groot2 可视化调试端口。设为 0 禁用 |
| `ros_plugins_timeout` | int | `1000` | ROS 插件初始化超时 (ms) |
| `use_cout_logger` | bool | `false` | 是否在终端打印节点状态切换日志 |
| `plugins` | list | `[sentry_behavior/bt_plugins]` | 插件库搜索路径 |
| `behavior_trees` | list | `[sentry_behavior/behavior_trees]` | 行为树 XML 搜索路径 |

### 客户端参数 (sentry_behavior_client)

| 参数 | 类型 | 默认值 | 说明 |
|:---|:---|:---|:---|
| `target_tree` | string | `a` | 要执行的行为树名（XML 中的 `<BehaviorTree ID="xxx">` 属性） |

### 可用行为树

当前 `src/sentry_behavior/behavior_trees/` 下仅有以下战术树（其余树名 `red` / `blue` / `rmul*` / `test_uphill` 等历史命名已删除，不再可用）：

| 文件 | 树 ID (target_tree) | 适用场景 |
|:---|:---|:---|
| `RMUC.xml` | `rmuc_2026_sentry` | RMUC 完整策略：HP/弹药滞回 + `NavigateTo` 真实 SUCCESS（一次到点即完成） |
| `a.xml` | `a` | 防守驻守 + 致命撤退补给（多场比赛永循环），用 `PubGoal` fire-and-forget 维持驻守语义 |
| `b.xml` | `b` | 在 a.xml 基础上增加 `IsOutpostStatusOK`：前哨站阵亡时改前往兜底点 3 |
| `test_navigate.xml` | `test_navigate` | 两点往返 `NavigateTo` 调试树（不依赖裁判系统） |

### 多场比赛永循环外壳（a.xml / b.xml）

a.xml / b.xml 共用以下根结构，目的是裁判帧 progress 在 `4 → 5/0 → 4` 间反复跳变（多场比赛、暂停恢复）时整树不退出：

```xml
KeepRunningUntilFailure                <!-- 一场赛 SUCCESS 后 reset+RUNNING 重启等下一场 -->
└─ Sequence
   ├─ RetryUntilSuccessful(num_attempts=-1)
   │  └─ Delay(delay_msec=500)         <!-- 500ms 限速防 spam -->
   │     └─ IsGameStatus(progress=4)   <!-- 等比赛开始 -->
   └─ ReactiveFallback
      ├─ Inverter(IsGameStatus 4)      <!-- progress!=4 时 SUCCESS → 一场赛 SUCCESS -->
      └─ KeepRunningUntilFailure       <!-- 比赛中持续运行的主决策子树 -->
         └─ ReactiveSequence ...       <!-- 里面 PubGoal 持续驻守 -->
```

设计要点：

- **根用 `ReactiveSequence` 而非 `Sequence`（RMUC.xml）**：每 tick 重判 `IsGameStatus`，否则 `Sequence` 会 latch 在已 SUCCESS 的子节点，比赛切阶段时无法及时退出。RMUC.xml 之前曾因此问题被 `fix(behavior_trees/RMUC): 根 Sequence -> ReactiveSequence` 修过。
- **`Delay(delay_msec=500)` 不可省略**：`RetryUntilSuccessful` 在子节点 FAILURE 时立即重 tick，比赛开始前 progress=0/1 持续 FAILURE 会让 Condition 节点以 ~4.5M 行/秒疯狂打日志（实测 357 MB/分钟）。Delay 把 retry 节奏限到 0.5s。
- **a/b.xml 用 `PubGoal` 而非 `NavigateTo`**：`PubGoal` 是 fire-and-forget 的 `RosTopicPubNode<PoseStamped>`，发完即返回 SUCCESS，外层 `KeepRunningUntilFailure` 持续重发，实现"持续驻守"语义；机器人到点后即使 Nav2 进入 idle 也不退出主决策。`NavigateTo` 是 `RosActionNode<NavigateToPose>`，等真实 SUCCESS（到达目标），更适合 RMUC.xml / test_navigate.xml 这类一次性导航场景。

### 策略切换

```bash
# 方法 1：修改 sentry_behavior.yaml 中的 target_tree 后重启
# 方法 2：launch 参数覆盖
ros2 launch sentry_behavior sentry_behavior_launch.py target_tree:=b
```

### Groot2 / btviz 可视化调试

行为树服务启动后，Groot2 ZMQ 服务会自动在 `groot2_port` (默认 1667) 监听。

**Groot2**：打开 Groot2 → "Connect to running tree" → 输入 `localhost:1667`（或远程 IP）。免费版有 20 节点限制，超出后部分节点不可见。

**[btviz](https://github.com/Boombroke/btviz)**（推荐）：本仓库维护者编写的 Tauri 应用，无 20 节点限制，可视化协议与 Groot2 兼容（同样连 `localhost:1667`）。在节点数较多的 a.xml / b.xml / RMUC.xml 上更顺手。

可实时查看：

- 行为树执行路径和节点状态（Success/Failure/Running）
- 全局黑板变量的实时值
- 各节点的端口参数

### 自定义行为树节点一览

> 本仓库**只**实现下列 5 个自定义节点，控制 / 装饰节点全部走 BT.CPP / BT.ROS2 自带的 `KeepRunningUntilFailure` / `RetryUntilSuccessful` / `Delay` / `WhileDoElse` / `ReactiveSequence` / `ReactiveFallback` / `Inverter` / `Sequence` / `AlwaysSuccess` 等。**没有** `RecoveryNode` / `RateController` / `TickAfterTimeout` / `Pursuit` / `BattlefieldInformation` / `IsRfidDetected` / `IsAttacked` / `IsDetectEnemy` / `IsHPAdd`，历史文档若有提及均为已删除节点。

#### 条件节点 (Condition Nodes)

| 节点名 | 黑板键 | 说明 | 关键端口 |
|:---|:---|:---|:---|
| `IsGameStatus` | `@referee_gameStatus` | 检查比赛阶段（`expected_game_progress`）与剩余秒数区间 | `expected_game_progress` (int), `min_remain_time` (int), `max_remain_time` (int) |
| `IsStatusOK` | `@referee_robotStatus` | 检查弹药 / 血量是否满足阈值；任一项越界返回 FAILURE | `ammo_min` (int, 默认 0), `ammo_max` (int, 默认 65535), `hp_min` (int, 默认 300) |
| `IsOutpostStatusOK` | `@referee_robotsHP` | 检查己方前哨站 HP；低于阈值返回 FAILURE | `outpost_hp_min` (int, 默认 1) |

#### 动作节点 (Action Nodes)

| 节点名 | 类型 | 说明 | 关键端口 |
|:---|:---|:---|:---|
| `NavigateTo` | `RosActionNode<NavigateToPose>` | 调用 Nav2 `navigate_to_pose` action，等真实 SUCCESS / FAILURE | `goal_pose_x`, `goal_pose_y`, `goal_pose_yaw` (float, rad), `action_name`, `error_code` (output, int) |
| `PubGoal` | `RosTopicPubNode<PoseStamped>` | 向 `/goal_pose` 话题发布一次目标，立即返回 SUCCESS（fire-and-forget） | `goal_pose_x`, `goal_pose_y`, `goal_pose_yaw`, `topic_name` (默认 `/goal_pose`) |

`PubGoal` 内部带去重状态（`feat(pub_goal): add state tracking to prevent duplicate goal publishing`），同坐标重复 tick 不会刷爆 `/goal_pose`。

### 黑板数据来源

行为树服务端 (`sentry_behavior_server`) 订阅以下裁判系统话题，并将消息整体写入全局黑板：

| 黑板键 (使用 `@` 前缀访问) | 订阅话题 | 消息类型 |
|:---|:---|:---|
| `referee_gameStatus` | `referee/game_status` | `rm_interfaces/GameStatus` |
| `referee_robotStatus` | `referee/robot_status` | `rm_interfaces/RobotStatus` |
| `referee_robotsHP` | `referee/all_robot_hp` | `rm_interfaces/GameRobotHP` |
| `referee_rfidStatus` | `referee/rfid_status` | `rm_interfaces/RfidStatus` |

> 视觉链黑板键 `@detector_armors` / `@tracker_target` 与 `@nav_globalCostmap` 在 omni 主线下已不再注入，依赖它们的 Pursuit / BattlefieldInformation 节点也一并移除。

---

## 5. 辅助工具

### 5.1 手柄遥控

导航 launch 文件默认内嵌启动手柄控制节点。也可独立启动：

```bash
ros2 launch sentry_nav_bringup joy_teleop_launch.py
```

PS4 手柄键位映射在 `nav2_params.yaml` 的 `teleop_twist_joy_node:` 段配置。

### 5.2 地图坐标拾取工具

使用 matplotlib 可视化地图并交互式拾取坐标（用于设定行为树中的目标点）：

```bash
ros2 launch location map_visualizer_launch.py
```

### 5.3 独立模块启动

以下模块可独立启动，用于调试或单元测试：

| 模块 | 启动命令 |
|:---|:---|
| Point-LIO 里程计 | `ros2 launch point_lio point_lio.launch.py` |
| 里程计桥接 | 已集成到 navigation_launch.py 中 (odom_bridge composable node) |
| GICP 重定位 | `ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py` |
| Livox 驱动 | `ros2 launch livox_ros_driver2 msg_MID360_launch.py` |
| 速度变换 | `ros2 launch fake_vel_transform fake_vel_transform_launch.py` |
| 里程计插值 | `ros2 launch odom_interpolator odom_interpolator_launch.py` |
| 机器人描述 | `ros2 launch sentry_robot_description robot_description_launch.py` |
| 串口驱动 | `ros2 launch serial_driver serial_driver.launch.py` |

---

## 6. Nav2 导航参数详解

参数文件位于：
- 实车：`sentry_nav_bringup/config/reality/nav2_params.yaml`

### 6.1 全局规划器

**系统使用 `nav2_smac_planner::SmacPlanner2D`**。全向底盘无最小转弯半径约束，SmacPlanner2D（2D A*）在此场景下路径更简洁，计算开销低于 Hybrid A*。

#### SmacPlanner2D

| 参数 | 值 | 说明 |
|:---|:---|:---|
| `tolerance` | `0.5` | 无法到达精确位置的规划容差 (m) |
| `allow_unknown` | `true` | 允许在未知空间中行驶 |
| `downsample_costmap` | `false` | 是否对代价地图进行下采样 |
| `max_iterations` | `1000000` | 最大搜索迭代次数，-1 禁用 |

### 6.2 局部控制器 - OmniPidPursuitController

实车使用此控制器：

| 参数 | 值 | 说明 |
|:---|:---|:---|
| `lookahead_dist` | `2.0` | 前瞻距离 (m) |
| `v_linear_max` | `2.5` | 最大线速度 (m/s) |
| `v_angular_max` | `3.0` | 最大角速度 (rad/s) |
| `curvature_max` | `1.0` | 触发减速的最大曲率阈值 |
| `reduction_ratio` | `0.5` | 高曲率时速度缩减比例 |
| `enable_rotation` | `false` | 是否旋转到目标朝向（全向底盘不需要） |
| `target_frame` | `gimbal_yaw_fake` | 控制器参考坐标系 |

### 6.3 代价地图 (Costmap2D)

#### 局部代价地图 (Local Costmap)

| 参数 | 值 | 说明 |
|:---|:---|:---|
| `update_frequency` | `30.0` | 更新频率 (Hz) |
| `publish_frequency` | `30.0` | 发布频率 (Hz) |
| `width` | `5.0` | 滚动窗口宽度 (m) |
| `height` | `5.0` | 滚动窗口高度 (m) |
| `resolution` | `0.05` | 分辨率 (m/像素) |
| `robot_radius` | `0.24` | 机器人外接圆半径 (m) |
| `inflation_radius` | `0.5` | 膨胀半径 (m) |
| `cost_scaling_factor` | `4.0` | 代价随距离衰减的指数因子 |

**插件层顺序**：`static_layer` → `IntensityVoxelLayer` → `inflation_layer`

- **IntensityVoxelLayer**: 自定义插件 (位于 `nav2_plugins` 包)，订阅 `terrain_map` 话题，利用点云强度信息标记 3D 障碍物。

#### 全局代价地图 (Global Costmap)

与局部代价地图参数结构相同，但：
- 覆盖整个场地（非滚动窗口）
- IntensityVoxelLayer 订阅 `terrain_map_ext` 话题（更大感知范围）

### 6.4 速度平滑器 (Velocity Smoother)

| 参数 | 值 | 说明 |
|:---|:---|:---|
| `smoothing_frequency` | `30.0` | 平滑处理频率 (Hz)，**必须与 `controller_frequency` 一致** |
| `max_velocity` | `[1.5, 1.5, 3.0]` | 最大速度 [vx, vy, vθ] (m/s, m/s, rad/s) |
| `min_velocity` | `[-1.5, -1.5, -3.0]` | 最小速度（反向） |
| `max_accel` | `[3.0, 3.0, 5.0]` | 最大加速度 [ax, ay, aθ] |
| `max_decel` | `[-3.0, -3.0, -5.0]` | 最大减速度 |
| `feedback` | `OPEN_LOOP` | 反馈模式。`OPEN_LOOP` 不依赖里程计反馈 |

### 6.5 恢复行为插件

| 插件名 | 说明 |
|:---|:---|
| `Spin` | 原地旋转以重新观测环境 |
| `BackUpFreeSpace` | 自定义插件：向自由空间方向后退 |
| `DriveOnHeading` | 沿当前朝向直行一段距离 |
| `Wait` | 原地等待指定时间 |
| `AssistedTeleop` | 辅助遥控模式 |

### 6.6 Nav2 导航行为树

| 文件 | 用途 |
|:---|:---|
| `navigate_to_pose_w_replanning_and_recovery.xml` | 单目标导航：3Hz 重规划 + 代价地图清除 + 后退恢复 |
| `navigate_through_poses_w_replanning_and_recovery.xml` | 多航点导航：同上 + 已通过航点自动移除 (radius=0.7m) |

---

## 7. 定位模块参数详解

### 7.1 Point-LIO 里程计

| 参数 | 值 | 说明 |
|:---|:---|:---|
| `common.lid_topic` | `livox/lidar` | LiDAR 输入话题 |
| `common.imu_topic` | `livox/imu` | IMU 输入话题 |
| `preprocess.lidar_type` | `1` (Livox) | LiDAR 类型: 1=Livox, 2=Velodyne, 3=Ouster |
| `preprocess.scan_line` | `4` | 扫描线数（MID360 实际为 4 线） |
| `preprocess.timestamp_unit` | `3` (纳秒) | 实车 Livox 配置 |
| `preprocess.blind` | `0.2` | 盲区半径 (m)，过滤近距离噪声 |
| `mapping.acc_norm` | `1.0` | 加速度单位：1.0=g |
| `mapping.satu_acc` | `4.0` | IMU 加速度计饱和值 |
| `mapping.gravity` | `[2.64, 0, -9.68]` | **重力向量，必须匹配 LiDAR 安装倾角** |
| `mapping.gravity_init` | `[2.64, 0, -9.68]` | 初始重力估计 |
| `mapping.extrinsic_T` | `[-0.011, -0.023, 0.044]` | LiDAR-IMU 外参平移 (m) |
| `mapping.lidar_meas_cov` | `0.01` | LiDAR 测量协方差 |
| `filter_size_surf` | `0.2` | 表面特征降采样步长 (m) |
| `filter_size_map` | `0.2` | 地图降采样步长 (m) |
| `publish.tf_send_en` | `False` | TF 发布（禁用，由 odom_bridge 处理） |
| `pcd_save.pcd_save_en` | `False` | 退出时保存 PCD（SLAM 模式自动设为 True） |

> **调参警告**：`gravity` 向量必须精确匹配 LiDAR 的物理安装角度。错误的重力向量会导致里程计快速发散。

### 7.2 Small GICP 重定位

| 参数 | 值 | 说明 |
|:---|:---|:---|
| `num_threads` | `8` | OpenMP 并行线程数 |
| `num_neighbors` | `20` | 协方差估计邻域点数 |
| `global_leaf_size` | `0.2` | 先验地图体素降采样步长 (m) |
| `registered_leaf_size` | `0.1` | 累积扫描体素降采样步长 (m) |
| `max_dist_sq` | `3.0` | GICP 对应点最大距离平方 (m²) |
| `max_iterations` | `20` | GICP 优化器最大迭代次数 |
| `accumulated_count_threshold` | `20` | 累积多少帧点云后触发一次配准 |
| `min_range` | `0.5` | 最小点云距离过滤 (m) |
| `min_inlier_ratio` | `0.3` | 最小内点比率 |
| `max_fitness_error` | `1.0` | 最大每内点适配误差 |
| `enable_periodic_relocalization` | `false` | 是否启用周期性重定位 |
| `relocalization_interval` | `30.0` | 周期性重定位间隔 (秒) |
| `map_frame` | `map` | 地图坐标系 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `prior_pcd_file` | (由 launch 传入) | 先验 PCD 地图文件路径 |

**质量门控机制**：
1. GICP 必须报告收敛
2. 内点比率 ≥ `min_inlier_ratio`
3. 平均适配误差 ≤ `max_fitness_error`
4. 周期性修正幅度 < 2m（硬编码上限）
5. **2D 约束**：输出的 map→odom 仅包含 (x, y, yaw)，z/roll/pitch 强制置零

> **调参提示**：修改这些参数时以 `config/reality/` 配置为准。

### 7.3 Livox MID360 驱动参数

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `xfer_format` | `4` | 数据格式: 0=PointCloud2, 1=CustomMsg, 4=AllMsg(推荐) |
| `publish_freq` | `30.0` | 发布频率 (Hz)。可选: 5, 10, 20, 30, 50 |
| `frame_id` | `front_mid360` | TF 坐标系名称 |
| `multi_topic` | `0` | 0=共享话题, 1=每个 LiDAR 独立话题 |

---

## 8. 地形分析参数详解

### 8.1 Terrain Analysis (局部地形)

订阅 `sensor_scan` + `odometry`，发布 `terrain_map` → 供**局部代价地图** IntensityVoxelLayer 使用。

| 参数 | 值 | 说明 |
|:---|:---|:---|
| `scanVoxelSize` | `0.05` | 点云降采样步长 (m) |
| `decayTime` | `0.5` | 点云衰减时间 (秒)，超过此时间的点被丢弃 |
| `noDecayDis` | `0.0` | 此距离内的点不衰减 (m) |
| `clearingDis` | `0.0` | 超出此距离的点被清除 (m)，0=禁用 |
| `useSorting` | `true` | 使用分位数地面估计（支持坡道识别）。**不可与 considerDrop 同时启用** |
| `quantileZ` | `0.2` | Z 方向分位数（仅 useSorting=true 时生效） |
| `considerDrop` | `false` | 考虑凹地形（绝对高度差） |
| `clearDyObs` | `true` | 清除动态障碍物 |
| `minDyObsDis` | `0.5` | 动态障碍物最小检测距离 (m) |
| `vehicleHeight` | `0.55` | 机器人高度 (m)，仅处理低于此高度的点 |
| `minRelZ` | `-1.5` | 有效点最低相对高度 (m) |
| `maxRelZ` | `0.5` | 有效点最高相对高度 (m) |
| `disRatioZ` | `0.2` | Z 范围随距离缩放因子（坡道补偿） |
| `minBlockPointNum` | `10` | 每个体素块最少点数 |
| `noDataObstacle` | `false` | 无数据区域视为障碍物 |

### 8.2 Terrain Analysis Ext (全局地形)

订阅 `terrain_map`，发布 `terrain_map_ext` → 供**全局代价地图** IntensityVoxelLayer 使用。

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `scanVoxelSize` | `0.1` | 更粗的降采样（全局范围） |
| `decayTime` | `0.2` | 更快衰减以保持全局地图新鲜度 |
| `clearingDis` | `20.0` | 扩展清除距离 (m) |
| `vehicleHeight` | `1.0` | 更高的过滤阈值 |
| `localTerrainMapRadius` | `4.0` | 局部地形地图半径 (m) |
| `ceilingFilteringThre` | `2.0` | 天花板过滤阈值 (m) |
| `checkTerrainConn` | `false` | 检查地形连通性 |

---

## 9. 串口通信参数

### 配置文件

`serial/serial_driver/config/serial_driver.yaml`

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `device_name` | `/dev/ttyACM0` | 串口设备路径 |
| `baud_rate` | `115200` | 波特率 |
| `flow_control` | `none` | 流控: `none`, `hardware`, `software` |
| `parity` | `none` | 校验: `none`, `odd`, `even` |
| `stop_bits` | `"1"` | 停止位: `1`, `1.5`, `2` |

### 通信协议

**上行 (MCU → ROS2)**：`ReceiveNavPacket`，帧头 `0x5B`，CRC16 校验

| 字段 | 类型 | 说明 |
|:---|:---|:---|
| `pitch`, `yaw` | float | 云台关节角度 |
| `game_progress` | uint8 | 比赛阶段 (0-5) |
| `stage_remain_time` | uint16 | 当前阶段剩余时间 (秒) |
| `current_hp` | uint16 | 机器人当前血量 |
| `projectile_allowance_17mm` | uint16 | 剩余弹药量 |
| `red_*/blue_*_robot_hp` | uint16 | 全场 14 台机器人血量 |
| `team_colour` | uint16 | 队伍颜色 (1=红方) |
| `tracker_x`, `tracker_y` | float | 追踪目标位置 |
| `tracking` | bool | 是否正在追踪 |

**下行 (ROS2 → MCU)**：`SendNavPacket`，帧头 `0xB5`，CRC16 校验

| 字段 | 类型 | 说明 |
|:---|:---|:---|
| `vel_x`, `vel_y` | float | 线速度指令 (m/s) |
| `vel_v` | float | 角速度指令（映射自 `cmd_vel.angular.z`） |

### 发布话题

| 话题 | 消息类型 | 说明 |
|:---|:---|:---|
| `serial/gimbal_joint_state` | `JointState` | 云台 pitch/yaw 关节状态 |
| `referee/game_status` | `GameStatus` | 比赛阶段 + 剩余时间 |
| `referee/robot_status` | `RobotStatus` | 血量 + 弹药 |
| `referee/all_robot_hp` | `GameRobotHP` | 全场机器人血量 + 队伍颜色 |
| `referee/rfid_status` | `RfidStatus` | RFID 区域检测 |
| `tracker/target` | `Target` | 追踪目标位姿 |

---

## 10. 常见调参场景

### 场景 1: 机器人频繁撞墙

- 增大 `inflation_radius`（如 0.5 → 0.8）
- 增大 `robot_radius`（确保覆盖实际外形）
- 降低 `v_linear_max`（减速增加反应时间）
- 检查 `costmap update_frequency` 是否足够高

### 场景 2: 路径规划失败 / 找不到路径

- 减小 `inflation_radius`（给规划器留出更多通行空间）
- 检查静态地图是否准确反映实际环境
- 增大 `SmacPlannerHybrid.max_iterations` 或降低 `cost_travel_multiplier`
- 确认 `robot_radius` 不大于最窄通道宽度的一半

### 场景 3: 重定位不收敛

- 增大 `max_dist_sq`（放宽对应点搜索范围）
- 降低 `min_inlier_ratio`（降低匹配质量要求）
- 增大 `max_fitness_error`（提高容忍度）
- 增大 `accumulated_count_threshold`（累积更多帧再配准）
- 确认先验 PCD 地图与当前环境一致

### 场景 4: Point-LIO 里程计发散

- **首先检查 `gravity` 向量**是否匹配 LiDAR 实际安装角度
- 确认 `lidar_type` 和 `timestamp_unit` 设置正确
- 检查 IMU 数据质量，调整 `satu_acc` / `satu_gyro`
- 增大 `lidar_meas_cov`（降低 LiDAR 测量信任度）

### 场景 5: 状态机不发目标 / 无动作

- 确认裁判系统话题正在发布数据：`ros2 topic echo /referee/game_status`、`/referee/robot_status`
- 确认 `game_progress==4`（未开赛时停在 `WAIT_START`，本就不发 `/goal_pose`，符合预期）
- 检查 `strategy` 参数是否为有效策略名（`rmuc_defend` / `a` / `b`；未知名节点会 FATAL 退出）
- 连接状态可视化看活动状态：`nc 127.0.0.1 <viz_port>`（默认 1667）
- 确认 Nav2 导航栈已正常启动（状态机通过 `/goal_pose` 发布目标）

### 场景 6: 需要切换比赛策略

1. 修改 `sentry_behavior.yaml` 中的 `strategy` 值
2. 或在 launch 命令中指定：`ros2 launch sentry_behavior sentry_behavior_launch.py strategy:=b`
3. 可选策略：`rmuc_defend` / `a` / `b`（参见[第 4 节](#4-状态机决策系统)）
