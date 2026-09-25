# sentry_match_recorder

## 概述

`sentry_match_recorder` 是哨兵机器人**比赛期间自动 rosbag 录制**功能包。订阅 `/referee/game_status`，在 `game_progress` 连续 `debounce_count` 帧（默认 3 帧）匹配 `target_progress`（默认 4）后自动启动 `ros2 bag record`，连续 3 帧不匹配后自动停止。走 rosbag2 内建 `--max-bag-duration` 在单个进程内滚动切片，单场比赛全部切片输出到一个 `sortie_YYYYMMDD_HHMMSS/` 目录。

## 工作原理

### 状态机

```
IDLE  ──── consec_active >= debounce_count ───► RECORDING
      ◄─── consec_inactive >= debounce_count ──
```

每收到一帧 `game_progress`：若与 `target_progress` 相符则 `consec_active++`（同时清零 `consec_inactive`），否则反向计数。仅当连续 N 帧一致才发生状态跳变，防止裁判帧瞬时抖动误触。

- **上升沿**：`mkdir -p logs/match-bags/sortie_YMDHMS/` → spawn `ros2 bag record -o <sortie_dir> --storage mcap --max-bag-duration 60 <topics...>`
- **下降沿**：对子进程组发 `SIGINT`，等待 10s；超时升级 `SIGTERM` 再等 5s，保证最后一个切片 flush 完整

### 切片机制

走 rosbag2 内建 `--max-bag-duration 60`：单个 `ros2 bag record` 进程从比赛开始跑到结束，rosbag2 自身按时长滚动 `*_0.mcap`、`*_1.mcap` …，全部落在同一个 `sortie_*/` 目录，切片间零缝隙。

### Watchdog（0.5s 周期）

| 场景 | 行为 |
|---|---|
| `ros2 bag record` 异常退出 | 状态强制回 IDLE，ERROR 日志；下次上升沿重新创建新 sortie |
| 单场时长超过 `max_session_seconds`（默认 480s） | 强制 SIGINT 停录（防裁判帧异常卡 4 漏停） |
| 节点退出（Ctrl-C / launch 关闭） | `destroy_node()` 与 `main` finally 里发 SIGINT 优雅停 |

## ROS 接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/referee/game_status` | `rm_interfaces/msg/GameStatus` | Subscription | 状态机唯一输入；QoS = `qos_profile_sensor_data`（best-effort） |

录制通过 `ros2 bag record` 子进程实现，节点本身不发布任何话题。

## 参数（`config/recorder.yaml`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `record_dir` | `logs/match-bags` | 录制根目录（相对仓库根，可改绝对路径） |
| `sortie_prefix` | `sortie` | 单场目录前缀，最终目录名为 `<prefix>_YYYYMMDD_HHMMSS` |
| `slice_seconds` | `60` | 每个 `.mcap` 切片时长（秒） |
| `storage_id` | `mcap` | rosbag2 存储后端，`mcap` / `sqlite3` |
| `debounce_count` | `3` | 状态转换需要的连续帧数（裁判帧约 10Hz → 0.3s） |
| `max_session_seconds` | `480` | 单场最长录制时长（8 分钟），超时强制停录 |
| `target_progress` | `4` | 触发录制的 `game_progress` 值（4 = 比赛中） |
| `topics_file` | `""` | 话题白名单 YAML 路径；空字符串走包内 share 的 `topics.yaml` |
| `extra_topics` | `[]` | 在白名单基础上追加录制的话题 |
| `exclude_topics` | `[]` | 从合并后列表中剔除的话题 |

最终录制话题 = `(topics_file 加载) + extra_topics - exclude_topics`，按出现顺序去重。

## 默认录制话题（`topics.yaml`）

| 类别 | 话题 |
|---|---|
| TF + 云台关节 | `/tf`、`/tf_static`、`/serial/gimbal_joint_state` |
| 传感器原始 | `/livox/lidar_primary`、`/livox/lidar_secondary`、`/livox/imu_primary`、`/livox/imu_secondary` |
| 里程计 / 配准点云 | `/odometry`、`/lidar_odometry`、`/cloud_registered`、`/registered_scan` |
| Nav2 控制链 | `/plan`、`/local_plan`、`/received_global_plan`、`/cmd_vel_nav`、`/cmd_vel_chassis`、`/cmd_vel` |
| Costmap | `/global_costmap/costmap`、`/global_costmap/costmap_updates`、`/local_costmap/costmap`、`/local_costmap/costmap_updates` |
| 裁判系统 | `/referee/game_status`、`/referee/robot_status`、`/referee/all_robot_hp`、`/referee/rfidStatus` |

需要新增/裁剪时优先用 `extra_topics` / `exclude_topics` 参数，而非直接修改 `topics.yaml`。

## 编译与启动

```bash
colcon build --packages-select sentry_match_recorder --symlink-install
source install/setup.bash
```

### 方式 A：实车一键启动（推荐）

`rm_sentry_launch.py` 已内置 `enable_recorder` 开关（默认 `True`），通过 `TimerAction(period=5.0)` 延迟 5s 启动，避开 nav 栈/串口刚启动时话题未就绪：

```bash
ros2 launch sentry_nav_bringup rm_sentry_launch.py                         # 默认带录包
ros2 launch sentry_nav_bringup rm_sentry_launch.py enable_recorder:=False  # 临时禁用
```

### 方式 B：独立启动

```bash
ros2 launch sentry_match_recorder match_recorder_launch.py
# 覆盖录制目录:
ros2 launch sentry_match_recorder match_recorder_launch.py record_dir:=/data/match_bags
```

### 方式 C：直接运行节点

```bash
ros2 run sentry_match_recorder match_recorder_node \
  --ros-args --params-file install/sentry_match_recorder/share/sentry_match_recorder/config/recorder.yaml
```

## 输出目录结构

```
logs/match-bags/
├── sortie_20260521_143012/
│   ├── metadata.yaml
│   ├── sortie_20260521_143012_0.mcap   # 0~60s
│   ├── sortie_20260521_143012_1.mcap   # 60~120s
│   └── ...
└── sortie_20260521_153504/
    └── ...
```

`logs/` 已加入 `.gitignore`，为本地产物。

## 离线回放

```bash
# 整场连续回放（rosbag2 ≥ Jazzy 支持直接传目录）
ros2 bag play logs/match-bags/sortie_20260521_143012

# 查看信息
ros2 bag info logs/match-bags/sortie_20260521_143012
```

### 切片合并（`merge_sortie` CLI）

若需要一个独立 mcap 文件用于离线分析或 Foxglove Studio：

```bash
# 合并到 <SORTIE_DIR>/<basename>_full.mcap，保留原切片
ros2 run sentry_match_recorder merge_sortie logs/match-bags/sortie_20260521_143012

# 自定义输出路径
ros2 run sentry_match_recorder merge_sortie logs/match-bags/sortie_20260521_143012 \
    --output /tmp/match01.mcap

# 合并后删除原切片
ros2 run sentry_match_recorder merge_sortie logs/match-bags/sortie_20260521_143012 \
    --remove-shards

# 仅预览不写文件
ros2 run sentry_match_recorder merge_sortie logs/match-bags/sortie_20260521_143012 \
    --dry-run
```

实现基于 `rosbag2_py` SequentialReader/Writer 直接拷贝序列化字节，不依赖具体消息类型；切片顺序优先读 `metadata.yaml`，否则按文件名末尾 `_<N>` 数字排序。

## 注意事项

- **磁盘空间**：默认 25 个话题 + 双 Mid360 + LIO IMU，单场 7 分钟约 1~3 GB，比赛日前确认 IPC 剩余空间
- **不要 SIGKILL**：强杀 `ros2 bag record` 会导致最后一个 mcap 索引损坏；本节点统一走 `SIGINT → SIGTERM` 升级路径
- **多实例**：若同时跑两个实例，需通过 `record_dir` / `sortie_prefix` 区分输出目录

## 目录结构

```
sentry_match_recorder/
├── config/recorder.yaml                # 节点参数默认值
├── launch/match_recorder_launch.py     # 独立启动入口
├── sentry_match_recorder/
│   ├── match_recorder_node.py          # 状态机 + subprocess 管理
│   ├── merge_sortie.py                 # 切片合并 CLI（独立入口）
│   └── topics.yaml                     # 默认录制话题白名单
└── test/test_merge_sortie.py           # merge_sortie 单元测试
```
