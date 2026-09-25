# small_gicp_relocalization

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 简介

`small_gicp_relocalization` 使用 [small_gicp](https://github.com/koide3/small_gicp) 将累积注册点云（`registered_scan`，`odom` 系）与先验 PCD 地图对齐，发布 `map` → `odom` TF，为 Nav2 提供全局定位。

输出变换约束为 2D（z=0, roll=0, pitch=0），与 Nav2 标准定位（AMCL）一致。

### 工作流程

1. 启动时加载 `prior_pcd_file`，下采样（`global_leaf_size`）并构建 KdTree
2. 持续积累 `registered_scan`，每 `accumulated_count_threshold` 帧触发一次 GICP 注册
3. GICP 结果通过三重质量门控（收敛检查 + inlier_ratio + fitness_error）后才接受
4. 接受后以 50 ms 周期（20 Hz）持续广播 `map` → `odom` TF

### 重定位层次（三层架构）

| 层次 | 触发条件 | 说明 |
|------|----------|------|
| **初始定位** | 冷启动，累积帧数达阈值 | 可选先走 Scan Context 全局粗定位，再用 GICP 精调 |
| **周期定位** | `enable_periodic_relocalization=true`，每 `relocalization_interval` 秒 | 小修正（`max_correction_distance` 硬限制）；连续 `emergency_consecutive_failures` 次失败后触发紧急模式（扩大搜索半径 + 多 yaw 种子） |
| **深度验证** | `enable_deep_verification=true`，定时器触发 | 用更精细参数在后台线程独立验证，不阻塞主流程 |

若 `enable_global_relocalization=true` 且提供 `.scdb` 数据库，健康监控（`health_monitor`）判定定位异常时自动触发 Scan Context 全局重定位。

### 先验 PCD 坐标系要求

先验 PCD 文件必须在 `odom`/`map` 系下保存（由当前建图流程生成）。旧版 `lidar_odom` 系 PCD 不兼容，需重新建图。

## 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `registered_scan` | `sensor_msgs/msg/PointCloud2` | `odom_bridge` 发出的 `odom` 系注册点云 |
| `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz 手动初始化位姿；接受后禁用所有自动重定位，直到节点重启 |

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| TF `map` → `odom` | — | 20 Hz 持续广播 |
| `map_clearing` | `std_msgs/msg/Float32` | 大幅修正后通知清除代价图 |
| `cloud_clearing` | `std_msgs/msg/Float32` | 大幅修正后通知清除点云缓存 |

## 核心参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `prior_pcd_file` | string | `""` | 先验 PCD 文件路径（必填） |
| `init_pose` | double[6] | `[0,0,0,0,0,0]` | 初始位姿 [x,y,z,roll,pitch,yaw]，作为 GICP 初值 |
| `map_frame` | string | `map` | 地图坐标系 |
| `odom_frame` | string | `odom` | 里程计坐标系 |
| `robot_base_frame` | string | `""` | 机器人坐标系（用于全局重定位 TF 查询） |
| `num_threads` | int | `4` | OpenMP 线程数 |
| `global_leaf_size` | float | `0.25` | 先验地图体素下采样分辨率（m） |
| `registered_leaf_size` | float | `0.25` | 累积点云体素下采样分辨率（m） |
| `max_dist_sq` | float | `1.0` | GICP 点对最大匹配距离平方（m²） |
| `max_iterations` | int | `20` | GICP 最大迭代次数 |
| `accumulated_count_threshold` | int | `20` | 触发 GICP 的积累帧数 |
| `min_range` | double | `0.5` | 近距离噪声过滤半径（m） |
| `min_inlier_ratio` | double | `0.3` | 接受 GICP 的最小内点率 |
| `max_fitness_error` | double | `0.05` | 接受 GICP 的最大 per-inlier 误差 |
| `enable_periodic_relocalization` | bool | `false` | 初始定位成功后是否周期性重定位 |
| `relocalization_interval` | double | `30.0` | 周期重定位间隔（秒） |
| `max_correction_distance` | double | `5.0` | 周期修正的最大允许位移（m） |
| `quality_convergence_threshold` | double | `0.008` | 未正式收敛时的质量代理阈值（per-point error） |
| `terrain_clearing_threshold` | double | `0.1` | 修正距离超过此值时触发代价图清除（m） |
| `enable_deep_verification` | bool | `false` | 启用深度验证（后台线程，更精细参数） |
| `enable_global_relocalization` | bool | `false` | 启用 Scan Context 全局重定位 |
| `scan_context_db_file` | string | `""` | Scan Context 数据库路径（`.scdb`） |

## 质量门控

GICP 结果必须同时满足以下条件才被接受：

1. **收敛**：`converged=true` 或 per-point error < `quality_convergence_threshold`
2. **内点率**：`num_inliers / source_points >= min_inlier_ratio`
3. **适配误差**：`total_error / num_inliers <= max_fitness_error`
4. **修正距离**（周期模式）：位移 <= `max_correction_distance`

## 使用说明

```bash
# 在 launch 文件中配置先验 PCD
# 确认 PCD 在 odom/map 系（非 lidar_odom 系）
ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py

# RViz 手动初始化（接受后禁用自动重定位直到重启）
# 在 RViz 中点击 "2D Pose Estimate" 并拖拽到正确位置
```

Given a registered pointcloud (based on the `odom` frame) and a prior PCD map that is already saved in the same `odom`/`map` coordinate frame, the node calculates the transformation between the two point clouds and publishes the correction from the `map` frame to the `odom` frame.

PCD frame policy for this project: newly generated prior PCD files must be saved in the `odom`/`map` frame. Old `lidar_odom`-frame PCD files are intentionally not supported; regenerate them with the current mapping flow before using navigation/relocalization.

The published map→odom transform is constrained to 2D (x, y, yaw only), consistent with standard 2D navigation localization (e.g. AMCL).

## Dependencies

- ROS2 Humble
- small_gicp (>= v1.0.0, C++17)
- pcl
- OpenMP

## Build

```zsh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/LihanChen2004/small_gicp_relocalization.git

cd ..
```

1. Install dependencies

    ```zsh
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. Build

    ```zsh
    colcon build --symlink-install -DCMAKE_BUILD_TYPE=release
    ```

## Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `num_threads` | int | 4 | OpenMP thread count for GICP |
| `num_neighbors` | int | 20 | Neighbors for covariance estimation |
| `global_leaf_size` | float | 0.25 | Voxel leaf size for prior map downsampling |
| `registered_leaf_size` | float | 0.25 | Voxel leaf size for accumulated scan downsampling |
| `max_dist_sq` | float | 1.0 | Max squared distance for GICP correspondence rejection |
| `max_iterations` | int | 20 | Max LM optimizer iterations for GICP |
| `accumulated_count_threshold` | int | 20 | Number of scan frames to accumulate before registration |
| `min_range` | double | 0.5 | Min point range filter (removes near-range noise) |
| `min_inlier_ratio` | double | 0.3 | Min inlier ratio to accept GICP result |
| `max_fitness_error` | double | 1.0 | Max per-inlier fitness error to accept GICP result |
| `enable_periodic_relocalization` | bool | false | Enable periodic re-registration after initial localization |
| `relocalization_interval` | double | 30.0 | Interval (seconds) between periodic relocalization attempts |
| `map_frame` | string | "map" | Map frame ID |
| `odom_frame` | string | "odom" | Odom frame ID |
| `base_frame` | string | "" | Legacy parameter; not used by this node |
| `robot_base_frame` | string | "" | Robot base frame (for initialpose callback) |
| `lidar_frame` | string | "" | Legacy parameter; not used by this node |
| `prior_pcd_file` | string | "" | Path to prior PCD map file |
| `init_pose` | double[] | [0,0,0,0,0,0] | Initial pose [x, y, z, roll, pitch, yaw] |

## Usage

1. Set prior pointcloud file in [launch file](launch/small_gicp_relocalization_launch.py)

2. Ensure the prior PCD map is in the `odom`/`map` frame

    The node treats `prior_pcd_file` as already aligned with `registered_scan`. It loads and prepares the target map immediately after the PCD file is read, without waiting for or applying `odom_to_lidar_odom`.

    If the PCD was generated by an old workflow and is still in `lidar_odom`, it must be regenerated. No compatibility transform or fallback mode is provided.

3. Run

    ```zsh
    ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py
    ```

## Quality Gates

The node validates GICP results before accepting them:

1. **Convergence check**: GICP must report convergence
2. **Inlier ratio**: `num_inliers / source_points >= min_inlier_ratio`
3. **Fitness error**: `total_error / num_inliers <= max_fitness_error`
4. **Periodic correction bound**: Periodic relocalization rejects corrections > 2m

If the initial registration fails quality checks, the node automatically retries with newly accumulated frames.
