# odom_bridge

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 简介

`odom_bridge` 将原 `loam_interface` 与 `sensor_scan_generation` 合并为单一 Composable Node。节点同步订阅 Point-LIO 里程计与注册点云，在一次回调中完成以下全部工作：

1. **lidar_odom 偏移初始化**：首帧时查 TF `base_frame` → `lidar_frame`，结合 Point-LIO 首帧重力对齐旋转，计算 `odom` → `lidar_odom` 的静态偏移并 latched 发布
2. **2D 约束 TF 广播**：将 `odom` → `base_frame` 变换限制到 2D（z=0, roll=0, pitch=0）后广播，让 Nav2 始终工作在平面坐标系
3. **里程计发布**：以 `odom` → `robot_base_frame` 发布 `nav_msgs/Odometry`，twist 由相邻帧位置差分计算
4. **点云转换与发布**：
   - `sensor_scan`：注册点云变换到 `lidar_frame`（供 Point-LIO 管道使用）
   - `registered_scan`：注册点云变换到 `odom` 系（供 `terrain_analysis` / `small_gicp_relocalization`）
   - `lidar_odometry`：`odom` → `lidar_frame` 的里程计（供 `terrain_analysis_ext`）

两路输入通过 `message_filters::ApproximateTime` 同步（队列深度 100）。

## 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `state_estimation_topic` | `nav_msgs/msg/Odometry` | Point-LIO 原始里程计，默认 `aft_mapped_to_init` |
| `registered_scan_topic` | `sensor_msgs/msg/PointCloud2` | Point-LIO 注册点云，默认 `cloud_registered` |

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `sensor_scan` | `sensor_msgs/msg/PointCloud2` | 注册点云转换到 `lidar_frame` |
| `odometry` | `nav_msgs/msg/Odometry` | `odom` → `robot_base_frame`，含位置差分速度 |
| `registered_scan` | `sensor_msgs/msg/PointCloud2` | 注册点云在 `odom` 系，供下游使用 |
| `lidar_odometry` | `nav_msgs/msg/Odometry` | `odom` → `lidar_frame` |
| `odom_to_lidar_odom` | `geometry_msgs/msg/TransformStamped` | latched，`odom` → `lidar_odom` 静态偏移 |

## TF 广播

`odom` → `base_frame`（2D 约束，z=0, roll=0, pitch=0）

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `state_estimation_topic` | string | `aft_mapped_to_init` | Point-LIO 里程计话题 |
| `registered_scan_topic` | string | `cloud_registered` | Point-LIO 注册点云话题 |
| `odom_frame` | string | `odom` | 里程计坐标系 |
| `base_frame` | string | `base_footprint` | 机器人底盘平面坐标系（2D TF 目标） |
| `lidar_frame` | string | `front_mid360` | LiDAR 坐标系 |
| `robot_base_frame` | string | `gimbal_yaw` | 里程计发布目标坐标系 |

## 与上下游的关系

```
Point-LIO
  ├─ aft_mapped_to_init  ──┐
  └─ cloud_registered    ──┴── odom_bridge ──┬── TF: odom → base_footprint  ──▶ Nav2
                                              ├── odometry                    ──▶ velocity_smoother
                                              ├── registered_scan             ──▶ terrain_analysis
                                              │                               ──▶ small_gicp_relocalization
                                              ├── lidar_odometry              ──▶ terrain_analysis_ext
                                              └── sensor_scan                 ──▶ (诊断/可视化)
```
