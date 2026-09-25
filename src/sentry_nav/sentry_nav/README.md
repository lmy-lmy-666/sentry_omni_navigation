# sentry_nav

`sentry_nav` 是哨兵机器人自主导航系统的顶层元包（meta-package），不含实际节点代码，仅通过 `package.xml` 依赖声明聚合导航子包，确保 `colcon build` 按正确顺序编译。

## 子包列表

### 自研包

| 包名 | 说明 |
|---|---|
| `odom_bridge` | Point-LIO → TF / Odometry / registered_scan 桥接，单次同步回调完成 lidar_odom→odom 变换、2D 约束、TF 广播、点云转换 |
| `fake_vel_transform` | 速度坐标变换节点：将 Nav2 输出从 `gimbal_yaw_fake` 系旋回 `gimbal_yaw` 系，并叠加 `spin_speed`，输出到 `/cmd_vel_chassis` |
| `omni_pid_pursuit_controller` | 全向 PID 路径跟踪控制器，实现 `nav2_core::Controller` 插件接口，插件名 `omni_pid_pursuit_controller::OmniPidPursuitController` |
| `nav2_plugins` | 自定义 Nav2 插件集：`IntensityVoxelLayer`（强度体素代价地图层）+ `BackUpFreeSpace`（自由空间后退恢复） |
| `small_gicp_relocalization` | 基于 small_gicp 的全局重定位，对齐先验 PCD 地图，输出 `map→odom` TF |

### 第三方 Fork 包

| 包名 | 上游来源 | 说明 |
|---|---|---|
| `point_lio` | [HKU-MARS](https://github.com/hku-mars/Point-LIO) | 激光惯性紧耦合里程计，提供高频位姿估计 |
| `terrain_analysis` | [CMU](https://github.com/jizhang-cmu/autonomous_exploration_development_environment) | 基础地形分析，实时障碍物检测 |
| `terrain_analysis_ext` | CMU 同源 | 地形分析扩展，改善坡道与复杂障碍物通过性 |
| `livox_ros_driver2` | Livox 官方 | Livox Mid360 / HAP ROS2 驱动 |
| `pointcloud_to_laserscan` | ROS 官方 | 3D 点云投影为 2D 激光扫描 |

## 编译

通常直接在工作空间根目录全量编译：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

单独编译：

```bash
colcon build --packages-select sentry_nav --symlink-install
```

> **注意**：`package.xml` 当前仅显式声明了 `fake_vel_transform` 和 `odom_bridge` 两个 `<depend>`。
> 其余子包需确保在工作空间 `src/sentry_nav/` 下存在，colcon 会自动发现并编译。

