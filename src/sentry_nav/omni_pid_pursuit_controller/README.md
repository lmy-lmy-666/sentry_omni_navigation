# omni_pid_pursuit_controller

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 简介

`omni_pid_pursuit_controller` 是专为全向麦克纳姆底盘设计的 Nav2 控制器插件，实现纯跟踪（Pure Pursuit）+ 双 PID（平移 / 旋转）的路径跟踪算法。

**插件类名**：`omni_pid_pursuit_controller::OmniPidPursuitController`
**插件基类**：`nav2_core::Controller`

核心特性：
- **全向速度分解**：`cmd_vel.linear.x/y` 由 lookahead 方向角直接分解，无需旋转到正前方再走直线
- **速度缩放 lookahead**：可按当前速度大小动态伸缩前视距离（`use_velocity_scaled_lookahead_dist`）
- **曲率限速**：在 lookahead 点附近三点拟合曲率半径，高曲率时自动降低线速度上限，带 EMA 低通滤波（α=0.3）防止路径锯齿引起振荡
- **路径弧长跟踪**：lookahead 点按弧长而非欧氏距离计算，避免 U 形弯道中跨腿跳跃
- **障碍物检测**：对 lookahead 直线路径做 Bresenham 追踪；若直线被致命障碍遮挡，退回到路径切线方向；对采样路径点做 costmap 碰撞检查，碰撞时立即停车
- **接近减速**：临近目标时按剩余弧长缩放速度
- **动态参数**：所有主要参数支持 `ros2 param set` 运行时修改

## 插件配置

在 `controller_server` 参数文件中：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "omni_pid_pursuit_controller::OmniPidPursuitController"
      # --- PID ---
      translation_kp: 3.0
      translation_ki: 0.1
      translation_kd: 0.3
      enable_rotation: true
      rotation_kp: 3.0
      rotation_ki: 0.1
      rotation_kd: 0.3
      min_max_sum_error: 1.0
      # --- Lookahead ---
      lookahead_dist: 0.3
      use_velocity_scaled_lookahead_dist: true
      lookahead_time: 1.0
      min_lookahead_dist: 0.2
      max_lookahead_dist: 1.0
      use_interpolation: true
      # --- Rotation ---
      use_rotate_to_heading: true
      use_rotate_to_heading_treshold: 0.1
      # --- Velocity limits ---
      v_linear_min: -3.0
      v_linear_max: 3.0
      v_angular_min: -3.0
      v_angular_max: 3.0
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      # --- Curvature limiting ---
      curvature_min: 0.4
      curvature_max: 0.7
      reduction_ratio_at_high_curvature: 0.5
      curvature_forward_dist: 0.7
      curvature_backward_dist: 0.3
      max_velocity_scaling_factor_rate: 0.9
      # --- Misc ---
      transform_tolerance: 0.1
      max_robot_pose_search_dist: 3.0   # 默认取 costmap 半径
```

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `translation_kp/ki/kd` | 3.0 / 0.1 / 0.3 | 平移 PID 增益 |
| `enable_rotation` | `true` | 是否启用旋转 PID；`false` 时 `twist.angular.z` 恒为 0 |
| `rotation_kp/ki/kd` | 3.0 / 0.1 / 0.3 | 旋转 PID 增益 |
| `min_max_sum_error` | 1.0 | PID 积分限幅（防 windup） |
| `transform_tolerance` | 0.1 | TF 查询容忍时间（秒） |
| `lookahead_dist` | 0.3 | 固定前视距离（m），`use_velocity_scaled_lookahead_dist=false` 时生效 |
| `use_velocity_scaled_lookahead_dist` | `true` | 按速度缩放前视距离 |
| `lookahead_time` | 1.0 | 速度缩放系数（s），前视距离 = speed × time |
| `min_lookahead_dist` | 0.2 | 速度缩放模式下的最小前视距离（m） |
| `max_lookahead_dist` | 1.0 | 速度缩放模式下的最大前视距离（m） |
| `use_interpolation` | `true` | 在路径段间插值选取精确 lookahead 点 |
| `use_rotate_to_heading` | `true` | 目标朝向偏差超阈值时原地旋转对齐再前进 |
| `use_rotate_to_heading_treshold` | 0.1 | 触发原地旋转的角度阈值（rad） |
| `v_linear_min` | -3.0 | 线速度下限（m/s），负值允许倒退 |
| `v_linear_max` | 3.0 | 线速度上限（m/s） |
| `v_angular_min` | -3.0 | 角速度下限（rad/s） |
| `v_angular_max` | 3.0 | 角速度上限（rad/s） |
| `min_approach_linear_velocity` | 0.05 | 接近目标时的最小线速度（m/s） |
| `approach_velocity_scaling_dist` | 0.6 | 开始接近减速的剩余弧长（m） |
| `curvature_min` | 0.4 | 曲率低于此值不降速 |
| `curvature_max` | 0.7 | 曲率高于此值按最大降速比例降速 |
| `reduction_ratio_at_high_curvature` | 0.5 | 高曲率时线速度保留比例（0.5 = 降 50%） |
| `curvature_forward_dist` | 0.7 | 曲率计算前向采样距离（m） |
| `curvature_backward_dist` | 0.3 | 曲率计算后向采样距离（m） |
| `max_velocity_scaling_factor_rate` | 0.9 | 曲率限速的速度变化率限幅（平滑过渡） |
| `max_robot_pose_search_dist` | costmap 半径 | 路径上机器人最近点的最大搜索距离 |

## 发布话题（调试）

| 话题 | 类型 | 说明 |
|------|------|------|
| `local_plan` | `nav_msgs/msg/Path` | 当前帧局部裁剪后的路径 |
| `lookahead_point` | `geometry_msgs/msg/PointStamped` | 当前 lookahead 点 |
| `curvature_points_marker_array` | `visualization_msgs/msg/MarkerArray` | 曲率计算采样点（10 Hz）

## Configuration

| Parameter | Description |
|-----|----|
| `translation_kp` | The proportional gain for the translation PID controller. Controls how strongly the robot reacts to translation errors. |
| `translation_ki` | The integral gain for the translation PID controller. Helps eliminate steady-state errors over time. |
| `translation_kd` | The derivative gain for the translation PID controller. Damps oscillations by reacting to the rate of change of translation errors. |
| `enable_rotation` | Whether to enable the rotation PID controller. If disabled, the robot will not rotate to face the path's direction. `twist.angular.z` always remains zero. |
| `rotation_kp` | The proportional gain for the rotation PID controller. Controls how strongly the robot reacts to rotational errors. |
| `rotation_ki` | The integral gain for the rotation PID controller. Helps eliminate steady-state rotational errors. |
| `rotation_kd` | The derivative gain for the rotation PID controller. Damps oscillations by reacting to the rate of change of rotational errors. |
| `transform_tolerance` | The tolerance for transforming between frames. A higher value may allow for more flexibility in handling small delays in the transformation. |
| `min_max_sum_error` | The minimum threshold for the maximum sum of errors used to limit the accumulated error in the PID controller to prevent windup. |
| `lookahead_dist` | The fixed lookahead distance used to find the lookahead point for path following. |
| `use_velocity_scaled_lookahead_dist` | Whether to scale the lookahead distance based on the current velocity, instead of using a constant distance. |
| `min_lookahead_dist` | The minimum allowable lookahead distance when using velocity scaling for the lookahead point. |
| `max_lookahead_dist` | The maximum allowable lookahead distance when using velocity scaling for the lookahead point. |
| `lookahead_time` | The time used to project the robot's velocity to calculate the velocity-scaled lookahead distance. |
| `use_interpolation` | Enables interpolation between poses along the path when selecting the lookahead point, improving smoothness but potentially increasing computational cost. |
| `use_rotate_to_heading` | Whether to rotate the robot to face the path's direction before moving forward, useful in holonomic robots. |
| `use_rotate_to_heading_treshold` | The angular threshold at which the robot should rotate in place to align with the desired heading. |
| `min_approach_linear_velocity` | The minimum linear velocity when approaching the goal to ensure the robot moves slowly when close to its target. |
| `approach_velocity_scaling_dist` | The distance from the goal where velocity scaling starts when approaching, slowing the robot down as it nears the target. |
| `v_linear_min` | The minimum translation speed the robot can command, allowing for reverse or slow-forward movement. |
| `v_linear_max` | The maximum translation speed the robot can command, setting the upper limit for forward movement. |
| `v_angular_min` | The minimum rotation speed the robot can command, allowing for counter-clockwise rotation. |
| `v_angular_max` | The maximum rotation speed the robot can command, setting the upper limit for clockwise rotation. |
| `curvature_min` | The minimum curvature threshold below which no speed reduction is applied. |
| `curvature_max` | The maximum curvature threshold above which significant speed reduction is applied. |
| `reduction_ratio_at_high_curvature` | The speed reduction ratio at high curvature. 0.5 means a 50% reduction. |
| `curvature_forward_dist` | The forward distance used for curvature calculation. |
| `curvature_backward_dist` | The backward distance used for curvature calculation. |
| `max_velocity_scaling_factor_rate` | The maximum rate of change for the velocity scaling factor. |
| `max_robot_pose_search_dist` | The maximum distance along the path to search for the robot's closest pose, used to keep the robot on the planned path. |

Example fully-described XML with default parameter values:

```yaml
controller_server:
  ros__parameters:
    odom_topic: odometry
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    FollowPath:
      plugin: "omni_pid_pursuit_controller::OmniPidPursuitController"
      translation_kp: 3.0
      translation_ki: 0.1
      translation_kd: 0.3
      enable_rotation: true
      rotation_kp: 3.0
      rotation_ki: 0.1
      rotation_kd: 0.3
      transform_tolerance: 0.1
      min_max_sum_error: 1.0
      lookahead_dist: 2.0
      use_velocity_scaled_lookahead_dist: true
      lookahead_time: 1.0
      min_lookahead_dist: 0.5
      max_lookahead_dist: 1.0
      use_interpolation: false
      use_rotate_to_heading: false
      use_rotate_to_heading_treshold: 0.1
      min_approach_linear_velocity: 0.5
      approach_velocity_scaling_dist: 1.0
      v_linear_min: -2.5
      v_linear_max: 2.5
      v_angular_min: -3.0
      v_angular_max: 3.0
      curvature_min: 0.4
      curvature_max: 0.7
      reduction_ratio_at_high_curvature: 0.5
      curvature_forward_dist: 0.7
      curvature_backward_dist: 0.3
      max_velocity_scaling_factor_rate: 0.9
```
