# fake_vel_transform

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 简介

`fake_vel_transform` 解决底盘持续自旋时 Nav2 局部规划器的速度参考系漂移问题。

Nav2 以 `robot_base_frame`（`gimbal_yaw`）为速度参考系。底盘持续自旋时，`gimbal_yaw` 随底盘连续旋转，Nav2 会误判机器人朝向，导致轨迹跟踪失稳。本节点引入 `fake_robot_base_frame`（`gimbal_yaw_fake`），其 yaw 始终反向抵消 `robot_base_frame` 的实时旋转角，使 Nav2 看到的虚拟坐标系在惯性系下保持朝向稳定。

速度指令处理流程：
1. 订阅 `input_cmd_vel_topic`（`TwistStamped`，Nav2 输出，基于 `gimbal_yaw_fake` 系）
2. 从 `odom_topic` 读取当前 `robot_base_frame` 的 yaw 角
3. 将线速度从 `gimbal_yaw_fake` 系旋转到 `gimbal_yaw` 系（2D 旋转矩阵）
4. angular.z 叠加 `spin_speed`（底盘自旋速度）
5. 发布 `output_cmd_vel_topic`（`Twist`）给底盘驱动

TF 广播以 50ms 周期（20 Hz）持续更新 `robot_base_frame` → `fake_robot_base_frame` 变换，旋转角为当前 `robot_base_frame` yaw 的相反数，保证虚拟系在惯性系下朝向固定。

## 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `input_cmd_vel_topic` | `geometry_msgs/msg/TwistStamped` | Nav2 输出速度指令（`fake_robot_base_frame` 系） |
| `odom_topic` | `nav_msgs/msg/Odometry` | 里程计，用于提取当前 yaw 角，默认 `odom` |
| `cmd_spin_topic` | `example_interfaces/msg/Float32` | 动态更新底盘自旋速度，默认 `cmd_spin` |

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `output_cmd_vel_topic` | `geometry_msgs/msg/Twist` | 变换并叠加自旋后的底盘速度指令 |

## TF 广播

`robot_base_frame` → `fake_robot_base_frame`，旋转量 = −current_yaw，20 Hz 持续更新

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `robot_base_frame` | string | `gimbal_yaw` | 真实底盘/云台坐标系 |
| `fake_robot_base_frame` | string | `gimbal_yaw_fake` | 虚拟惯性参考系（Nav2 规划用） |
| `odom_topic` | string | `odom` | 里程计话题，用于读取实时 yaw |
| `cmd_spin_topic` | string | `cmd_spin` | 底盘自旋速度覆写话题 |
| `input_cmd_vel_topic` | string | `""` | 输入速度话题（需在 launch 中赋值） |
| `output_cmd_vel_topic` | string | `""` | 输出速度话题（需在 launch 中赋值） |
| `init_spin_speed` | float | `0.0` | 未收到 `cmd_spin_topic` 时的初始自旋速度（rad/s） |

## 速度变换公式

```
vx_out =  vx_in * cos(yaw) + vy_in * sin(yaw)
vy_out = -vx_in * sin(yaw) + vy_in * cos(yaw)
wz_out =  wz_in + spin_speed
```

其中 `yaw` 为当前 `robot_base_frame` 相对 `odom` 的偏航角。
