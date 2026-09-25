# rm_interfaces

项目统一 ROS2 自定义消息包，涵盖裁判系统与视觉系统两类接口。

## 消息分类

### 裁判系统消息 (`msg/referee/`)

对应裁判系统串口协议 V1.7.0（20241225）。由 `rm_serial_driver` 从 Telemetry 包（`0x50`）解析后发布。

#### GameStatus.msg

```
uint8 behavior_state
uint8 game_progress          # 0=未开始 1=准备 2=自检 3=5s倒计时 4=比赛中 5=结算
int32 stage_remain_time      # 当前阶段剩余时间（秒）
```

含常量：`NOT_START=0` / `PREPARATION=1` / `SELF_CHECKING=2` / `COUNT_DOWN=3` / `RUNNING=4` / `GAME_OVER=5`

#### RobotStatus.msg

```
uint8 robot_id / robot_level
uint16 current_hp / maximum_hp
uint16 shooter_barrel_cooling_value / shooter_barrel_heat_limit
bool add_ok                          # 血量增加标志（上位机二次处理）
uint16 shooter_17mm_1_barrel_heat    # 17mm 枪口热量
geometry_msgs/Pose robot_pos         # 本机位置
uint8 armor_id / hp_deduction_reason # 扣血装甲 ID 与原因
uint16 projectile_allowance_17mm     # 17mm 弹丸剩余发射次数
uint16 remaining_gold_coin
bool is_hp_deduced                   # 血量是否下降（上位机二次处理）
```

扣血原因常量：`ARMOR_HIT=0` / `SYSTEM_OFFLINE=1` / `OVER_SHOOT_SPEED=2` / `OVER_HEAT=3` / `OVER_POWER=4` / `ARMOR_COLLISION=5`

#### GameRobotHP.msg

```
uint16 ally_1_robot_hp   # 英雄
uint16 ally_2_robot_hp   # 工程
uint16 ally_3_robot_hp   # 步兵3
uint16 ally_4_robot_hp   # 步兵4
uint16 ally_7_robot_hp   # 哨兵
uint16 ally_outpost_hp   # 前哨站
uint16 ally_base_hp      # 基地
uint32 event_data        # 1=己方增益点激活，2=己方堡垒被占
```

#### RfidStatus.msg

25 个 `bool` 字段，涵盖己方/对方各增益点的 RFID 状态，包括：基地增益点、中央高地、梯形高地、飞坡前后、公路上下方、堡垒、前哨站、补给区、大资源岛、中心增益点（仅 RMUL）等。

### 视觉消息 (`msg/vision/`)

由视觉识别节点发布，供 `sentry_behavior` 等节点消费。

#### Armor.msg

```
string number                        # 装甲板数字标签
string type                          # 装甲板类型
float32 distance_to_image_center     # 到图像中心的距离
geometry_msgs/Pose pose              # 装甲板姿态
```

#### Armors.msg

```
std_msgs/Header header
Armor[] armors                       # 装甲板检测结果数组
```

#### Target.msg

```
std_msgs/Header header
bool tracking                        # 是否正在跟踪
string id                            # 目标 ID
int32 armors_num                     # 装甲板数量
geometry_msgs/Point position         # 目标位置
geometry_msgs/Vector3 velocity       # 目标速度
float64 yaw / v_yaw                  # 目标 yaw 角及角速度
float64 radius_1 / radius_2          # 目标旋转半径
float64 dz                           # 高度差
```

## 编译

```bash
colcon build --packages-select rm_interfaces --symlink-install
```

修改 `.msg` 后须先编译本包，再编译依赖它的包：

```bash
colcon build --packages-select rm_interfaces
colcon build --packages-select rm_serial_driver sentry_behavior
```

## 依赖

- `geometry_msgs`
- `std_msgs`

