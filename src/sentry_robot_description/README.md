# sentry_robot_description

哨兵机器人 SDF/XMacro 描述包，适配 RoboMaster 2026 全向麦轮主线。描述文件由 [xmacro](https://github.com/gezp/xmacro) 格式编写，通过 Python API 在 launch 文件中动态生成 SDF / URDF，供 `robot_state_publisher` 使用。

## 文件结构

```
sentry_robot_description/
├── resource/
│   ├── models/                    # 引用的外部模型（mid360/camera/rplidar）
│   └── xmacro/
│       ├── sentry_robot.sdf.xmacro       # 实车模型（当前活跃）
│       └── infantry_robot.sdf.xmacro     # 步兵模型
├── launch/
│   └── robot_description_launch.py
├── params/robot_description.yaml
└── rviz/visualize_robot.rviz
```

## 模型说明

### sentry_robot.sdf.xmacro（实车，当前活跃）

基于 `rm25_example_robot` 底盘，搭载：

| 传感器 / 组件 | 挂载点 | 参数 |
|---|---|---|
| Livox Mid360 (`front_` 前缀) | `gimbal_yaw` | 50 Hz，1875 samples，安装偏移 `-0.15 0 0.15`，朝向 `π`（向后） |
| 工业相机 `industrial_camera` | `gimbal_pitch` | 30 Hz，1920×1080，FOV 1 rad |

SDF 插件定义（供参考）：

| 插件 | 说明 |
|---|---|
| `MecanumDrive2` | 全向麦轮底盘驱动（SDF 内定义的插件名） |
| `JointController` × 2 | gimbal_yaw（P=0.2，I=0.01）和 gimbal_pitch（P=1，I=0.01）速度控制 |
| `JointStatePublisher` | 关节状态发布 |
| `LightBarController` | 装甲板灯条颜色控制 |
| `ProjectileShooter` | 17mm 弹丸射击（初速 20 m/s） |

## URDF / SDF 生成

### 在 launch 文件中生成（推荐）

```python
from xmacro.xmacro4sdf import XMLMacro4sdf
from sdformat_tools.urdf_generator import UrdfGenerator

xmacro = XMLMacro4sdf()
xmacro.set_xml_file(robot_xmacro_path)

# 生成 SDF
xmacro.generate()
robot_sdf = xmacro.to_string()

# 从 SDF 转 URDF（供 robot_state_publisher 使用）
urdf_generator = UrdfGenerator()
urdf_generator.parse_from_sdf_string(robot_sdf)
robot_urdf = urdf_generator.to_string()
```

### 命令行

```bash
source install/setup.bash
xmacro4sdf src/sentry_robot_description/resource/xmacro/sentry_robot.sdf.xmacro > /tmp/sentry_robot.sdf
```

### 在 RViz 中可视化

```bash
ros2 launch sentry_robot_description robot_description_launch.py
```

## Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot_name` | `sentry_robot` | XMacro 文件名（无后缀），从 `resource/xmacro/` 加载 |
| `robot_xmacro_file` | — | XMacro 文件绝对路径（优先级高于 `robot_name`） |
| `use_sim_time` | `False` | 是否使用仿真时间（实车默认 False） |
| `use_rviz` | `True` | 是否启动 RViz |
| `params_file` | `params/robot_description.yaml` | 参数文件路径 |

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `robot_description` | `std_msgs/String` | 机器人描述（字符串） |
| `joint_states` | `sensor_msgs/JointState` | 关节状态 |
| `tf` / `tf_static` | `tf2_msgs/TFMessage` | 关节坐标系 TF |

## 环境依赖

```bash
pip install xmacro
sudo apt install ros-jazzy-sdformat-tools  # 或 pip install sdformat_tools
```

