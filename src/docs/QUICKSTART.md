# 快速部署与上手指南

> 如需了解系统各模块的详细架构设计，请参阅 [系统架构详解](ARCHITECTURE.md)。

## 0. 一键配置（推荐）
如果你使用全新的 Ubuntu 24.04 系统，可以直接运行一键配置脚本完成所有环境安装和编译：
```bash
bash src/scripts/setup_env.sh
```

该脚本会依次执行以下步骤：
1. **安装 ROS2 Jazzy**: 添加官方 APT 源，安装 `ros-jazzy-desktop` 和 `ros-dev-tools`
2. **安装系统依赖**: Eigen3、OpenMP、PCL、Nav2、SLAM Toolbox、serial-driver 等
3. **编译安装 small_gicp v1.0.0**: 从 GitHub 克隆并编译（需要 C++17）
4. **初始化 rosdep**: 配置 ROS 包依赖管理
5. **创建工作空间并编译**: 在 `~/sentry_ws` 下创建工作空间并执行 `colcon build`
6. **配置 bashrc（可选）**: 询问是否将工作空间环境写入 `~/.bashrc`

如果你希望手动逐步配置，请继续阅读以下章节。

## 1. 环境要求
- Ubuntu 24.04
- ROS2 Jazzy

## 2. Docker 部署（可选）
Docker 是快速体验本项目的一种方式。

### Docker 安装
请参考 Docker 官方文档安装 Docker Engine 和 Docker Compose。

### 允许 Docker 访问显示器
在宿主机执行：
```bash
xhost +local:docker
```

### 运行容器
```bash
docker run -it --rm \
  --name sentry_nav_container \
  --net=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  sentry_nav:latest
```

> 注意：镜像名需根据实际构建的标签更新。

## 3. 源码编译部署

### 3.1 安装依赖
本项目依赖 small_gicp 进行点云配准，请先编译安装：
```bash
sudo apt install -y libeigen3-dev libomp-dev
git clone https://github.com/koide3/small_gicp.git
cd small_gicp && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

### 3.2 安装 ROS 依赖
在仓库根目录执行：
```bash
source /opt/ros/jazzy/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

### 3.3 编译

> **OOM 预防（重要）**：`btcpp_ros2_interfaces` / `rm_interfaces` 等 IDL 包 Python binding 内存占用大。全量并行在 16G 内存机器上易 OOM。推荐加 `--parallel-workers 4`，或内存极紧时用 `--executor sequential`。

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4
source install/setup.bash
```

`--symlink-install` 选项让 YAML/XML 参数文件修改后无需重新编译，直接生效。

单包编译（调试用）：
```bash
colcon build --packages-select sentry_behavior --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

如果编译出错，`src/scripts/` 下有常见问题修复脚本（`fix_nav2_deps.sh`、`fix_libusb_pcl.sh` 等）。

## 4. 实车模式

### 前置条件
- Livox MID360 激光雷达已连接并配置网络（默认 LiDAR IP: `192.168.1.150`，主机 IP: `192.168.1.50`）
- 串口设备已连接（默认 `/dev/ttyACM0`），已授权：`sudo chmod 666 /dev/ttyACM0`
- 先验点云（PCD）和 2D 地图已放置到 `sentry_nav_bringup/pcd/reality/` 和 `map/reality/`

### 4.0 一键启动（推荐）
```bash
ros2 launch sentry_nav_bringup rm_sentry_launch.py
```
该命令同时启动串口驱动、Livox 驱动、Point-LIO、导航栈，以及（可选）状态机决策和录包。

关键参数：
```bash
# 不启动录包
ros2 launch sentry_nav_bringup rm_sentry_launch.py enable_recorder:=False

# 启动状态机决策
ros2 launch sentry_nav_bringup rm_sentry_launch.py enable_behavior:=True strategy:=rmuc_defend

# 指定地图/PCD 名称（world 参数对应 map/reality/<world>.yaml 和 pcd/reality/<world>.pcd）
ros2 launch sentry_nav_bringup rm_sentry_launch.py world:=rmul_2026
```

### 4.1 建图
```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True
```

### 4.2 导航（已有先验地图）
```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py \
  world:=<YOUR_WORLD_NAME> \
  slam:=False \
  use_robot_state_pub:=True
```

> 实车 launch 的 `world` 默认值是 `204`（数字，对应实验室地图命名），需根据实际地图文件名覆盖。

## 5. 状态机决策（独立启动）
```bash
ros2 launch sentry_behavior sentry_behavior_launch.py
```

或在实车 launch 时通过 `enable_behavior:=True` 自动延迟 8 秒启动（见 5.0 节）。

## 6. 比赛录制与回放

`sentry_match_recorder` 在比赛进入 `game_progress=4` 上升沿自动启动 `ros2 bag record`，下降沿自动停止，单场所有切片归到一个目录。`rm_sentry_launch.py` 默认带录包，临时关用 `enable_recorder:=False`。

### 6.1 自动录制

```bash
# 默认带录包，比赛开始即自动录到 logs/match-bags/sortie_<TS>/
ros2 launch sentry_nav_bringup rm_sentry_launch.py
# 临时禁用录包
ros2 launch sentry_nav_bringup rm_sentry_launch.py enable_recorder:=False
```

输出目录结构（rosbag2 `--max-bag-duration` 切片）：
```
logs/match-bags/
└── sortie_20260521_143012/
    ├── metadata.yaml
    ├── sortie_20260521_143012_0.mcap   # 0~60s
    ├── sortie_20260521_143012_1.mcap   # 60~120s
    └── ...
```

### 6.2 整场连续回放

```bash
ros2 bag play logs/match-bags/sortie_20260521_143012
```

### 6.3 切片合并到单文件

```bash
ros2 run sentry_match_recorder merge_sortie logs/match-bags/sortie_20260521_143012

# 合并后删除原切片
ros2 run sentry_match_recorder merge_sortie <SORTIE_DIR> --remove-shards

# 仅打印清单，不写文件
ros2 run sentry_match_recorder merge_sortie <SORTIE_DIR> --dry-run
```

## 7. 常见问题

| 现象 | 解决 |
|------|------|
| 编译失败 | 确认 small_gicp (>= v1.0.0) 已安装；检查 `src/scripts/fix_*.sh` |
| 编译 OOM | 加 `--parallel-workers 4` 或 `--executor sequential` |
| 先验点云缺失 | PCD 文件体积较大未入仓，需自行准备并放置到正确路径 |
| 重定位后位置异常 | 确认先验 PCD 与 2D 地图使用相同坐标原点（同一次建图产生）|
