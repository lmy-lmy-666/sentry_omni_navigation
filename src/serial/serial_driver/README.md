# rm_serial_driver

ROS2 串口驱动包，连接导航上位机与 STM32 电控端。通过 USB CDC / UART 双向传输：将导航速度指令封 Control 包下发至底盘，并实时接收云台姿态、底盘状态与裁判系统数据。以 Composable Node 形式运行，支持进程内通信。

## 目录结构

```
serial/serial_driver/
├── config/serial_driver.yaml        # 串口参数配置
├── example/                         # STM32 电控端示例代码
│   ├── navigation_auto.h
│   ├── navigation_auto.c
│   └── README.md
├── include/rm_serial_driver/
│   ├── packet.hpp                   # 收发包结构体（由 generate.py 生成）
│   ├── rm_serial_driver.hpp         # 节点类声明
│   └── crc.hpp                      # CRC16 校验
├── launch/serial_driver.launch.py   # 启动文件
├── protocol/
│   ├── protocol.yaml                # 协议唯一真相源
│   ├── generate.py                  # 代码生成器
│   ├── templates/                   # Jinja2 模板
│   │   ├── packet.hpp.j2
│   │   ├── navigation_auto.h.j2
│   │   └── protocol_py.j2
│   └── generated/                   # 生成产物
├── src/
│   ├── rm_serial_driver.cpp         # 节点实现
│   └── crc.cpp                      # CRC16 实现
├── CMakeLists.txt
└── package.xml
```

## 协议 v3.1

### 帧格式
`[HEADER 1B][LEN 1B][PAYLOAD N B][CRC16 2B]`，总长 `N+4`
- CRC16 覆盖 `[HEADER..PAYLOAD]`，小端序追加。
- 结构体 `__attribute__((packed))`，无填充。

### 数据包

| 帧头 | 包名 | 方向 | 总帧大小 | 频率 | 说明 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0x50` | Telemetry | STM32 → ROS | 55B（payload 51B） | 200 Hz | IMU 姿态 + 底盘状态 + 裁判系统全量 |
| `0xA0` | Control | ROS → STM32 | 20B（payload 16B） | 20 Hz + 1 Hz 心跳 | lx/ly/az/mode + ros_state（刷新看门狗） |

#### Telemetry 字段（STM32 → ROS，200 Hz）

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| gimbal_pitch / gimbal_yaw | float | rad | 云台 pitch / yaw |
| chassis_pitch / chassis_yaw | float | rad | 车体 pitch / yaw |
| mcu_timestamp_ms | uint16 | ms | MCU 时间戳低 16 位 |
| current_hp | uint16 | hp | 本机当前血量 |
| projectile_allowance_17mm | uint16 | count | 17mm 弹丸剩余发射次数 |
| chassis_power | float | W | 底盘实时功率 |
| chassis_mode | uint8 | — | 0=normal 1=spin_low 2=spin_high 3=estop |
| game_progress | uint8 | — | 0=未开始 1=准备 2=自检 3=5s倒计时 4=比赛中 5=结算 |
| stage_remain_time | uint16 | s | 当前阶段剩余时间 |
| team_colour | uint8 | — | 1=红方 0=蓝方 |
| rfid_base | uint8 | — | 己方基地增益点 RFID（1=触发） |
| ally_1/2/3/4/7_robot_hp | uint16 | hp | 己方各机器人血量 |
| ally_outpost_hp / ally_base_hp | uint16 | hp | 己方前哨站 / 基地血量 |
| event_data | uint32 | — | 事件 bitfield（1=增益点激活，2=堡垒被占） |

#### Control 字段（ROS → STM32，20 Hz）

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| lx | float | m/s | body 系前向线速度 |
| ly | float | m/s | body 系侧向线速度 |
| az | float | rad/s | 底盘角速度 / 自旋速度 |
| mode | uint8 | — | 0=normal 1=spin_low 2=spin_high 3=estop |
| ros_state | uint8 | — | 0=init 1=ready 2=running 3=fault |

## ROS 接口

| 话题 | 类型 | 方向 | 说明 |
| :--- | :--- | :--- | :--- |
| `/cmd_vel_chassis` | geometry_msgs/msg/Twist | Subscription | 速度指令，封装为 Control 包发送 |
| `motion_manager/state` | std_msgs/msg/UInt8 | Subscription | 模式 0–3，写入 Control.mode |
| `serial/gimbal_joint_state` | sensor_msgs/msg/JointState | Publication | 云台 pitch/yaw |
| `serial/chassis_attitude` | sensor_msgs/msg/JointState | Publication | 车体 pitch/yaw |
| `serial/chassis_status` | std_msgs/msg/Float32MultiArray | Publication | [chassis_power, chassis_mode] |
| `referee/game_status` | rm_interfaces/msg/GameStatus | Publication | 比赛阶段、剩余时间 |
| `referee/robot_status` | rm_interfaces/msg/RobotStatus | Publication | 血量、弹量 |
| `referee/all_robot_hp` | rm_interfaces/msg/GameRobotHP | Publication | 己方单位 HP |
| `referee/rfidStatus` | rm_interfaces/msg/RfidStatus | Publication | RFID 基地增益 |

## 参数

参数定义于 `config/serial_driver.yaml`：

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| device_name | `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3056357F3034-if00` | 本机导航 MCU 的稳定路径，换板时需重新确认 |
| baud_rate | 115200 | 波特率 |
| flow_control | none | 流控模式 |
| parity | none | 校验位 |
| stop_bits | "1" | 停止位 |
| enable_vel_log | false | 启用速度下发 CSV 日志（详见第 11 节） |

## 编译与启动

### 编译
```bash
colcon build --packages-select rm_serial_driver --symlink-install
```

### 启动
```bash
ros2 launch rm_serial_driver serial_driver.launch.py
```

### 带参数覆盖启动
```bash
ros2 launch rm_serial_driver serial_driver.launch.py device_name:=/dev/ttyUSB0
```
通常由 `rm_navigation_reality_launch.py` 作为 Composable Node 拉起，无需单独启动。

## 协议修改流程

1. 修改 `protocol/protocol.yaml`（协议唯一真相源）。
2. 执行生成脚本：
   ```bash
   cd protocol && python3 generate.py
   ```
3. 手动同步生成的代码文件：
   - `generated/packet.hpp` → `include/rm_serial_driver/packet.hpp`
   - `generated/navigation_auto.h` → `example/navigation_auto.h`
   - `generated/protocol.py` → `../../../sentry_tools/protocol.py`
4. 重新编译 ROS 包：
   ```bash
   colcon build --packages-select rm_serial_driver
   ```
5. 使用新的 `navigation_auto.h` 和 `navigation_auto.c` 更新 STM32 电控端代码。

注意：`sentry_tools` 工具箱的 GUI 控件会根据 `protocol.py` 自动更新。

## 端口定位与接收验证

本机排查结果：序列号 `2061347D534D` 为 `/dev/gimbal`（当时为 ACM0），由自瞄使用；
序列号 `3056357F3034`（当时为 ACM1）持续发送 CRC 正确的 `0x50`、55 字节导航遥测帧。
默认配置已绑定后者的 by-id 路径，不能再根据 ACM 编号猜测设备用途。换板后用
`udevadm info --query=property --name=/dev/ttyACM1` 核对序列号并修改配置。

启动日志会输出设备路径。`TX locally written` 和速度 CSV 仅表示整帧已写入本地传输层，
不代表电控已解析；当前协议没有 ACK。零字节写入会报错，短写会补齐剩余字节。
电控侧应检查 `g_navigation.valid` 和 `g_navigation.last_recv_ms` 是否随 Control 更新，
并确认 `CDC_Receive_FS` 调用了 `Navigation_OnUsbReceive(Buf, *Len)` 后重新挂接 USB 接收。

电控解析器修复位于模板及 `example/navigation_auto.c`：未知帧头、错误长度和 CRC 失败
均逐字节重新同步，保留半帧，支持粘包及大块输入。必须将更新后的示例合入实际电控工程、
保留项目的数据获取适配并重新烧录，才能让板上固件获得该修复。

离线回归（不会访问硬件）：

```bash
colcon test --packages-select rm_serial_driver --ctest-args -R '^protocol_transport_test$' --output-on-failure
colcon test-result --verbose
```

## 串口调试

- **虚拟串口测试**：使用 socat 创建虚拟串口对进行闭环测试：
  ```bash
  socat -d -d pty,raw,echo=0 pty,raw,echo=0
  ```
- **模拟电控**：使用 `sentry_toolbox.py` 的 "串口 Mock" 选项卡模拟 STM32 发送数据。
- **链路诊断**：使用 `sentry_toolbox.py` 的 "串口诊断" 选项卡监控实时通信质量与丢包率。
- **常见问题**：若提示权限不足，请执行 `sudo chmod 666 /dev/ttyACM0` 或将当前用户加入 `dialout` 用户组。

## 电控端集成

电控端集成代码位于 `example/` 目录，提供了基于 FreeRTOS 和 USB CDC 的实现示例。
- 详细集成说明请参考 `example/README.md`。
- 关键适配点包括：裁判系统结构体命名对接、通信接口切换（USB CDC vs UART）以及发送频率宏定义。

## 注意事项

- ROS 端与电控端的结构体必须保持严格的字节对齐，协议变更时两端必须同步更新。
- CRC16 校验算法两端必须一致（采用查表法，初始值为 0xFFFF）。
- `/cmd_vel_chassis` 是绝对路径话题，接收的是经 `fake_vel_transform` 转换后的 body 系速度（含自旋叠加）。
- 驱动具备自动重连机制，串口断开后会以 1s 为间隔尝试重新打开设备。
- 本包特有的 CMake 配置将 C++ 标准设为 C++14（项目其他包通常使用 C++17）。
- `package.xml` 中保留了部分历史遗留的未使用依赖（如 `auto_nav_interfaces`, `visualization_msgs` 等）。

## 速度日志

用于调试速度毛刺、突变等问题。启用后在整帧成功写入本地串口后记录每帧 `vel_x`, `vel_y`, `vel_w` 到 CSV 文件，便于离线波形分析。

### 启用方式

在 `nav2_params.yaml` 中：

```yaml
rm_serial_driver:
  ros__parameters:
    enable_vel_log: true
```

或 launch 参数覆盖：

```bash
ros2 launch rm_serial_driver serial_driver.launch.py enable_vel_log:=true
```

### 日志位置

`/tmp/vel_log_<启动时间戳>.csv`，节点启动时打印完整路径。

### CSV 格式

```csv
timestamp_ns,lx,ly,az,mode
1712345678000000000,1.23,-0.45,3.14,0
```

- `timestamp_ns`：ROS 时间（纳秒）
- `lx`, `ly`：body 系线速度（m/s）
- `az`：角速度（rad/s）
- `mode`：0=normal 1=spin_low 2=spin_high 3=estop

### 离线分析

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('/tmp/vel_log_xxx.csv')
df['t'] = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) / 1e9

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 6))
for i, col in enumerate(['lx', 'ly', 'az']):
    axes[i].plot(df['t'], df[col], linewidth=0.5)
    axes[i].set_ylabel(col)
    axes[i].grid(True, alpha=0.3)
axes[2].set_xlabel('time (s)')
plt.tight_layout()
plt.savefig('vel_debug.png', dpi=150)
plt.show()
```

### 性能影响

默认关闭（`enable_vel_log: false`），零开销。启用后 50Hz 写入约 2KB/s，对实时性无影响。调试完毕后建议关闭。
