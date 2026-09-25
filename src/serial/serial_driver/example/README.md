# 电控集成指南（v3.1 协议）

`navigation_auto.h` / `navigation_auto.c` 是哨兵 ROS 端 `rm_serial_driver` 与 STM32 电控之间的串口协议参考实现。两边各编译各的，**协议**由 `protocol/protocol.yaml` 单源定义，跑 `python3 generate.py` 同时刷新两端代码。

---

## 1. 概述

### 协议特点

- **统一帧格式**：`[HEADER 1B][LEN 1B][PAYLOAD N B][CRC16 2B]`，总长 `N+4`
- **单一大包**：上行 `0x50 Telemetry` 55B@200Hz；下行 `0xA0 Control` 20B@20Hz+1Hz
- **看门狗**：任意 Control 帧 CRC 正确即刷新；1500ms 无下行则急停
- **mode 字段**：Control 带 `mode`，电控按 `normal / spin_low / spin_high / estop` 4 套 PID

### 版本差异

| 项 | v2.x | v3.0 | v3.1（当前） |
|---|---|---|---|
| 帧头标识 | `0x5B / 0xB5` | `0x5x` / `0xAx` 段位 | `0x50` / `0xA0` |
| LEN 字节 | 无 | 有 | 有 |
| 上行包数 | 1 个（38B） | 3 个分级 | **1 个（55B）** |
| 下行包数 | 1 个（15B） | 2 个 | **1 个（20B）** |
| 心跳 | 无 | 独立 0xA3 | 合入 Control |

### 提供的两个文件

| 文件 | 说明 |
|---|---|
| `navigation_auto.h` | 协议结构 + Mode 枚举 + 公共 API + 全局变量声明 |
| `navigation_auto.c` | CRC16 表实现 + 流式解析 + 1kHz 周期分发器 + 看门狗 |

⚠️ **两个文件都是 `protocol.yaml` 自动生成**。手改会被下一次 `generate.py` 覆盖。任何字段 / 包结构改动应改 yaml 再重新生成。

---

## 2. 5 步快速集成

### 步骤 1：拷贝文件

```bash
# 从 ROS 仓库复制到 STM32 工程
cp navigation_auto.h /your_stm32_project/USER/Navigation/
cp navigation_auto.c /your_stm32_project/USER/Navigation/
```

### 步骤 2：CubeMX 配置 USB CDC

- 启用 `USB_DEVICE` → `Communication Device Class (Virtual Port Com)`
- 默认 64-byte 端点足够用，**不必加大 buffer**
- `usbd_cdc_if.h` 自动生成 `CDC_Transmit_FS` / `CDC_Receive_FS`

### 步骤 3：在 USB CDC 接收回调里插入解析入口

打开 `Src/usbd_cdc_if.c`，找到 `CDC_Receive_FS`：

```c
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  // ↓↓↓ 加这一行 ↓↓↓
  Navigation_OnUsbReceive(Buf, *Len);
  // ↑↑↑ 加这一行 ↑↑↑
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}
```

`Navigation_OnUsbReceive()` 是流式解析器，可处理多帧、半帧及大块输入。仅接受已定义的帧头和对应长度；CRC 错误时逐字节重新同步，不整段丢弃可能包含正常帧的数据。更新解析器后需重新编译并烧录电控固件。

### 步骤 4：创建 1kHz FreeRTOS 任务

在 `freertos.c` 或自己的任务文件加：

```c
#include "navigation_auto.h"

osThreadDef(navTask, NavTask_Entry, osPriorityAboveNormal, 0, 256);
osThreadCreate(osThread(navTask), NULL);

void NavTask_Entry(void const *argument)
{
  Navigation_Init();              // 内部清零所有静态结构 + 设置帧头
  for (;;) {
    Navigation_Task();            // 周期分发器：看门狗 + 200Hz Telemetry 上行
    osDelay(1);                   // 1ms tick
  }
}
```

`Navigation_Task()` 自身不阻塞、不申请内存，开销极小（< 1% CPU）。

### 步骤 5：业务任务读速度指令

在你的运动控制任务里：

```c
#include "navigation_auto.h"

void ChassisCtrl_Task(void const *argument)
{
  for (;;) {
    /* 1. 链路存活检查（可选，已在 Navigation_Task 自动 estop） */
    if (!Navigation_IsLinkAlive()) {
      // 心跳超时已在 Navigation_Task 内部把 g_navigation.* 清零并设为 ESTOP
      // 这里可加额外报警 / LED
    }

    /* 2. 取速度指令 */
    float vx = g_navigation.lx;
    float vy = g_navigation.ly;
    float wz = g_navigation.az;
    uint8_t mode = g_navigation.mode;

    /* 3. 按 mode 切 PID 配置 */
    switch (mode) {
      case CHASSIS_MODE_NORMAL:    Chassis_RunNormalPID(vx, vy, wz); break;
      case CHASSIS_MODE_SPIN_LOW:  Chassis_RunSpinPID(vx, vy, /*lock_az=*/1.5f); break;
      case CHASSIS_MODE_SPIN_HIGH: Chassis_RunSpinPID(vx, vy, /*lock_az=*/3.0f); break;
      case CHASSIS_MODE_ESTOP:     Chassis_ForceStop(); break;
    }

    osDelay(2);  // 500Hz 控制频率
  }
}
```

至此完成。机器人应该能收到 ROS 下发的速度指令并按模式执行。

---

## 3. mode 字段语义详解

下行 `RecvControlPacket.mode` 是 1 字节枚举：

| mode | 名称 | 触发场景 | 电控应做 |
|---|---|---|---|
| 0 | `CHASSIS_MODE_NORMAL` | 正常导航点对点 | 用导航 PID，**精确跟随** `lx/ly/az` |
| 1 | `CHASSIS_MODE_SPIN_LOW` | 巡逻 / 侦查 / 省功 | 自旋 PID，固定 ~1.5 rad/s（也可用电控自定义值） |
| 2 | `CHASSIS_MODE_SPIN_HIGH` | 受击 / 规避 | 自旋 PID，固定 ~3.0 rad/s（保命模式） |
| 3 | `CHASSIS_MODE_ESTOP` | 系统异常 / 决策报错 | **强制归零**所有速度，**忽略 lx/ly/az** |

**关键**：电控**必须**实现 4 套 PID 配置或参数集，`mode` 是切换标签。
- `NORMAL` 追求角度精度无超调
- `SPIN` 追求转速恒定 + 功率稳定
- `ESTOP` 立即归零，不走 PID 平滑

ESTOP 是优先级**最高**的安全锁——即使后续因为某种原因收到了带速度的脏数据，只要 mode=3 就忽略。

---

## 4. 心跳看门狗机制

### 工作原理

| 时机 | 行为 |
|---|---|
| MCU 收到 `0xA0 RecvControlPacket` 且 CRC OK | `g_heartbeat.last_recv_ms = now_ms()` |
| 每次 `Navigation_Task()` tick | 检查 `now_ms() - g_heartbeat.last_recv_ms < HEARTBEAT_TIMEOUT_MS` |
| 超过 1500ms 没心跳 | 自动把 `g_navigation.{lx,ly,az}=0`、`mode=CHASSIS_MODE_ESTOP` |

### 用户可查询

```c
if (!Navigation_IsLinkAlive()) {
  // 链路异常：可点亮 LED / 发蜂鸣 / 切到本地手柄控制
}
```

### 超时阈值修改

`navigation_auto.h` 顶部：

```c
#define HEARTBEAT_TIMEOUT_MS 1500    /* 默认 1.5s */
```

设太短会被偶发抖动误触发；设太长 ROS 崩溃后机器人会失控太久。**1500ms 是经验值**，ROS 端 1Hz 心跳丢失一包时，间隔可能达到 2s，超过此阈值会触发急停。

---

## 5. 数据获取钩子（TODO 接线）

`send_TelemetryPacket()` 每 5ms 组一包，IMU 每帧更新，裁判/底盘字段建议 **1Hz 读入缓存、200Hz 只 memcpy**：

```c
#ifdef INS_H
  Smsg_telemetry.gimbal_pitch  = -INS.Pitch / 57.3f;
  Smsg_telemetry.gimbal_yaw    =  INS.Yaw   / 57.3f;
  Smsg_telemetry.chassis_pitch = -INS.Pitch / 57.3f;  /* TODO: 车体 IMU */
  Smsg_telemetry.chassis_yaw   =  INS.Yaw   / 57.3f;
#endif
  Smsg_telemetry.mcu_timestamp_ms = (uint16_t)(now_ms() & 0xFFFF);
#ifdef REFEREE_H
  Smsg_telemetry.current_hp                = Robot_Status.current_HP;
  Smsg_telemetry.projectile_allowance_17mm = Projectile_Allowance.projectile_allowance_17mm;
  Smsg_telemetry.game_progress             = Game_Status.game_progress;
  /* ... ally_*_hp, event_data 等同理 ... */
#endif
#ifdef MODE_CONTROL_H
  Smsg_telemetry.chassis_power = ModeControl_GetChassisPower();
  Smsg_telemetry.chassis_mode  = (uint8_t)ModeControl_GetChassisMode();
#endif
```

---

## 6. CRC16 配置

### 默认实现

`navigation_auto.c` 内嵌完整 256 项 CRC16 查表实现：
- 多项式：`0x1189`
- 初始值：`0xFFFF`
- 字节序：小端追加（`buf[len-2] = lo, buf[len-1] = hi`）

直接编译即可工作。

### 与项目自有 CRC 实现复用

如果项目已有 `CRCs.h` 提供 `Append_CRC16_Check_Sum` / `CRC16_Verify`：

```c
// 编译选项加上 -DCRC16_PROVIDED_BY_PROJECT
// 然后在 navigation_auto.c 顶部 include 你自己的头文件
#include "CRCs.h"
```

`navigation_auto.c` 看到这个宏会跳过内嵌实现。**但你必须保证两端 CRC 算法**（多项式 / 初值 / 字节序）**一模一样**，否则 ROS 端会一直报 `CRC failed`。

---

## 7. 调试建议

### 7.1 ROS 端模拟（电控未就绪时）

`sentry_tools/sentry_toolbox.py` 的"串口 Mock"选项卡可以模拟电控持续发上行帧：

```bash
python3 src/sentry_tools/sentry_toolbox.py
# 选 "串口 Mock"，配置虚拟串口或真实 ttyUSB
```

### 7.2 实时可视化解析结果

`sentry_tools/serial_visualizer.py` 订阅 ROS 话题实时显示（与 Telemetry 字段对应）：

```bash
python3 src/sentry_tools/serial_visualizer.py
```

### 7.3 链路诊断

```bash
# 看 ROS 端日志（节点终端）
[WARN] CRC failed on header 0x50        # CRC 算法不一致
[WARN] LEN mismatch on 0x50: got N want 51  # 包结构对齐错误
[WARN] Unknown header 0x37, dropping byte   # 杂讯字节，单字节丢弃后会自动恢复

# 看话题（均由 Telemetry 大包拆分发布）
ros2 topic hz /serial/gimbal_joint_state    # 应稳定 ~200Hz
ros2 topic hz /referee/robot_status         # 应稳定 ~200Hz（字段来自 MCU 缓存）
ros2 topic hz /referee/game_status          # 应稳定 ~200Hz

# 静止 2s 后不应误 estop（ROS 1Hz 补发 Control）
```

### 7.4 心跳排查

```c
// 在 MCU 侧加调试输出
printf("hb valid=%d, last=%lu, now=%lu, alive=%d\n",
       g_heartbeat.valid,
       g_heartbeat.last_recv_ms,
       now_ms(),
       Navigation_IsLinkAlive());
```

---

## 8. 协议变更流程（强制单源）

任何字段 / 包结构修改都要走单源链路，**禁止手改 navigation_auto.{h,c}**：

```
                                     ┌─→ packet.hpp        (ROS 端)
                                     │
protocol.yaml ── generate.py ────────┼─→ navigation_auto.h (电控端)
   (single source)                   │
                                     ├─→ navigation_auto.c (电控端)
                                     │
                                     └─→ protocol.py       (Python 工具)
```

操作步骤：

```bash
# 1. 改协议
vim src/serial/serial_driver/protocol/protocol.yaml

# 2. 重新生成
cd src/serial/serial_driver/protocol
python3 generate.py

# 3. 复制到消费侧（generate.py 不自动覆盖）
cp generated/packet.hpp        ../include/rm_serial_driver/packet.hpp
cp generated/navigation_auto.h ../example/navigation_auto.h
cp generated/navigation_auto.c ../example/navigation_auto.c
cp generated/protocol.py       ../../../sentry_tools/protocol.py

# 4. 重编 ROS 侧
cd /path/to/ws
colcon build --packages-select rm_serial_driver

# 5. 同步给电控同学，重编固件
```

---

## 9. 性能与带宽

### 默认带宽预算（115200 baud）

| 包 | 帧长 | 频率 | 带宽 |
|---|---|---|---|
| Telemetry | 55 B | 200 Hz | 11000 B/s |
| Control | 20 B | ~21 Hz | ~420 B/s |
| **合计** | | | **~11420 B/s（≈99%）** |

115200 baud 可用约 11520 B/s；余量紧时可升 **460800**（两端同改 `baud_rate`）。

### MCU CPU 开销

`Navigation_Task()` 1kHz：每 5 ticks 一次 55B CRC + USB 发送，整体仍 < 1% CPU。瓶颈多为 USB CDC 端点拥塞。

---

## 10. 故障排查表

| 现象 | 可能原因 | 排查 |
|---|---|---|
| ROS 端持续 `CRC failed on 0x5x` | CRC 算法不匹配 | 多项式 0x1189 / 初值 0xFFFF / 小端追加，三者必须一致 |
| ROS 端 `LEN mismatch on 0x5x` | 包结构对齐错误 | 检查 `__attribute__((packed))` 是否生效（GCC/Keil 都支持） |
| ROS 端 `Unknown header 0xXX` 偶发 | USB 噪声 / 偶发字节漂移 | 正常现象，dispatcher 单字节丢弃自动恢复 |
| ROS 端 `Unknown header` 持续 | 帧头未对齐到协议规范 | 检查 `Smsg_*.header` 是否正确赋值（`Navigation_Init()` 已自动赋值） |
| 机器人随机停止 / 抽搐 | 心跳超时 → estop | `Navigation_IsLinkAlive()` 看链路状态；看 ROS 端节点是否还活着 |
| 200Hz IMU 频率上不去（实测 < 100Hz） | USB CDC 端点拥塞 / FreeRTOS 优先级低 | 提升 nav 任务优先级到 `osPriorityAboveNormal` 以上；或升 baud 到 460800 |
| 自旋速度不准 / 角度跟踪不稳 | mode 切换没生效 | 看 `chassis_mode` 反馈是否等于下行的 mode；电控 4 套 PID 是否真的切换 |
| ROS 端节点起不来报 `Error creating serial port` | `/dev/ttyACM0` 不存在或权限问题 | `ls -l /dev/ttyACM0`；`sudo chmod 666 /dev/ttyACM0` 或加入 `dialout` 用户组 |

---

## 附录 A：完整 API 速查

```c
/* 生命周期 */
void Navigation_Init(void);                                    /* 拷贝文件后第一次调 */
void Navigation_Task(void);                                    /* 1kHz 周期调 */
void Navigation_OnUsbReceive(const uint8_t *data, uint32_t len);  /* CDC 收到字节就调 */
bool Navigation_IsLinkAlive(void);                             /* 心跳活着？ */

/* 主动发送（Navigation_Task 内部 200Hz 自动调） */
void send_TelemetryPacket(void);

/* CRC 工具（已内嵌实现，业务无需调用） */
void Append_CRC16_Check_Sum_Buf(uint8_t *buf, uint32_t len);
bool Verify_CRC16_Check_Sum_Buf(const uint8_t *buf, uint32_t len);

/* 全局状态 — 业务代码读 */
extern Navigation     g_navigation;   /* lx/ly/az/mode + valid + last_recv_ms */
extern HeartbeatState g_heartbeat;    /* ros_state + valid + last_recv_ms */

/* mode 枚举 */
typedef enum {
  CHASSIS_MODE_NORMAL    = 0,
  CHASSIS_MODE_SPIN_LOW  = 1,
  CHASSIS_MODE_SPIN_HIGH = 2,
  CHASSIS_MODE_ESTOP     = 3,
} ChassisMode_e;
```
