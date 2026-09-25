/* AUTO-GENERATED from protocol.yaml — DO NOT EDIT
 * Run `python3 generate.py` after modifying protocol.yaml.
 *
 * Frame format (v3.1):
 *   [HEADER 1B] [LEN 1B] [PAYLOAD N B] [CRC16 2B]
 *   - LEN     = N (payload bytes only)
 *   - CRC16   = covers [HEADER..PAYLOAD] (N+2 bytes), little-endian appended
 *   - total   = N + 4
 *
 * Direction (from MCU perspective):
 *   FH_TX_xxx : MCU sends to ROS
 *   FH_RX_xxx : MCU receives from ROS
 *
 * STM32 integration:
 *   1) Call Navigation_Init() once.
 *   2) Inside a 1kHz FreeRTOS task call Navigation_Task() each tick.
 *      Send timing is dispatched internally.
 *   3) Implement the data-fetch hooks in navigation_auto.c marked with
 *      "TODO: connect to ...".
 *   4) The receive-side watchdog will force estop if no Heartbeat
 *      arrives within HEARTBEAT_TIMEOUT_MS.
 */

#ifndef NAVIGATION_AUTO_H
#define NAVIGATION_AUTO_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define APP_RX_DATA_SIZE   2048
#define APP_TX_DATA_SIZE   2048

/* Watchdog: if no Heartbeat for this long, MCU forces estop. */
#define HEARTBEAT_TIMEOUT_MS 1500

/* ----------------------------------------------------------------------------
 * Header constants  (MCU view: TX = MCU→ROS, RX = ROS→MCU)
 * ------------------------------------------------------------------------- */
#define HDR_TX_TELEMETRY       0x50  /* MCU→ROS, 200Hz, frame=55B */

#define HDR_RX_CONTROL         0xA0  /* ROS→MCU, 20Hz, frame=20B */


/* ----------------------------------------------------------------------------
 * Send intervals (in 1kHz task ticks)
 * ------------------------------------------------------------------------- */
#define SEND_TELEMETRY_INTERVAL_MS     5  /* 200Hz */


/* ----------------------------------------------------------------------------
 * Mode enums (kept consistent with protocol.yaml comments)
 * ------------------------------------------------------------------------- */
typedef enum {
  CHASSIS_MODE_NORMAL    = 0,
  CHASSIS_MODE_SPIN_LOW  = 1,
  CHASSIS_MODE_SPIN_HIGH = 2,
  CHASSIS_MODE_ESTOP     = 3,
} ChassisMode_e;

typedef enum {
  ROS_STATE_INIT    = 0,
  ROS_STATE_READY   = 1,
  ROS_STATE_RUNNING = 2,
  ROS_STATE_FAULT   = 3,
} RosState_e;

/* ----------------------------------------------------------------------------
 * TX packet structs (MCU → ROS)
 * ------------------------------------------------------------------------- */

/* 0x50 SendTelemetryPacket — 55B total (51B payload), 200Hz */
typedef struct
{
  uint8_t  header;
  uint8_t  len;

  float    gimbal_pitch;  /* 云台 pitch（绕 y） [rad] */
  float    gimbal_yaw;  /* 云台 yaw（绕 z） [rad] */
  float    chassis_pitch;  /* 车体 pitch（绕 y），轮足姿态 [rad] */
  float    chassis_yaw;  /* 车体 yaw（绕 z） [rad] */
  uint16_t mcu_timestamp_ms;  /* MCU 时间戳低 16 位 [ms] */
  uint16_t current_hp;  /* 本机当前血量 [hp] */
  uint16_t projectile_allowance_17mm;  /* 17mm 弹丸剩余发射次数 [count] */
  float    chassis_power;  /* 底盘实时功率 [W] */
  uint8_t  chassis_mode;  /* 电控实际模式 0=normal 1=spin_low 2=spin_high 3=estop */
  uint8_t  reserved;  /* 字节对齐占位 */
  uint8_t  game_progress;  /* 0=未开始 1=准备 2=自检 3=5s倒计时 4=比赛中 5=结算 */
  uint16_t stage_remain_time;  /* 当前阶段剩余时间 [s] */
  uint8_t  team_colour;  /* 1=红方 0=蓝方 */
  uint8_t  rfid_base;  /* 己方基地增益点 RFID（1=触发） */
  uint16_t ally_1_robot_hp;  /* 己方 1 号英雄血量 [hp] */
  uint16_t ally_2_robot_hp;  /* 己方 2 号工程血量 [hp] */
  uint16_t ally_3_robot_hp;  /* 己方 3 号步兵血量 [hp] */
  uint16_t ally_4_robot_hp;  /* 己方 4 号步兵血量 [hp] */
  uint16_t ally_7_robot_hp;  /* 己方 7 号哨兵血量 [hp] */
  uint16_t ally_outpost_hp;  /* 己方前哨站血量 [hp] */
  uint16_t ally_base_hp;  /* 己方基地血量 [hp] */
  uint32_t event_data;  /* 事件数据 bitfield */
  uint16_t checksum;
} __attribute__((packed)) SendTelemetryPacket;


/* ----------------------------------------------------------------------------
 * RX packet structs (ROS → MCU)
 * ------------------------------------------------------------------------- */

/* 0xA0 RecvControlPacket — 20B total (16B payload), 20Hz */
typedef struct
{
  uint8_t  header;
  uint8_t  len;

  float    lx;  /* 底盘 body 系前向线速度 [m/s] */
  float    ly;  /* 底盘 body 系侧向线速度 [m/s] */
  float    az;  /* 底盘角速度 / 自旋速度 [rad/s] */
  uint8_t  mode;  /* 0=normal 1=spin_low 2=spin_high 3=estop */
  uint8_t  reserved;  /* 字节对齐占位 */
  uint8_t  ros_state;  /* 0=init 1=ready 2=running 3=fault */
  uint8_t  reserved2;  /* 字节对齐占位 */
  uint16_t checksum;
} __attribute__((packed)) RecvControlPacket;


/* ----------------------------------------------------------------------------
 * Cached high-level state available to user code (after parsing RX frames).
 * ------------------------------------------------------------------------- */
typedef struct
{
  float    lx;  /* 底盘 body 系前向线速度 [m/s] */
  float    ly;  /* 底盘 body 系侧向线速度 [m/s] */
  float    az;  /* 底盘角速度 / 自旋速度 [rad/s] */
  uint8_t  mode;  /* 0=normal 1=spin_low 2=spin_high 3=estop */
  uint8_t  reserved;  /* 字节对齐占位 */
  bool     valid;         /* true once a Control frame was received and CRC ok */
  uint32_t last_recv_ms;  /* MCU tick of last successful Control */
} Navigation;

typedef struct
{
  uint8_t  ros_state;  /* 0=init 1=ready 2=running 3=fault */
  uint8_t  reserved2;  /* 字节对齐占位 */
  bool     valid;
  uint32_t last_recv_ms;
} HeartbeatState;

/* ----------------------------------------------------------------------------
 * Public API — call from FreeRTOS task or main loop
 * ------------------------------------------------------------------------- */
void Navigation_Init(void);
void Navigation_Task(void);                 /* call at 1kHz */
void Navigation_OnUsbReceive(const uint8_t *data, uint32_t len);  /* call from USB CDC RX ISR/callback */
bool Navigation_IsLinkAlive(void);          /* true if last Heartbeat within HEARTBEAT_TIMEOUT_MS */

/* Send hooks — MCU emits these. User code can also call manually. */
void send_TelemetryPacket(void);


/* CRC helper — MCU project should provide CRC16 (polynomial 0x1189 / table-based). */
void Append_CRC16_Check_Sum_Buf(uint8_t *buf, uint32_t len);
bool Verify_CRC16_Check_Sum_Buf(const uint8_t *buf, uint32_t len);

/* ----------------------------------------------------------------------------
 * Globals — exposed for legacy access patterns
 * ------------------------------------------------------------------------- */
extern SendTelemetryPacket    Smsg_telemetry;

extern RecvControlPacket      Rmsg_control;

extern Navigation       g_navigation;
extern HeartbeatState   g_heartbeat;

#endif  /* NAVIGATION_AUTO_H */
