// AUTO-GENERATED from protocol.yaml — DO NOT EDIT
// Run `python3 generate.py` after modifying protocol.yaml.
//
// Frame format (v3.0):
//   [HEADER 1B] [LEN 1B] [PAYLOAD N B] [CRC16 2B]
//   - LEN     = N (payload bytes only, NOT including header/len/crc)
//   - CRC16   = covers [HEADER..PAYLOAD] (N+2 bytes), little-endian appended
//   - total   = N + 4
//
// Header range:
//   0x50-0x5F : MCU → ROS  (RX from ROS perspective)
//   0xA0-0xAF : ROS → MCU  (TX from ROS perspective)

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace rm_serial_driver
{

// ---------------------------------------------------------------------------
// Header constants
// ---------------------------------------------------------------------------
constexpr uint8_t HDR_RX_TELEMETRY       = 0x50;  // stm32_to_ros, 200Hz, frame=55B
constexpr uint8_t HDR_TX_CONTROL         = 0xA0;  // ros_to_stm32, 20Hz, frame=20B


// Convenience: set of all RX (MCU→ROS) headers
constexpr bool isRxHeader(uint8_t h)
{
  return
h == HDR_RX_TELEMETRY;


}

constexpr bool isTxHeader(uint8_t h)
{
  return
h == HDR_TX_CONTROL;


}

// ---------------------------------------------------------------------------
// RX packets (MCU → ROS)
// ---------------------------------------------------------------------------

// 0x50 RxTelemetryPacket — 55 bytes total (51B payload), 200Hz
struct RxTelemetryPacket
{
  uint8_t  header = HDR_RX_TELEMETRY;
  uint8_t  len    = 51;  // payload bytes

  float    gimbal_pitch;                 // 云台 pitch（绕 y） [rad]
  float    gimbal_yaw;                   // 云台 yaw（绕 z） [rad]
  float    chassis_pitch;                // 车体 pitch（绕 y），轮足姿态 [rad]
  float    chassis_yaw;                  // 车体 yaw（绕 z） [rad]
  uint16_t mcu_timestamp_ms;             // MCU 时间戳低 16 位 [ms]
  uint16_t current_hp;                   // 本机当前血量 [hp]
  uint16_t projectile_allowance_17mm;    // 17mm 弹丸剩余发射次数 [count]
  float    chassis_power;                // 底盘实时功率 [W]
  uint8_t  chassis_mode;                 // 电控实际模式 0=normal 1=spin_low 2=spin_high 3=estop
  uint8_t  reserved;                     // 字节对齐占位
  uint8_t  game_progress;                // 0=未开始 1=准备 2=自检 3=5s倒计时 4=比赛中 5=结算
  uint16_t stage_remain_time;            // 当前阶段剩余时间 [s]
  uint8_t  team_colour;                  // 1=红方 0=蓝方
  uint8_t  rfid_base;                    // 己方基地增益点 RFID（1=触发）
  uint16_t ally_1_robot_hp;              // 己方 1 号英雄血量 [hp]
  uint16_t ally_2_robot_hp;              // 己方 2 号工程血量 [hp]
  uint16_t ally_3_robot_hp;              // 己方 3 号步兵血量 [hp]
  uint16_t ally_4_robot_hp;              // 己方 4 号步兵血量 [hp]
  uint16_t ally_7_robot_hp;              // 己方 7 号哨兵血量 [hp]
  uint16_t ally_outpost_hp;              // 己方前哨站血量 [hp]
  uint16_t ally_base_hp;                 // 己方基地血量 [hp]
  uint32_t event_data;                   // 事件数据 bitfield
  uint16_t checksum = 0;  // CRC16 over [header..payload]
} __attribute__((packed));
static_assert(sizeof(RxTelemetryPacket) == 55, "RxTelemetryPacket wire size mismatch");


// ---------------------------------------------------------------------------
// TX packets (ROS → MCU)
// ---------------------------------------------------------------------------

// 0xA0 TxControlPacket — 20 bytes total (16B payload), 20Hz
struct TxControlPacket
{
  uint8_t  header = HDR_TX_CONTROL;
  uint8_t  len    = 16;

  float    lx;                           // 底盘 body 系前向线速度 [m/s]
  float    ly;                           // 底盘 body 系侧向线速度 [m/s]
  float    az;                           // 底盘角速度 / 自旋速度 [rad/s]
  uint8_t  mode;                         // 0=normal 1=spin_low 2=spin_high 3=estop
  uint8_t  reserved;                     // 字节对齐占位
  uint8_t  ros_state;                    // 0=init 1=ready 2=running 3=fault
  uint8_t  reserved2;                    // 字节对齐占位
  uint16_t checksum = 0;
} __attribute__((packed));
static_assert(sizeof(TxControlPacket) == 20, "TxControlPacket wire size mismatch");


// ---------------------------------------------------------------------------
// Header → frame size lookup. Returns 0 for unknown headers (caller should drop).
// Useful for building a generic dispatcher over [HEADER][LEN][PAYLOAD][CRC].
// ---------------------------------------------------------------------------
inline size_t frameSizeForHeader(uint8_t header)
{
  switch (header) {
case HDR_RX_TELEMETRY: return 55;
case HDR_TX_CONTROL: return 20;

    default: return 0;
  }
}

inline size_t payloadSizeForHeader(uint8_t header)
{
  switch (header) {
case HDR_RX_TELEMETRY: return 51;
case HDR_TX_CONTROL: return 16;

    default: return 0;
  }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::memcpy(reinterpret_cast<uint8_t *>(&packet), data.data(),
              std::min(data.size(), sizeof(T)));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::memcpy(packet.data(), reinterpret_cast<const uint8_t *>(&data), sizeof(T));
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
