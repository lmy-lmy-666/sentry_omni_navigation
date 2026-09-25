/* AUTO-GENERATED from protocol.yaml — DO NOT EDIT
 * Run `python3 generate.py` after modifying protocol.yaml.
 *
 * This file is a *reference implementation* for the STM32 side. It depends on:
 *   - cmsis_os.h            (for osKernelSysTick or HAL_GetTick)
 *   - usbd_cdc_if.h         (CDC_Transmit_FS / CDC_Receive_FS)
 *   - ins_Task.h            (INS.Pitch / INS.Yaw / chassis attitude)
 *   - referee.h             (Game_Status, Robot_Status, Game_Robot_HP, RFID_Status)
 *   - ModeControl.h         (chassis runtime mode + power)
 *
 * Anything tagged "TODO: connect to ..." is the integration point that the
 * firmware engineer must wire to the real data sources on this MCU project.
 */

#include "navigation_auto.h"

#include "cmsis_os.h"
#include "usbd_cdc_if.h"

/* TODO: connect to project-specific headers. Comment out if not present. */
#if __has_include("CRCs.h")
#  include "CRCs.h"
#endif
#if __has_include("ins_Task.h")
#  include "ins_Task.h"
#endif
#if __has_include("referee.h")
#  include "referee.h"
#endif
#if __has_include("ModeControl.h")
#  include "ModeControl.h"
#endif

/* ----------------------------------------------------------------------------
 * Globals
 * ------------------------------------------------------------------------- */
SendTelemetryPacket Smsg_telemetry;

RecvControlPacket Rmsg_control;


Navigation     g_navigation;
HeartbeatState g_heartbeat;

static uint8_t  s_usb_tx_buf[APP_TX_DATA_SIZE];
static uint8_t  s_usb_rx_buf[APP_RX_DATA_SIZE];
static uint32_t s_rx_fill = 0;
static uint32_t s_tick_counter = 0;  /* incremented every Navigation_Task() call */

/* ----------------------------------------------------------------------------
 * CRC16 (polynomial 0x1189, init 0xFFFF, little-endian append).
 * If your project already provides Append_CRC16_Check_Sum / CRC16_Verify in
 * CRCs.h you can delete this and forward to those.
 * ------------------------------------------------------------------------- */
#ifndef CRC16_PROVIDED_BY_PROJECT
static const uint16_t W_CRC_TABLE[256] = {
  0x0000,0x1189,0x2312,0x329B,0x4624,0x57AD,0x6536,0x74BF,0x8C48,0x9DC1,0xAF5A,0xBED3,0xCA6C,0xDBE5,0xE97E,0xF8F7,
  0x1081,0x0108,0x3393,0x221A,0x56A5,0x472C,0x75B7,0x643E,0x9CC9,0x8D40,0xBFDB,0xAE52,0xDAED,0xCB64,0xF9FF,0xE876,
  0x2102,0x308B,0x0210,0x1399,0x6726,0x76AF,0x4434,0x55BD,0xAD4A,0xBCC3,0x8E58,0x9FD1,0xEB6E,0xFAE7,0xC87C,0xD9F5,
  0x3183,0x200A,0x1291,0x0318,0x77A7,0x662E,0x54B5,0x453C,0xBDCB,0xAC42,0x9ED9,0x8F50,0xFBEF,0xEA66,0xD8FD,0xC974,
  0x4204,0x538D,0x6116,0x709F,0x0420,0x15A9,0x2732,0x36BB,0xCE4C,0xDFC5,0xED5E,0xFCD7,0x8868,0x99E1,0xAB7A,0xBAF3,
  0x5285,0x430C,0x7197,0x601E,0x14A1,0x0528,0x37B3,0x263A,0xDECD,0xCF44,0xFDDF,0xEC56,0x98E9,0x8960,0xBBFB,0xAA72,
  0x6306,0x728F,0x4014,0x519D,0x2522,0x34AB,0x0630,0x17B9,0xEF4E,0xFEC7,0xCC5C,0xDDD5,0xA96A,0xB8E3,0x8A78,0x9BF1,
  0x7387,0x620E,0x5095,0x411C,0x35A3,0x242A,0x16B1,0x0738,0xFFCF,0xEE46,0xDCDD,0xCD54,0xB9EB,0xA862,0x9AF9,0x8B70,
  0x8408,0x9581,0xA71A,0xB693,0xC22C,0xD3A5,0xE13E,0xF0B7,0x0840,0x19C9,0x2B52,0x3ADB,0x4E64,0x5FED,0x6D76,0x7CFF,
  0x9489,0x8500,0xB79B,0xA612,0xD2AD,0xC324,0xF1BF,0xE036,0x18C1,0x0948,0x3BD3,0x2A5A,0x5EE5,0x4F6C,0x7DF7,0x6C7E,
  0xA50A,0xB483,0x8618,0x9791,0xE32E,0xF2A7,0xC03C,0xD1B5,0x2942,0x38CB,0x0A50,0x1BD9,0x6F66,0x7EEF,0x4C74,0x5DFD,
  0xB58B,0xA402,0x9699,0x8710,0xF3AF,0xE226,0xD0BD,0xC134,0x39C3,0x284A,0x1AD1,0x0B58,0x7FE7,0x6E6E,0x5CF5,0x4D7C,
  0xC60C,0xD785,0xE51E,0xF497,0x8028,0x91A1,0xA33A,0xB2B3,0x4A44,0x5BCD,0x6956,0x78DF,0x0C60,0x1DE9,0x2F72,0x3EFB,
  0xD68D,0xC704,0xF59F,0xE416,0x90A9,0x8120,0xB3BB,0xA232,0x5AC5,0x4B4C,0x79D7,0x685E,0x1CE1,0x0D68,0x3FF3,0x2E7A,
  0xE70E,0xF687,0xC41C,0xD595,0xA12A,0xB0A3,0x8238,0x93B1,0x6B46,0x7ACF,0x4854,0x59DD,0x2D62,0x3CEB,0x0E70,0x1FF9,
  0xF78F,0xE606,0xD49D,0xC514,0xB1AB,0xA022,0x92B9,0x8330,0x7BC7,0x6A4E,0x58D5,0x495C,0x3DE3,0x2C6A,0x1EF1,0x0F78,
};

static uint16_t crc16_compute(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint32_t i = 0; i < len; ++i) {
    crc = (crc >> 8) ^ W_CRC_TABLE[(crc ^ data[i]) & 0xFF];
  }
  return crc;
}

void Append_CRC16_Check_Sum_Buf(uint8_t *buf, uint32_t len)
{
  if (buf == 0 || len <= 2) return;
  uint16_t crc = crc16_compute(buf, len - 2);
  buf[len - 2] = (uint8_t)(crc & 0xFF);
  buf[len - 1] = (uint8_t)((crc >> 8) & 0xFF);
}

bool Verify_CRC16_Check_Sum_Buf(const uint8_t *buf, uint32_t len)
{
  if (buf == 0 || len <= 2) return false;
  uint16_t expected = (uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
  uint16_t actual = crc16_compute(buf, len - 2);
  return expected == actual;
}
#endif  /* CRC16_PROVIDED_BY_PROJECT */

/* ----------------------------------------------------------------------------
 * Helper: timestamp in ms (HAL_GetTick if FreeRTOS not running yet)
 * ------------------------------------------------------------------------- */
static uint32_t now_ms(void)
{
#ifdef osCMSIS
  return osKernelSysTick();
#else
  extern uint32_t HAL_GetTick(void);
  return HAL_GetTick();
#endif
}

/* ----------------------------------------------------------------------------
 * Init
 * ------------------------------------------------------------------------- */
void Navigation_Init(void)
{
memset(&Smsg_telemetry, 0, sizeof(Smsg_telemetry));
  Smsg_telemetry.header = HDR_TX_TELEMETRY;
  Smsg_telemetry.len    = 51;

memset(&Rmsg_control, 0, sizeof(Rmsg_control));

  memset(&g_navigation, 0, sizeof(g_navigation));
  memset(&g_heartbeat, 0, sizeof(g_heartbeat));
  s_tick_counter = 0;
  s_rx_fill = 0;
}

/* ----------------------------------------------------------------------------
 * Send helpers
 * ------------------------------------------------------------------------- */

/* 0x50 send_TelemetryPacket — 200Hz */
void send_TelemetryPacket(void)
{
  Smsg_telemetry.header = HDR_TX_TELEMETRY;
  Smsg_telemetry.len    = 51;


  /* TODO: connect to project IMU/INS module. */
#ifdef INS_H
  Smsg_telemetry.gimbal_pitch  = -INS.Pitch / 57.3f;
  Smsg_telemetry.gimbal_yaw    =  INS.Yaw   / 57.3f;
  Smsg_telemetry.chassis_pitch = -INS.Pitch / 57.3f;  /* TODO: replace with chassis IMU pitch */
  Smsg_telemetry.chassis_yaw   =  INS.Yaw   / 57.3f;  /* TODO: replace with chassis IMU yaw */
#endif
  Smsg_telemetry.mcu_timestamp_ms = (uint16_t)(now_ms() & 0xFFFF);
  /* TODO: connect to referee.h / ModeControl — use cached values, refresh at 1Hz elsewhere */
#ifdef REFEREE_H
  Smsg_telemetry.current_hp                = Robot_Status.current_HP;
  Smsg_telemetry.projectile_allowance_17mm = Projectile_Allowance.projectile_allowance_17mm;
  Smsg_telemetry.game_progress             = Game_Status.game_progress;
  Smsg_telemetry.stage_remain_time       = Game_Status.stage_remain_time;
  Smsg_telemetry.team_colour             = (Robot_Status.robot_id < 100) ? 1 : 0;
  Smsg_telemetry.rfid_base               = (RFID_Status.rfid_status == 1) ? 1 : 0;
  Smsg_telemetry.ally_1_robot_hp         = Game_Robot_HP.ally_1_robot_HP;
  Smsg_telemetry.ally_2_robot_hp         = Game_Robot_HP.ally_2_robot_HP;
  Smsg_telemetry.ally_3_robot_hp         = Game_Robot_HP.ally_3_robot_HP;
  Smsg_telemetry.ally_4_robot_hp         = Game_Robot_HP.ally_4_robot_HP;
  Smsg_telemetry.ally_7_robot_hp         = Game_Robot_HP.ally_7_robot_HP;
  Smsg_telemetry.ally_outpost_hp         = Game_Robot_HP.ally_outpost_HP;
  Smsg_telemetry.ally_base_hp            = Game_Robot_HP.ally_base_HP;
  Smsg_telemetry.event_data              = Game_Status.event_data;
#endif
#ifdef MODE_CONTROL_H
  Smsg_telemetry.chassis_power = ModeControl_GetChassisPower();
  Smsg_telemetry.chassis_mode  = (uint8_t)ModeControl_GetChassisMode();
#endif
  Smsg_telemetry.reserved = 0;


  Append_CRC16_Check_Sum_Buf((uint8_t *)&Smsg_telemetry, sizeof(Smsg_telemetry));
  memcpy(s_usb_tx_buf, &Smsg_telemetry, sizeof(Smsg_telemetry));
  CDC_Transmit_FS(s_usb_tx_buf, sizeof(Smsg_telemetry));
}


/* ----------------------------------------------------------------------------
 * Receive — generic frame parser
 * ------------------------------------------------------------------------- */
static void parse_frame(const uint8_t *data, uint32_t total_len)
{
  if (total_len < 4) return;          /* too short to be any frame */
  uint8_t header = data[0];
  uint8_t plen   = data[1];
  if ((uint32_t)plen + 4 != total_len) return;  /* LEN mismatch, drop */

  if (!Verify_CRC16_Check_Sum_Buf(data, total_len)) {
    return;  /* CRC bad, drop silently (could log if needed) */
  }

  switch (header) {
case HDR_RX_CONTROL: {
      if (sizeof(RecvControlPacket) != total_len) return;
      memcpy(&Rmsg_control, data, sizeof(RecvControlPacket));

      g_navigation.lx           = Rmsg_control.lx;
      g_navigation.ly           = Rmsg_control.ly;
      g_navigation.az           = Rmsg_control.az;
      g_navigation.mode         = Rmsg_control.mode;
      g_navigation.reserved     = Rmsg_control.reserved;
      g_navigation.valid        = true;
      g_navigation.last_recv_ms = now_ms();
      g_heartbeat.ros_state     = Rmsg_control.ros_state;
      g_heartbeat.reserved2     = Rmsg_control.reserved2;
      g_heartbeat.valid         = true;
      g_heartbeat.last_recv_ms  = now_ms();

      break;
    }

    default:
      /* Unknown header — drop. */
      break;
  }
}

/* ----------------------------------------------------------------------------
 * USB CDC RX entry point. The CDC stack hands us a chunk that may contain
 * one frame, multiple frames, or a partial frame. We do a simple stream
 * parser using LEN to find frame boundaries.
 *
 * Call this from CDC_Receive_FS callback. NOT inside an ISR with priorities
 * higher than FreeRTOS API.
 * ------------------------------------------------------------------------- */
/* Only accept defined headers and their exact wire sizes. */
static uint32_t rx_frame_size(uint8_t header)
{
  switch (header) {

    case HDR_RX_CONTROL: return 20;

    default: return 0;
  }
}

void Navigation_OnUsbReceive(const uint8_t *data, uint32_t len)
{
  if (data == 0 || len == 0) return;

  /* Consume incrementally: even a callback larger than the buffer is safe.
   * At each step only a partial known frame can remain in the buffer. */
  for (uint32_t n = 0; n < len; ++n) {
    if (s_rx_fill >= APP_RX_DATA_SIZE) s_rx_fill = 0;
    s_usb_rx_buf[s_rx_fill++] = data[n];

    while (s_rx_fill >= 2) {
      uint32_t frame_len = rx_frame_size(s_usb_rx_buf[0]);
      if (frame_len == 0 || s_usb_rx_buf[1] != frame_len - 4) {
        /* Bad header/length: shift one byte, never wait on an arbitrary LEN. */
        memmove(s_usb_rx_buf, s_usb_rx_buf + 1, --s_rx_fill);
        continue;
      }
      if (s_rx_fill < frame_len) break;
      if (!Verify_CRC16_Check_Sum_Buf(s_usb_rx_buf, frame_len)) {
        /* A valid frame may start inside this corrupt candidate. */
        memmove(s_usb_rx_buf, s_usb_rx_buf + 1, --s_rx_fill);
        continue;
      }
      parse_frame(s_usb_rx_buf, frame_len);
      s_rx_fill -= frame_len;
      memmove(s_usb_rx_buf, s_usb_rx_buf + frame_len, s_rx_fill);
    }
  }
}

bool Navigation_IsLinkAlive(void)
{
  if (!g_heartbeat.valid) return false;
  return (now_ms() - g_heartbeat.last_recv_ms) < HEARTBEAT_TIMEOUT_MS;
}

/* ----------------------------------------------------------------------------
 * Periodic dispatcher — call at 1kHz (osDelay(1) loop or 1ms timer).
 * ------------------------------------------------------------------------- */
void Navigation_Task(void)
{
  s_tick_counter++;

  /* Watchdog: if heartbeat lost, force g_navigation to estop. */
  if (!Navigation_IsLinkAlive()) {
    g_navigation.lx       = 0.0f;
    g_navigation.ly       = 0.0f;
    g_navigation.az       = 0.0f;
    g_navigation.mode     = (uint8_t)CHASSIS_MODE_ESTOP;
  }

  /* Dispatch periodic sends. */
if ((s_tick_counter % SEND_TELEMETRY_INTERVAL_MS) == 0) {
    send_TelemetryPacket();
  }

}
