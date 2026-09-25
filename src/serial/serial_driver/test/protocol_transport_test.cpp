// Copyright 2026. Apache-2.0.
#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/write_all.hpp"
extern "C" {
#include "navigation_auto.h"
uint32_t HAL_GetTick(void) {return 100;}
uint8_t CDC_Transmit_FS(uint8_t *, uint16_t) {return 0;}
}

void require(bool ok, const char * message)
{
  if (!ok) {throw std::runtime_error(message);}
}

std::vector<uint8_t> control(float lx)
{
  rm_serial_driver::TxControlPacket p{};
  p.lx = lx;
  p.ly = -0.5f;
  p.az = 0.75f;
  p.ros_state = 2;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&p), sizeof(p));
  return rm_serial_driver::toVector(p);
}

void expectControl(float lx)
{
  require(g_navigation.valid && g_navigation.lx == lx &&
    g_navigation.ly == -0.5f && g_navigation.az == 0.75f &&
    g_heartbeat.ros_state == 2 && Navigation_IsLinkAlive(), "Control not recovered");
}

int main()
{
  const auto good = control(1.25f);
  // C++ sender -> real C firmware parser, at every possible split boundary.
  for (size_t split = 0; split <= good.size(); ++split) {
    Navigation_Init();
    Navigation_OnUsbReceive(good.data(), split);
    Navigation_OnUsbReceive(good.data() + split, good.size() - split);
    expectControl(1.25f);
  }
  // Invalid length, undefined header, CRC error, and a truncated preceding frame.
  auto corrupt = good;
  corrupt.back() ^= 1;
  for (auto prefix : std::vector<std::vector<uint8_t>>{
      {0xA0, 0xFF}, {0xAF, 0xFF}, corrupt, {0xA0, 0x10, 0x00},
      std::vector<uint8_t>(5000, 0xAF)})
  {
    Navigation_Init();
    prefix.insert(prefix.end(), good.begin(), good.end());
    Navigation_OnUsbReceive(prefix.data(), prefix.size());
    expectControl(1.25f);
  }
  // Multiple frames in a single callback and a later command replacing the first.
  Navigation_Init();
  auto joined = good;
  const auto next = control(2.5f);
  joined.insert(joined.end(), next.begin(), next.end());
  Navigation_OnUsbReceive(joined.data(), joined.size());
  expectControl(2.5f);

  Navigation_Init();
  Navigation_OnUsbReceive(corrupt.data(), corrupt.size());
  require(!g_navigation.valid && !Navigation_IsLinkAlive(), "Bad CRC accepted");
  Navigation_Init();
  Navigation_OnUsbReceive(good.data(), 7);
  Navigation_Init();
  Navigation_OnUsbReceive(good.data(), good.size());
  expectControl(1.25f);

  // Short writes must preserve bytes exactly, without repeating an accepted prefix.
  std::vector<uint8_t> received;
  rm_serial_driver::writeAll(good, [&received](const std::vector<uint8_t> & bytes) {
    const size_t count = std::min<size_t>(3, bytes.size());
    received.insert(received.end(), bytes.begin(), bytes.begin() + count);
    return count;
  });
  require(received == good, "Short write corrupted frame");
  for (size_t invalid : {size_t{0}, good.size() + 1}) {
    bool failed = false;
    try {
      rm_serial_driver::writeAll(good, [invalid](const std::vector<uint8_t> &) {
        return invalid;
      });
    } catch (const std::runtime_error &) {failed = true;}
    require(failed, "Invalid send count treated as success");
  }
  bool failed = false;
  try {
    rm_serial_driver::writeAll(good, [](const std::vector<uint8_t> &) -> size_t {
      throw std::runtime_error("disconnected");
    });
  } catch (const std::runtime_error &) {failed = true;}
  require(failed, "Transport failure swallowed");
  std::puts("PASS: wire compatibility, splits, resync, oversized input, reset, short/failed writes");
}
