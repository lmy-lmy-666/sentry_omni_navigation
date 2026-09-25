// Copyright 2026. Apache-2.0.
#ifndef RM_SERIAL_DRIVER__WRITE_ALL_HPP_
#define RM_SERIAL_DRIVER__WRITE_ALL_HPP_

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace rm_serial_driver
{
// Completion means local transport acceptance, not an MCU acknowledgement.
template<typename Sender>
void writeAll(const std::vector<uint8_t> & data, Sender send)
{
  size_t offset = 0;
  while (offset < data.size()) {
    const std::vector<uint8_t> remaining(data.begin() + offset, data.end());
    const size_t sent = send(remaining);
    if (sent == 0 || sent > remaining.size()) {
      throw std::runtime_error("Serial write failed to make valid progress");
    }
    offset += sent;
  }
}
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__WRITE_ALL_HPP_
