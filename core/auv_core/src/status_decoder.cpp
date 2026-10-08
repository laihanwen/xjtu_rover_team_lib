#include "auv_core/status_decoder.hpp"
#include "auv_stm32_bridge/protocol.h"
#include <cmath>

namespace auv_core {
bool decode_status(const std::vector<std::uint8_t>& p, Stm32Status& out) {
  constexpr std::size_t fixed = 30;
  if (p.size() < fixed || p[29] > 8 || p.size() != fixed + 2U * p[29]) return false;
  Stm32Status s;
  s.sequence = auv_protocol_read_u32_le(p.data());
  s.armed = (p[4] & 1U) != 0;
  s.error_flags = auv_protocol_read_u32_le(p.data() + 5);
  s.voltage = auv_protocol_read_f32_le(p.data() + 9);
  s.depth = auv_protocol_read_f32_le(p.data() + 13);
  s.roll = auv_protocol_read_f32_le(p.data() + 17);
  s.pitch = auv_protocol_read_f32_le(p.data() + 21);
  s.yaw = auv_protocol_read_f32_le(p.data() + 25);
  s.voltage_valid = std::isfinite(s.voltage);
  s.telemetry_valid = std::isfinite(s.depth) &&
    std::isfinite(s.roll) && std::isfinite(s.pitch) && std::isfinite(s.yaw);
  for (std::size_t i = 0; i < p[29]; ++i) {
    s.thruster_outputs.push_back(auv_protocol_read_i16_le(p.data() + fixed + 2 * i) / 1000.0F);
  }
  out = std::move(s);
  return true;
}
}
