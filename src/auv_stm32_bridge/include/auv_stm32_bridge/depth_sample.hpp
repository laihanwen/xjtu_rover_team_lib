#pragma once
#include "auv_stm32_bridge/protocol.h"
#include <cstdint>
#include <vector>
#include <cmath>
namespace auv_stm32_bridge {
struct DepthSampleTelemetry {
  std::uint32_t sensor_sequence{0};double depth_m{0},age_sec{0};bool valid{false};
};
inline DepthSampleTelemetry decode_depth_sample(const std::vector<std::uint8_t>& payload) {
  DepthSampleTelemetry out;
  if(payload.size()!=17)return out; // Legacy 9-byte frame has no sample identity.
  out.depth_m=auv_protocol_read_f32_le(payload.data()+4);
  out.sensor_sequence=auv_protocol_read_u32_le(payload.data()+9);
  out.age_sec=auv_protocol_read_u32_le(payload.data()+13)/1000.0;
  out.valid=payload[8]==1 && out.sensor_sequence && std::isfinite(out.depth_m) && out.depth_m>=0 && out.age_sec<=3;
  return out;
}
}
