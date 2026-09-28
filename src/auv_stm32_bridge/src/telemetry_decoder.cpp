// Copyright 2026 hanwen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "auv_stm32_bridge/telemetry_decoder.hpp"

#include <cmath>

#include "auv_stm32_bridge/protocol.h"

namespace auv_stm32_bridge
{

bool decode_imu_telemetry(const std::vector<uint8_t> & payload, ImuTelemetry & telemetry)
{
  if (payload.size() != 40U) {
    return false;
  }
  const float roll = auv_protocol_read_f32_le(&payload[4]);
  const float pitch = auv_protocol_read_f32_le(&payload[8]);
  const float yaw = auv_protocol_read_f32_le(&payload[12]);
  if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) {
    return false;
  }

  telemetry.sequence = auv_protocol_read_u32_le(payload.data());
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  telemetry.orientation = {
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy};
  for (std::size_t index = 0; index < 3U; ++index) {
    telemetry.angular_velocity[index] = auv_protocol_read_f32_le(&payload[16U + index * 4U]);
    telemetry.linear_acceleration[index] = auv_protocol_read_f32_le(&payload[28U + index * 4U]);
  }
  telemetry.angular_velocity_available =
    std::isfinite(telemetry.angular_velocity[0]) &&
    std::isfinite(telemetry.angular_velocity[1]) &&
    std::isfinite(telemetry.angular_velocity[2]);
  telemetry.linear_acceleration_available =
    std::isfinite(telemetry.linear_acceleration[0]) &&
    std::isfinite(telemetry.linear_acceleration[1]) &&
    std::isfinite(telemetry.linear_acceleration[2]);
  return true;
}

bool decode_depth_telemetry(const std::vector<uint8_t> & payload, DepthTelemetry & telemetry)
{
  if (payload.size() != 9U || payload[8] > 1U) {
    return false;
  }
  telemetry.sequence = auv_protocol_read_u32_le(payload.data());
  telemetry.depth = auv_protocol_read_f32_le(&payload[4]);
  telemetry.valid = payload[8] == 1U;
  return !telemetry.valid || std::isfinite(telemetry.depth);
}

}  // namespace auv_stm32_bridge
