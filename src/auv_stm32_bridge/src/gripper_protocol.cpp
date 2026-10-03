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

#include "auv_stm32_bridge/gripper_protocol.hpp"

#include "auv_stm32_bridge/protocol.h"

namespace auv_stm32_bridge
{

std::array<uint8_t, 9> encode_gripper_command(
  const uint32_t sequence, const GripperAction action)
{
  std::array<uint8_t, 9> payload{};
  float normalized = 0.0F;
  if (action == GripperAction::kClose) {
    normalized = -1.0F;
  } else if (action == GripperAction::kOpen) {
    normalized = 1.0F;
  }
  auv_protocol_write_u32_le(payload.data(), sequence);
  payload[4] = kGripperActuatorId;
  auv_protocol_write_f32_le(&payload[5], normalized);
  return payload;
}

bool decode_gripper_status(
  const std::vector<uint8_t> & payload, GripperTelemetry & telemetry)
{
  if (payload.size() != 16U || payload[4] != kGripperActuatorId) {
    return false;
  }
  telemetry.sequence = auv_protocol_read_u32_le(payload.data());
  telemetry.state = payload[5];
  telemetry.calibrated = (payload[6] & 0x01U) != 0U;
  telemetry.current_pulse_us = auv_protocol_read_u16_le(&payload[7]);
  telemetry.target_pulse_us = auv_protocol_read_u16_le(&payload[9]);
  telemetry.error_flags = payload[11];
  telemetry.last_command_sequence = auv_protocol_read_u32_le(&payload[12]);
  return true;
}

}  // namespace auv_stm32_bridge
