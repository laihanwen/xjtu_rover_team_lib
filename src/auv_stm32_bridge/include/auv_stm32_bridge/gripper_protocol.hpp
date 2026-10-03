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

#ifndef AUV_STM32_BRIDGE__GRIPPER_PROTOCOL_HPP_
#define AUV_STM32_BRIDGE__GRIPPER_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <vector>

namespace auv_stm32_bridge
{

constexpr uint8_t kGripperActuatorId = 1U;

enum class GripperAction : uint8_t
{
  kStop = 0U,
  kClose = 1U,
  kOpen = 2U
};

struct GripperTelemetry
{
  uint32_t sequence{0U};
  uint8_t state{0U};
  bool calibrated{false};
  uint16_t current_pulse_us{0U};
  uint16_t target_pulse_us{0U};
  uint8_t error_flags{0U};
  uint32_t last_command_sequence{0U};
};

std::array<uint8_t, 9> encode_gripper_command(uint32_t sequence, GripperAction action);
bool decode_gripper_status(const std::vector<uint8_t> & payload, GripperTelemetry & telemetry);

}  // namespace auv_stm32_bridge

#endif  // AUV_STM32_BRIDGE__GRIPPER_PROTOCOL_HPP_
