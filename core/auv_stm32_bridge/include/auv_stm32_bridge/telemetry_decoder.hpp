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

#ifndef AUV_STM32_BRIDGE__TELEMETRY_DECODER_HPP_
#define AUV_STM32_BRIDGE__TELEMETRY_DECODER_HPP_

#include <array>
#include <cstdint>
#include <vector>

namespace auv_stm32_bridge
{

struct ImuTelemetry
{
  uint32_t sequence{};
  std::array<double, 4> orientation{};  // x, y, z, w
  std::array<double, 3> angular_velocity{};
  std::array<double, 3> linear_acceleration{};
  bool angular_velocity_available{};
  bool linear_acceleration_available{};
};

struct DepthTelemetry
{
  uint32_t sequence{};
  float depth{};
  bool valid{};
};

bool decode_imu_telemetry(const std::vector<uint8_t> & payload, ImuTelemetry & telemetry);
bool decode_depth_telemetry(const std::vector<uint8_t> & payload, DepthTelemetry & telemetry);

}  // namespace auv_stm32_bridge

#endif  // AUV_STM32_BRIDGE__TELEMETRY_DECODER_HPP_
