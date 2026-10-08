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

#ifndef AUV_STM32_BRIDGE__MOTION_TARGET_HPP_
#define AUV_STM32_BRIDGE__MOTION_TARGET_HPP_

#include <array>
#include <cstdint>
#include <optional>

namespace auv_stm32_bridge
{

struct MotionTarget
{
  float vx{0.0F};
  float vy{0.0F};
  float depth{0.0F};
  float yaw{0.0F};
};

class MotionTargetGate
{
public:
  MotionTargetGate(uint64_t timeout_ms, float max_velocity_mps, float max_depth_m);
  bool update_velocity(uint64_t now_ms, double vx, double vy);
  bool update_depth(uint64_t now_ms, double depth);
  bool update_yaw(uint64_t now_ms, double yaw);
  std::optional<MotionTarget> fresh_target(uint64_t now_ms) const;

private:
  bool fresh(bool seen, uint64_t timestamp_ms, uint64_t now_ms) const;

  uint64_t timeout_ms_;
  float max_velocity_mps_;
  float max_depth_m_;
  MotionTarget target_;
  uint64_t velocity_time_ms_{0U};
  uint64_t depth_time_ms_{0U};
  uint64_t yaw_time_ms_{0U};
  bool velocity_seen_{false};
  bool depth_seen_{false};
  bool yaw_seen_{false};
};

std::array<uint8_t, 20> encode_motion_target_payload(
  uint32_t sequence, const MotionTarget & target);

}  // namespace auv_stm32_bridge

#endif  // AUV_STM32_BRIDGE__MOTION_TARGET_HPP_
