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

#include "auv_stm32_bridge/motion_target.hpp"

#include <cmath>
#include <stdexcept>

#include "auv_stm32_bridge/protocol.h"

namespace auv_stm32_bridge
{

MotionTargetGate::MotionTargetGate(
  const uint64_t timeout_ms, const float max_velocity_mps, const float max_depth_m)
: timeout_ms_(timeout_ms),
  max_velocity_mps_(max_velocity_mps),
  max_depth_m_(max_depth_m)
{
  if (timeout_ms == 0U || !std::isfinite(max_velocity_mps) || max_velocity_mps <= 0.0F ||
    !std::isfinite(max_depth_m) || max_depth_m <= 0.0F)
  {
    throw std::invalid_argument("motion target limits and timeout must be positive");
  }
}

bool MotionTargetGate::update_velocity(const uint64_t now_ms, const double vx, const double vy)
{
  if (!std::isfinite(vx) || !std::isfinite(vy) ||
    std::abs(vx) > max_velocity_mps_ || std::abs(vy) > max_velocity_mps_)
  {
    return false;
  }
  target_.vx = static_cast<float>(vx);
  target_.vy = static_cast<float>(vy);
  velocity_time_ms_ = now_ms;
  velocity_seen_ = true;
  return true;
}

bool MotionTargetGate::update_depth(const uint64_t now_ms, const double depth)
{
  if (!std::isfinite(depth) || depth < 0.0 || depth > max_depth_m_) {return false;}
  target_.depth = static_cast<float>(depth);
  depth_time_ms_ = now_ms;
  depth_seen_ = true;
  return true;
}

bool MotionTargetGate::update_yaw(const uint64_t now_ms, const double yaw)
{
  constexpr double kPi = 3.14159265358979323846;
  if (!std::isfinite(yaw) || yaw < -kPi || yaw > kPi) {return false;}
  target_.yaw = static_cast<float>(yaw);
  yaw_time_ms_ = now_ms;
  yaw_seen_ = true;
  return true;
}

bool MotionTargetGate::fresh(
  const bool seen, const uint64_t timestamp_ms, const uint64_t now_ms) const
{
  return seen && now_ms >= timestamp_ms && now_ms - timestamp_ms <= timeout_ms_;
}

std::optional<MotionTarget> MotionTargetGate::fresh_target(const uint64_t now_ms) const
{
  if (!fresh(velocity_seen_, velocity_time_ms_, now_ms) ||
    !fresh(depth_seen_, depth_time_ms_, now_ms) ||
    !fresh(yaw_seen_, yaw_time_ms_, now_ms))
  {
    return std::nullopt;
  }
  return target_;
}

std::array<uint8_t, 20> encode_motion_target_payload(
  const uint32_t sequence, const MotionTarget & target)
{
  std::array<uint8_t, 20> payload{};
  auv_protocol_write_u32_le(&payload[0], sequence);
  auv_protocol_write_f32_le(&payload[4], target.vx);
  auv_protocol_write_f32_le(&payload[8], target.vy);
  auv_protocol_write_f32_le(&payload[12], target.depth);
  auv_protocol_write_f32_le(&payload[16], target.yaw);
  return payload;
}

}  // namespace auv_stm32_bridge
