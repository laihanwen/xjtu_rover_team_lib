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

#include <cmath>

#include "auv_stm32_bridge/motion_target.hpp"
#include "auv_stm32_bridge/protocol.h"
#include "gtest/gtest.h"

namespace
{

TEST(MotionTargetGate, RequiresAllFreshInputs)
{
  auv_stm32_bridge::MotionTargetGate gate(250U, 2.0F, 20.0F);
  EXPECT_TRUE(gate.update_velocity(100U, 0.5, -0.25));
  EXPECT_TRUE(gate.update_depth(100U, 2.5));
  EXPECT_FALSE(gate.fresh_target(100U).has_value());
  EXPECT_TRUE(gate.update_yaw(100U, 1.0));
  ASSERT_TRUE(gate.fresh_target(100U).has_value());
  EXPECT_FALSE(gate.fresh_target(351U).has_value());
}

TEST(MotionTargetGate, RejectsUnsafeOrNonFiniteValues)
{
  auv_stm32_bridge::MotionTargetGate gate(250U, 2.0F, 20.0F);
  EXPECT_FALSE(gate.update_velocity(0U, 2.1, 0.0));
  EXPECT_FALSE(gate.update_velocity(0U, NAN, 0.0));
  EXPECT_FALSE(gate.update_depth(0U, -0.1));
  EXPECT_FALSE(gate.update_yaw(0U, 4.0));
}

TEST(MotionTargetGate, EncodesProtocolPayload)
{
  const auv_stm32_bridge::MotionTarget target{1.0F, -0.5F, 2.5F, 0.5F};
  const auto payload = auv_stm32_bridge::encode_motion_target_payload(0x12345678U, target);
  EXPECT_EQ(auv_protocol_read_u32_le(&payload[0]), 0x12345678U);
  EXPECT_FLOAT_EQ(auv_protocol_read_f32_le(&payload[4]), 1.0F);
  EXPECT_FLOAT_EQ(auv_protocol_read_f32_le(&payload[8]), -0.5F);
  EXPECT_FLOAT_EQ(auv_protocol_read_f32_le(&payload[12]), 2.5F);
  EXPECT_FLOAT_EQ(auv_protocol_read_f32_le(&payload[16]), 0.5F);
}

}  // namespace
