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

#include <array>
#include <vector>

#include "auv_stm32_bridge/gripper_protocol.hpp"
#include "auv_stm32_bridge/protocol.h"
#include "gtest/gtest.h"

namespace
{

using auv_stm32_bridge::GripperAction;
using auv_stm32_bridge::GripperTelemetry;

TEST(GripperProtocol, EncodesDiscreteCommands)
{
  const auto close = auv_stm32_bridge::encode_gripper_command(0x12345678U, GripperAction::kClose);
  EXPECT_EQ(auv_protocol_read_u32_le(close.data()), 0x12345678U);
  EXPECT_EQ(close[4], auv_stm32_bridge::kGripperActuatorId);
  EXPECT_FLOAT_EQ(auv_protocol_read_f32_le(&close[5]), -1.0F);
  const auto open = auv_stm32_bridge::encode_gripper_command(2U, GripperAction::kOpen);
  EXPECT_FLOAT_EQ(auv_protocol_read_f32_le(&open[5]), 1.0F);
  const auto stop = auv_stm32_bridge::encode_gripper_command(3U, GripperAction::kStop);
  EXPECT_FLOAT_EQ(auv_protocol_read_f32_le(&stop[5]), 0.0F);
}

TEST(GripperProtocol, DecodesStatusAndRejectsWrongLayout)
{
  std::vector<uint8_t> payload(16U, 0U);
  auv_protocol_write_u32_le(payload.data(), 7U);
  payload[4] = auv_stm32_bridge::kGripperActuatorId;
  payload[5] = 5U;
  payload[6] = 1U;
  auv_protocol_write_u16_le(&payload[7], 1900U);
  auv_protocol_write_u16_le(&payload[9], 2000U);
  payload[11] = 4U;
  auv_protocol_write_u32_le(&payload[12], 42U);
  GripperTelemetry telemetry;
  ASSERT_TRUE(auv_stm32_bridge::decode_gripper_status(payload, telemetry));
  EXPECT_EQ(telemetry.sequence, 7U);
  EXPECT_EQ(telemetry.state, 5U);
  EXPECT_TRUE(telemetry.calibrated);
  EXPECT_EQ(telemetry.current_pulse_us, 1900U);
  EXPECT_EQ(telemetry.target_pulse_us, 2000U);
  EXPECT_EQ(telemetry.error_flags, 4U);
  EXPECT_EQ(telemetry.last_command_sequence, 42U);
  payload.pop_back();
  EXPECT_FALSE(auv_stm32_bridge::decode_gripper_status(payload, telemetry));
}

}  // namespace
