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
#include <cstdint>
#include <limits>
#include <vector>

#include "auv_stm32_bridge/protocol.h"
#include "auv_stm32_bridge/telemetry_decoder.hpp"
#include "gtest/gtest.h"

namespace
{

void write_f32(std::vector<uint8_t> & payload, const std::size_t offset, const float value)
{
  auv_protocol_write_f32_le(&payload[offset], value);
}

TEST(TelemetryDecoder, DecodesImuAndConvertsRpyToQuaternion)
{
  std::vector<uint8_t> payload(40U, 0U);
  auv_protocol_write_u32_le(payload.data(), 42U);
  write_f32(payload, 12U, static_cast<float>(M_PI_2));
  write_f32(payload, 16U, 1.0F);
  write_f32(payload, 20U, 2.0F);
  write_f32(payload, 24U, 3.0F);
  write_f32(payload, 28U, 4.0F);
  write_f32(payload, 32U, 5.0F);
  write_f32(payload, 36U, 6.0F);

  auv_stm32_bridge::ImuTelemetry telemetry;
  ASSERT_TRUE(auv_stm32_bridge::decode_imu_telemetry(payload, telemetry));
  EXPECT_EQ(telemetry.sequence, 42U);
  EXPECT_NEAR(telemetry.orientation[2], std::sqrt(0.5), 1e-6);
  EXPECT_NEAR(telemetry.orientation[3], std::sqrt(0.5), 1e-6);
  EXPECT_TRUE(telemetry.angular_velocity_available);
  EXPECT_TRUE(telemetry.linear_acceleration_available);
  EXPECT_DOUBLE_EQ(telemetry.angular_velocity[1], 2.0);
  EXPECT_DOUBLE_EQ(telemetry.linear_acceleration[2], 6.0);
}

TEST(TelemetryDecoder, MarksUnavailableImuVectors)
{
  std::vector<uint8_t> payload(40U, 0U);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (std::size_t offset = 16U; offset < 40U; offset += 4U) {
    write_f32(payload, offset, nan);
  }

  auv_stm32_bridge::ImuTelemetry telemetry;
  ASSERT_TRUE(auv_stm32_bridge::decode_imu_telemetry(payload, telemetry));
  EXPECT_FALSE(telemetry.angular_velocity_available);
  EXPECT_FALSE(telemetry.linear_acceleration_available);
}

TEST(TelemetryDecoder, RejectsMalformedImu)
{
  auv_stm32_bridge::ImuTelemetry telemetry;
  EXPECT_FALSE(auv_stm32_bridge::decode_imu_telemetry(std::vector<uint8_t>(39U), telemetry));
  std::vector<uint8_t> payload(40U, 0U);
  write_f32(payload, 4U, std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(auv_stm32_bridge::decode_imu_telemetry(payload, telemetry));
}

TEST(TelemetryDecoder, DecodesValidAndUnavailableDepth)
{
  std::vector<uint8_t> payload(9U, 0U);
  auv_protocol_write_u32_le(payload.data(), 7U);
  write_f32(payload, 4U, 1.25F);
  payload[8] = 1U;
  auv_stm32_bridge::DepthTelemetry telemetry;
  ASSERT_TRUE(auv_stm32_bridge::decode_depth_telemetry(payload, telemetry));
  EXPECT_EQ(telemetry.sequence, 7U);
  EXPECT_FLOAT_EQ(telemetry.depth, 1.25F);
  EXPECT_TRUE(telemetry.valid);

  write_f32(payload, 4U, std::numeric_limits<float>::quiet_NaN());
  payload[8] = 0U;
  ASSERT_TRUE(auv_stm32_bridge::decode_depth_telemetry(payload, telemetry));
  EXPECT_FALSE(telemetry.valid);
  EXPECT_TRUE(std::isnan(telemetry.depth));
}

TEST(TelemetryDecoder, RejectsInvalidDepth)
{
  auv_stm32_bridge::DepthTelemetry telemetry;
  EXPECT_FALSE(auv_stm32_bridge::decode_depth_telemetry(std::vector<uint8_t>(8U), telemetry));
  std::vector<uint8_t> payload(9U, 0U);
  payload[8] = 2U;
  EXPECT_FALSE(auv_stm32_bridge::decode_depth_telemetry(payload, telemetry));
  payload[8] = 1U;
  write_f32(payload, 4U, std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(auv_stm32_bridge::decode_depth_telemetry(payload, telemetry));
}

}  // namespace
