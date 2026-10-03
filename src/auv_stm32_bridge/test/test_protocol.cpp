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

#include <algorithm>
#include <array>
#include <cstdint>

#include "auv_stm32_bridge/protocol.h"
#include "gtest/gtest.h"

namespace
{

TEST(Protocol, MatchesStandardCrcCheckValue)
{
  constexpr std::array<uint8_t, 9> input{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(auv_protocol_crc16_ccitt_false(input.data(), input.size()), 0x29B1U);
}

TEST(Protocol, EncodesHeartbeatGoldenVector)
{
  constexpr std::array<uint8_t, 8> payload{
    0x04, 0x03, 0x02, 0x01, 0x0D, 0x0C, 0x0B, 0x0A};
  constexpr std::array<uint8_t, 15> expected{
    0xAA, 0x55, 0x01, 0x01, 0x08, 0x04, 0x03, 0x02,
    0x01, 0x0D, 0x0C, 0x0B, 0x0A, 0x55, 0x8D};
  std::array<uint8_t, AUV_PROTOCOL_MAX_FRAME_SIZE> output{};

  const std::size_t size = auv_protocol_encode_frame(
    AUV_PROTOCOL_MSG_HEARTBEAT, payload.data(), payload.size(), output.data(), output.size());
  ASSERT_EQ(size, expected.size());
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), output.begin()));
}

TEST(Protocol, EncodesSetArmedGoldenVector)
{
  constexpr std::array<uint8_t, 5> payload{0x04, 0x03, 0x02, 0x01, 0x01};
  constexpr std::array<uint8_t, 12> expected{
    0xAA, 0x55, 0x01, 0x02, 0x05, 0x04, 0x03, 0x02, 0x01, 0x01, 0xA5, 0x0A};
  std::array<uint8_t, AUV_PROTOCOL_MAX_FRAME_SIZE> output{};

  const std::size_t size = auv_protocol_encode_frame(
    AUV_PROTOCOL_MSG_SET_ARMED, payload.data(), payload.size(), output.data(), output.size());
  ASSERT_EQ(size, expected.size());
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), output.begin()));
}

TEST(Protocol, EncodesMotionTargetGoldenVector)
{
  constexpr std::array<uint8_t, 20> payload{
    0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00,
    0x00, 0xBF, 0x00, 0x00, 0x20, 0x40, 0x00, 0x00, 0x00, 0x3F};
  constexpr std::array<uint8_t, 27> expected{
    0xAA, 0x55, 0x01, 0x03, 0x14, 0x78, 0x56, 0x34, 0x12,
    0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0xBF, 0x00,
    0x00, 0x20, 0x40, 0x00, 0x00, 0x00, 0x3F, 0xC1, 0x01};
  std::array<uint8_t, AUV_PROTOCOL_MAX_FRAME_SIZE> output{};

  const std::size_t size = auv_protocol_encode_frame(
    AUV_PROTOCOL_MSG_MOTION_TARGET, payload.data(), payload.size(), output.data(), output.size());
  ASSERT_EQ(size, expected.size());
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), output.begin()));
}

TEST(Protocol, EncodesGripperOpenGoldenVector)
{
  constexpr std::array<uint8_t, 9> payload{
    0x04, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0x80, 0x3F};
  constexpr std::array<uint8_t, 16> expected{
    0xAA, 0x55, 0x01, 0x04, 0x09, 0x04, 0x03, 0x02,
    0x01, 0x01, 0x00, 0x00, 0x80, 0x3F, 0x69, 0xF4};
  std::array<uint8_t, AUV_PROTOCOL_MAX_FRAME_SIZE> output{};

  const std::size_t size = auv_protocol_encode_frame(
    AUV_PROTOCOL_MSG_ACTUATOR_COMMAND, payload.data(), payload.size(), output.data(),
    output.size());
  ASSERT_EQ(size, expected.size());
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), output.begin()));
}

TEST(Protocol, RejectsInvalidEncodeArguments)
{
  std::array<uint8_t, AUV_PROTOCOL_MAX_FRAME_SIZE> output{};
  std::array<uint8_t, AUV_PROTOCOL_MAX_PAYLOAD_SIZE + 1U> oversized{};
  EXPECT_EQ(auv_protocol_encode_frame(0x42, nullptr, 0, output.data(), output.size()), 0U);
  EXPECT_EQ(
    auv_protocol_encode_frame(
      AUV_PROTOCOL_MSG_HEARTBEAT, nullptr, 1, output.data(), output.size()),
    0U);
  EXPECT_EQ(
    auv_protocol_encode_frame(
      AUV_PROTOCOL_MSG_HEARTBEAT, oversized.data(), oversized.size(), output.data(),
      output.size()),
    0U);
  EXPECT_EQ(
    auv_protocol_encode_frame(
      AUV_PROTOCOL_MSG_HEARTBEAT, nullptr, 0, output.data(), 1),
    0U);
}

TEST(Protocol, LittleEndianHelpersRoundTrip)
{
  std::array<uint8_t, 4> bytes{};
  auv_protocol_write_u32_le(bytes.data(), 0x78563412U);
  EXPECT_EQ(bytes, (std::array<uint8_t, 4>{0x12, 0x34, 0x56, 0x78}));
  EXPECT_EQ(auv_protocol_read_u32_le(bytes.data()), 0x78563412U);

  auv_protocol_write_i16_le(bytes.data(), -1000);
  EXPECT_EQ(auv_protocol_read_i16_le(bytes.data()), -1000);

  constexpr float value = -12.5F;
  auv_protocol_write_f32_le(bytes.data(), value);
  EXPECT_FLOAT_EQ(auv_protocol_read_f32_le(bytes.data()), value);
}

}  // namespace
