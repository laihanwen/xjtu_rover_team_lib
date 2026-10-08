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
#include <cstdint>
#include <vector>

#include "auv_stm32_bridge/protocol.h"
#include "auv_stm32_bridge/stream_parser.hpp"
#include "gtest/gtest.h"

namespace
{

constexpr std::array<uint8_t, 15> kHeartbeatFrame{
  0xAA, 0x55, 0x01, 0x01, 0x08, 0x04, 0x03, 0x02,
  0x01, 0x0D, 0x0C, 0x0B, 0x0A, 0x55, 0x8D};

TEST(StreamParser, ReassemblesEveryPossibleSplit)
{
  for (std::size_t split = 0; split < kHeartbeatFrame.size(); ++split) {
    auv_stm32_bridge::StreamParser parser;
    EXPECT_TRUE(parser.consume(kHeartbeatFrame.data(), split).empty());
    const auto frames = parser.consume(
      kHeartbeatFrame.data() + split, kHeartbeatFrame.size() - split);
    ASSERT_EQ(frames.size(), 1U) << "split=" << split;
    EXPECT_EQ(frames.front().message_type, AUV_PROTOCOL_MSG_HEARTBEAT);
    EXPECT_EQ(frames.front().payload.size(), 8U);
  }
}

TEST(StreamParser, ExtractsConcatenatedFramesAfterNoise)
{
  auv_stm32_bridge::StreamParser parser;
  std::vector<uint8_t> bytes{0x00, 0xAA, 0x11, 0x22};
  bytes.insert(bytes.end(), kHeartbeatFrame.begin(), kHeartbeatFrame.end());
  bytes.insert(bytes.end(), kHeartbeatFrame.begin(), kHeartbeatFrame.end());

  const auto frames = parser.consume(bytes.data(), bytes.size());
  EXPECT_EQ(frames.size(), 2U);
  EXPECT_EQ(parser.statistics().accepted_frames, 2U);
  EXPECT_EQ(parser.statistics().discarded_bytes, 4U);
}

TEST(StreamParser, RejectsBadCrcAndRecovers)
{
  auv_stm32_bridge::StreamParser parser;
  auto corrupted = kHeartbeatFrame;
  corrupted[7] ^= 0x80U;
  std::vector<uint8_t> bytes(corrupted.begin(), corrupted.end());
  bytes.insert(bytes.end(), kHeartbeatFrame.begin(), kHeartbeatFrame.end());

  const auto frames = parser.consume(bytes.data(), bytes.size());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(parser.statistics().crc_errors, 1U);
}

TEST(StreamParser, RejectsUnknownVersionTypeAndOversizedLength)
{
  auv_stm32_bridge::StreamParser parser;
  std::vector<uint8_t> bytes{
    0xAA, 0x55, 0x02, 0x01, 0x00, 0x00, 0x00,
    0xAA, 0x55, 0x01, 0x42, 0x00, 0x00, 0x00,
    0xAA, 0x55, 0x01, 0x01, 0xFF, 0x00, 0x00};
  bytes.insert(bytes.end(), kHeartbeatFrame.begin(), kHeartbeatFrame.end());

  const auto frames = parser.consume(bytes.data(), bytes.size());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(parser.statistics().invalid_versions, 1U);
  EXPECT_EQ(parser.statistics().unknown_message_types, 1U);
  EXPECT_EQ(parser.statistics().oversized_payloads, 1U);
}

TEST(StreamParser, PreservesTrailingSyncByte)
{
  auv_stm32_bridge::StreamParser parser;
  constexpr std::array<uint8_t, 3> noise{0x10, 0x20, 0xAA};
  EXPECT_TRUE(parser.consume(noise.data(), noise.size()).empty());
  const auto frames = parser.consume(kHeartbeatFrame.data() + 1, kHeartbeatFrame.size() - 1);
  EXPECT_EQ(frames.size(), 1U);
}

}  // namespace
