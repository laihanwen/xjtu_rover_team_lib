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

#include "auv_stm32_bridge/stream_parser.hpp"

#include <algorithm>
#include <array>
#include <iterator>

#include "auv_stm32_bridge/protocol.h"

namespace auv_stm32_bridge
{

std::vector<ProtocolFrame> StreamParser::consume(const uint8_t * data, const std::size_t size)
{
  std::vector<ProtocolFrame> frames;
  if (data == nullptr && size != 0U) {
    return frames;
  }
  if (size != 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  while (buffer_.size() >= 2U) {
    static constexpr std::array<uint8_t, 2> kSync{
      AUV_PROTOCOL_SYNC_0, AUV_PROTOCOL_SYNC_1};
    const auto sync = std::search(
      buffer_.begin(), buffer_.end(), kSync.begin(), kSync.end());
    if (sync == buffer_.end()) {
      const bool keep_trailing_sync = buffer_.back() == AUV_PROTOCOL_SYNC_0;
      statistics_.discarded_bytes += buffer_.size() - (keep_trailing_sync ? 1U : 0U);
      const uint8_t trailing = buffer_.back();
      buffer_.clear();
      if (keep_trailing_sync) {
        buffer_.push_back(trailing);
      }
      break;
    }
    statistics_.discarded_bytes += static_cast<std::size_t>(sync - buffer_.begin());
    buffer_.erase(buffer_.begin(), sync);
    if (buffer_.size() < AUV_PROTOCOL_FRAME_OVERHEAD) {
      break;
    }

    const uint8_t version = buffer_[2];
    const uint8_t message_type = buffer_[3];
    const std::size_t payload_size = buffer_[4];
    if (version != AUV_PROTOCOL_VERSION) {
      ++statistics_.invalid_versions;
      buffer_.erase(buffer_.begin());
      continue;
    }
    if (auv_protocol_is_known_message_type(message_type) == 0) {
      ++statistics_.unknown_message_types;
      buffer_.erase(buffer_.begin());
      continue;
    }
    if (payload_size > AUV_PROTOCOL_MAX_PAYLOAD_SIZE) {
      ++statistics_.oversized_payloads;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t frame_size = AUV_PROTOCOL_FRAME_OVERHEAD + payload_size;
    if (buffer_.size() < frame_size) {
      break;
    }
    const uint16_t expected_crc = auv_protocol_read_u16_le(
      &buffer_[AUV_PROTOCOL_HEADER_SIZE + payload_size]);
    const uint16_t actual_crc = auv_protocol_crc16_ccitt_false(&buffer_[2], 3U + payload_size);
    if (actual_crc != expected_crc) {
      ++statistics_.crc_errors;
      buffer_.erase(buffer_.begin());
      continue;
    }

    ProtocolFrame frame;
    frame.message_type = message_type;
    frame.payload.assign(
      buffer_.begin() + AUV_PROTOCOL_HEADER_SIZE,
      buffer_.begin() + AUV_PROTOCOL_HEADER_SIZE + payload_size);
    frames.push_back(std::move(frame));
    ++statistics_.accepted_frames;
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
  }
  return frames;
}

void StreamParser::reset()
{
  buffer_.clear();
  statistics_ = {};
}

const ParserStatistics & StreamParser::statistics() const noexcept
{
  return statistics_;
}

}  // namespace auv_stm32_bridge
