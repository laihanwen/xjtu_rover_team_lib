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

#ifndef AUV_STM32_BRIDGE__STREAM_PARSER_HPP_
#define AUV_STM32_BRIDGE__STREAM_PARSER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace auv_stm32_bridge
{

struct ProtocolFrame
{
  uint8_t message_type{0};
  std::vector<uint8_t> payload;
};

struct ParserStatistics
{
  std::size_t accepted_frames{0};
  std::size_t discarded_bytes{0};
  std::size_t invalid_versions{0};
  std::size_t unknown_message_types{0};
  std::size_t oversized_payloads{0};
  std::size_t crc_errors{0};
};

class StreamParser
{
public:
  std::vector<ProtocolFrame> consume(const uint8_t * data, std::size_t size);
  void reset();
  [[nodiscard]] const ParserStatistics & statistics() const noexcept;

private:
  std::vector<uint8_t> buffer_;
  ParserStatistics statistics_;
};

}  // namespace auv_stm32_bridge

#endif  // AUV_STM32_BRIDGE__STREAM_PARSER_HPP_
