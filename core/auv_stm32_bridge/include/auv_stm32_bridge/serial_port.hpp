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

#ifndef AUV_STM32_BRIDGE__SERIAL_PORT_HPP_
#define AUV_STM32_BRIDGE__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace auv_stm32_bridge
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  void open(const std::string & device, int baud_rate);
  void close() noexcept;
  [[nodiscard]] bool is_open() const noexcept;
  [[nodiscard]] int native_handle() const noexcept;

  std::size_t read(uint8_t * data, std::size_t size);
  std::size_t write(const uint8_t * data, std::size_t size);

private:
  int fd_{-1};
};

}  // namespace auv_stm32_bridge

#endif  // AUV_STM32_BRIDGE__SERIAL_PORT_HPP_
