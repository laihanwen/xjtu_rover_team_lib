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

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "auv_stm32_bridge/serial_port.hpp"
#include "gtest/gtest.h"

namespace
{

class PseudoTerminal
{
public:
  PseudoTerminal()
  {
    std::array<char, 128> name{};
    if (openpty(&master_, &slave_, name.data(), nullptr, nullptr) != 0) {
      throw std::runtime_error("openpty failed");
    }
    slave_name_ = name.data();
  }

  ~PseudoTerminal()
  {
    ::close(master_);
    ::close(slave_);
  }

  int master() const {return master_;}
  const std::string & slave_name() const {return slave_name_;}

private:
  int master_{-1};
  int slave_{-1};
  std::string slave_name_;
};

TEST(SerialPort, RejectsInvalidConfiguration)
{
  auv_stm32_bridge::SerialPort port;
  EXPECT_THROW(port.open("", 115200), std::invalid_argument);
  EXPECT_THROW(port.open("/dev/null", 12345), std::invalid_argument);
}

TEST(SerialPort, ExchangesBytesThroughPseudoTerminal)
{
  PseudoTerminal terminal;
  auv_stm32_bridge::SerialPort port;
  port.open(terminal.slave_name(), 115200);
  ASSERT_TRUE(port.is_open());

  const std::array<uint8_t, 4> inbound{0xAA, 0x55, 0x01, 0x02};
  ASSERT_EQ(::write(terminal.master(), inbound.data(), inbound.size()), 4);
  std::array<uint8_t, 4> received{};
  EXPECT_EQ(port.read(received.data(), received.size()), received.size());
  EXPECT_EQ(received, inbound);

  const std::array<uint8_t, 3> outbound{0x10, 0x20, 0x30};
  EXPECT_EQ(port.write(outbound.data(), outbound.size()), outbound.size());
  std::array<uint8_t, 3> observed{};
  ASSERT_EQ(::read(terminal.master(), observed.data(), observed.size()), 3);
  EXPECT_EQ(observed, outbound);

  port.close();
  EXPECT_FALSE(port.is_open());
}

}  // namespace
