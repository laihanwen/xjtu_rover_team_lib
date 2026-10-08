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

#include "auv_stm32_bridge/serial_port.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace auv_stm32_bridge
{
namespace
{

speed_t baud_to_termios(const int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:
      throw std::invalid_argument("unsupported baud rate: " + std::to_string(baud_rate));
  }
}

std::runtime_error system_error(const std::string & operation)
{
  return std::runtime_error(operation + ": " + std::strerror(errno));
}

}  // namespace

SerialPort::~SerialPort()
{
  close();
}

void SerialPort::open(const std::string & device, const int baud_rate)
{
  if (device.empty()) {
    throw std::invalid_argument("serial device must not be empty");
  }
  const speed_t speed = baud_to_termios(baud_rate);

  close();
  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    throw system_error("open " + device);
  }
  // Cooperates with pyserial exclusive=True used by the ROV trial server.
  if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
    const auto error = system_error("serial already owned " + device);
    ::close(fd);
    throw error;
  }

  termios options{};
  if (tcgetattr(fd, &options) != 0) {
    const auto error = system_error("tcgetattr " + device);
    ::close(fd);
    throw error;
  }

  cfmakeraw(&options);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0 ||
    tcsetattr(fd, TCSANOW, &options) != 0)
  {
    const auto error = system_error("configure " + device);
    ::close(fd);
    throw error;
  }

  tcflush(fd, TCIOFLUSH);
  fd_ = fd;
}

void SerialPort::close() noexcept
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const noexcept
{
  return fd_ >= 0;
}

int SerialPort::native_handle() const noexcept
{
  return fd_;
}

std::size_t SerialPort::read(uint8_t * data, const std::size_t size)
{
  if (!is_open()) {
    throw std::logic_error("serial port is not open");
  }
  const ssize_t count = ::read(fd_, data, size);
  if (count < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }
    throw system_error("read serial port");
  }
  return static_cast<std::size_t>(count);
}

std::size_t SerialPort::write(const uint8_t * data, const std::size_t size)
{
  if (!is_open()) {
    throw std::logic_error("serial port is not open");
  }
  const ssize_t count = ::write(fd_, data, size);
  if (count < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }
    throw system_error("write serial port");
  }
  return static_cast<std::size_t>(count);
}

}  // namespace auv_stm32_bridge
