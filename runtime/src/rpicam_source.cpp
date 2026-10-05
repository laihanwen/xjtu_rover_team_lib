#include "rpicam_source.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <unistd.h>
#include <sys/wait.h>

#include <opencv2/imgcodecs.hpp>

namespace
{
constexpr std::size_t kMaximumMjpegBuffer = 4U * 1024U * 1024U;

std::string find_rpicam_vid()
{
  const char * path_value = std::getenv("PATH");
  const std::string path = path_value == nullptr ? "/usr/bin:/bin" : path_value;
  std::size_t start = 0U;
  while (start <= path.size()) {
    const std::size_t separator = path.find(':', start);
    const std::string directory = path.substr(
      start, separator == std::string::npos ? std::string::npos : separator-start);
    const std::string candidate = (directory.empty() ? "." : directory) + "/rpicam-vid";
    if (::access(candidate.c_str(), X_OK) == 0) return candidate;
    if (separator == std::string::npos) break;
    start = separator + 1U;
  }
  return {};
}
}

RpicamSource::RpicamSource(RpicamSourceConfig config)
: config_(std::move(config))
{
  if (config_.width <= 0 || config_.height <= 0 || config_.fps <= 0 ||
    config_.fps > 120 || config_.quality < 1 || config_.quality > 100 ||
    config_.brightness < -1.0 || config_.brightness > 1.0 || config_.gain < 0.0)
  {
    throw std::invalid_argument("invalid rpicam source configuration");
  }
}

RpicamSource::~RpicamSource()
{
  close();
}

bool RpicamSource::open()
{
  close();
  const std::string executable = find_rpicam_vid();
  if (executable.empty()) return false;
  int pipe_fds[2];
  if (::pipe2(pipe_fds, O_CLOEXEC) != 0) {
    return false;
  }

  std::vector<std::string> arguments{
    executable, "--codec", "mjpeg",
    "--width", std::to_string(config_.width),
    "--height", std::to_string(config_.height),
    "--framerate", std::to_string(config_.fps),
    "--nopreview", "--flush",
    "--ev", std::to_string(config_.exposure_compensation),
    "--denoise", config_.denoise,
    "--quality", std::to_string(config_.quality),
    "--brightness", std::to_string(config_.brightness),
    "-o", "-", "-t", "0"
  };
  if (config_.gain > 0.0) {
    arguments.insert(arguments.end() - 4, {"--gain", std::to_string(config_.gain)});
  }
  std::vector<char *> argv;
  argv.reserve(arguments.size() + 1U);
  for (auto & argument : arguments) {
    argv.push_back(argument.data());
  }
  argv.push_back(nullptr);

  const pid_t child = ::fork();
  if (child == 0) {
    ::dup2(pipe_fds[1], STDOUT_FILENO);
    ::close(pipe_fds[0]);
    ::close(pipe_fds[1]);
    ::execv(argv[0], argv.data());
    _exit(127);
  }
  ::close(pipe_fds[1]);
  if (child < 0) {
    ::close(pipe_fds[0]);
    return false;
  }
  read_fd_ = pipe_fds[0];
  child_pid_ = static_cast<int>(child);
  buffer_.clear();
  return true;
}

bool RpicamSource::extract_frame(cv::Mat & frame)
{
  static constexpr std::array<std::uint8_t, 2> start{{0xFFU, 0xD8U}};
  static constexpr std::array<std::uint8_t, 2> end{{0xFFU, 0xD9U}};
  auto begin = std::search(buffer_.begin(), buffer_.end(), start.begin(), start.end());
  if (begin == buffer_.end()) {
    if (buffer_.size() > 1U) {
      buffer_.erase(buffer_.begin(), buffer_.end() - 1);
    }
    return false;
  }
  if (begin != buffer_.begin()) {
    buffer_.erase(buffer_.begin(), begin);
  }
  auto finish = std::search(buffer_.begin() + 2, buffer_.end(), end.begin(), end.end());
  if (finish == buffer_.end()) {
    return false;
  }
  finish += 2;
  std::vector<std::uint8_t> jpeg(buffer_.begin(), finish);
  buffer_.erase(buffer_.begin(), finish);
  frame = cv::imdecode(jpeg, cv::IMREAD_COLOR);
  return !frame.empty();
}

bool RpicamSource::read(cv::Mat & frame, int timeout_ms)
{
  if (read_fd_ < 0 || child_pid_ < 0) {
    return false;
  }
  if (extract_frame(frame)) {
    return true;
  }
  pollfd descriptor{read_fd_, POLLIN, 0};
  const int ready = ::poll(&descriptor, 1, timeout_ms);
  if (ready <= 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    return false;
  }
  std::array<std::uint8_t, 65536> bytes{};
  const ssize_t count = ::read(read_fd_, bytes.data(), bytes.size());
  if (count <= 0) {
    return false;
  }
  buffer_.insert(buffer_.end(), bytes.begin(), bytes.begin() + count);
  if (buffer_.size() > kMaximumMjpegBuffer) {
    buffer_.erase(buffer_.begin(), buffer_.end() - 1);
  }
  return extract_frame(frame);
}

void RpicamSource::close()
{
  if (read_fd_ >= 0) {
    ::close(read_fd_);
    read_fd_ = -1;
  }
  if (child_pid_ >= 0) {
    const pid_t child = static_cast<pid_t>(child_pid_);
    ::kill(child, SIGTERM);
    bool exited = false;
    for (int attempt = 0; attempt < 20; ++attempt) {
      if (::waitpid(child, nullptr, WNOHANG) == child) {
        exited = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!exited) {
      ::kill(child, SIGKILL);
      (void)::waitpid(child, nullptr, 0);
    }
    child_pid_ = -1;
  }
  buffer_.clear();
}

bool RpicamSource::is_open()
{
  if (read_fd_ < 0 || child_pid_ < 0) {
    return false;
  }
  const pid_t child = static_cast<pid_t>(child_pid_);
  const pid_t result = ::waitpid(child, nullptr, WNOHANG);
  if (result == 0) return true;
  if (read_fd_ >= 0) {
    ::close(read_fd_);
    read_fd_ = -1;
  }
  child_pid_ = -1;
  buffer_.clear();
  return false;
}
