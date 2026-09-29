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

#include "auv_vision/camera_source.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace auv_vision
{

CameraSource::CameraSource(CameraSourceConfig config)
: config_(std::move(config))
{
  if (config_.width <= 0 || config_.height <= 0 || config_.frame_rate <= 0.0) {
    throw std::invalid_argument("camera width, height and frame_rate must be positive");
  }
  if (!config_.pixel_format.empty() && config_.pixel_format.size() != 4U) {
    throw std::invalid_argument("pixel_format must be empty or a four-character code");
  }
}

bool CameraSource::open()
{
  close();
  if (config_.source.empty()) {
    return false;
  }

  int device_index = 0;
  live_source_ = parse_device_index(config_.source, device_index) || is_device_path(config_.source);
  const bool opened = parse_device_index(config_.source, device_index) ?
    capture_.open(device_index, cv::CAP_V4L2) :
    capture_.open(config_.source, live_source_ ? cv::CAP_V4L2 : cv::CAP_ANY);
  if (!opened) {
    return false;
  }
  if (live_source_) {
    apply_device_settings();
  }
  return true;
}

void CameraSource::close()
{
  capture_.release();
}

bool CameraSource::is_open() const
{
  return capture_.isOpened();
}

bool CameraSource::read(cv::Mat & frame)
{
  if (!capture_.isOpened()) {
    return false;
  }
  if (capture_.read(frame) && !frame.empty()) {
    return true;
  }
  if (!live_source_ && config_.loop) {
    capture_.set(cv::CAP_PROP_POS_FRAMES, 0.0);
    return capture_.read(frame) && !frame.empty();
  }
  return false;
}

bool CameraSource::is_live() const
{
  return live_source_;
}

const CameraSourceConfig & CameraSource::config() const
{
  return config_;
}

bool CameraSource::parse_device_index(const std::string & source, int & index)
{
  if (source.empty() || !std::all_of(source.begin(), source.end(), [](const unsigned char value) {
      return std::isdigit(value) != 0;
    }))
  {
    return false;
  }
  try {
    std::size_t parsed = 0U;
    const int candidate = std::stoi(source, &parsed);
    if (parsed != source.size() || candidate < 0) {
      return false;
    }
    index = candidate;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool CameraSource::is_device_path(const std::string & source)
{
  return source.rfind("/dev/video", 0U) == 0U || source.rfind("/dev/v4l/", 0U) == 0U;
}

void CameraSource::apply_device_settings()
{
  capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
  capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
  capture_.set(cv::CAP_PROP_FPS, config_.frame_rate);
  capture_.set(cv::CAP_PROP_BUFFERSIZE, 1.0);
  if (!config_.pixel_format.empty()) {
    capture_.set(
      cv::CAP_PROP_FOURCC,
      cv::VideoWriter::fourcc(
        config_.pixel_format[0], config_.pixel_format[1], config_.pixel_format[2],
        config_.pixel_format[3]));
  }
}

}  // namespace auv_vision
