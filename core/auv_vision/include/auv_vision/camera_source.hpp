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

#ifndef AUV_VISION__CAMERA_SOURCE_HPP_
#define AUV_VISION__CAMERA_SOURCE_HPP_

#include <string>

#include "opencv2/core/mat.hpp"
#include "opencv2/videoio.hpp"

namespace auv_vision
{

struct CameraSourceConfig
{
  std::string source;
  int width{640};
  int height{480};
  double frame_rate{30.0};
  std::string pixel_format{"MJPG"};
  bool loop{false};
};

class CameraSource
{
public:
  explicit CameraSource(CameraSourceConfig config);

  bool open();
  void close();
  bool is_open() const;
  bool read(cv::Mat & frame);
  bool is_live() const;
  const CameraSourceConfig & config() const;
  int negotiated_width() const;
  int negotiated_height() const;
  double negotiated_frame_rate() const;
  std::string negotiated_pixel_format() const;

private:
  static bool parse_device_index(const std::string & source, int & index);
  static bool is_device_path(const std::string & source);
  void apply_device_settings();

  CameraSourceConfig config_;
  cv::VideoCapture capture_;
  bool live_source_{false};
};

}  // namespace auv_vision

#endif  // AUV_VISION__CAMERA_SOURCE_HPP_
