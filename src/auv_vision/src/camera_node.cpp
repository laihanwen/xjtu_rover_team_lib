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

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "auv_vision/camera_source.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/header.hpp"

namespace auv_vision
{

class CameraNode final : public rclcpp::Node
{
public:
  CameraNode()
  : Node("auv_camera"),
    source_(make_config())
  {
    topic_ = declare_parameter<std::string>("topic", "image_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_optical_frame");
    reconnect_interval_ms_ = declare_parameter<int>("reconnect_interval_ms", 1000);
    if (topic_.empty() || frame_id_.empty() || reconnect_interval_ms_ <= 0) {
      throw std::invalid_argument("topic, frame_id and reconnect_interval_ms must be valid");
    }

    publisher_ = image_transport::create_publisher(*this, topic_, rclcpp::SensorDataQoS());
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / source_.config().frame_rate));
    timer_ = create_wall_timer(period, std::bind(&CameraNode::capture_once, this));

    if (source_.config().source.empty()) {
      RCLCPP_WARN(
        get_logger(), "camera source is empty; %s remains idle", get_fully_qualified_name());
    } else {
      open_source();
    }
  }

private:
  CameraSourceConfig make_config()
  {
    CameraSourceConfig config;
    config.source = declare_parameter<std::string>("source", "");
    config.width = declare_parameter<int>("width", 640);
    config.height = declare_parameter<int>("height", 480);
    config.frame_rate = declare_parameter<double>("frame_rate", 30.0);
    config.pixel_format = declare_parameter<std::string>("pixel_format", "MJPG");
    config.loop = declare_parameter<bool>("loop", false);
    return config;
  }

  void open_source()
  {
    last_open_attempt_ = std::chrono::steady_clock::now();
    if (source_.open()) {
      RCLCPP_INFO(
        get_logger(), "camera source opened: %s -> %s", source_.config().source.c_str(),
        topic_.c_str());
      open_failure_reported_ = false;
      read_failure_reported_ = false;
      return;
    }
    if (!open_failure_reported_) {
      RCLCPP_WARN(
        get_logger(), "unable to open camera source: %s", source_.config().source.c_str());
      open_failure_reported_ = true;
    }
  }

  void capture_once()
  {
    if (source_.config().source.empty() || source_finished_) {
      return;
    }
    if (!source_.is_open()) {
      const auto elapsed = std::chrono::steady_clock::now() - last_open_attempt_;
      if (elapsed >= std::chrono::milliseconds(reconnect_interval_ms_)) {
        open_source();
      }
      return;
    }

    cv::Mat frame;
    if (!source_.read(frame)) {
      if (!source_.is_live() && !source_.config().loop) {
        RCLCPP_INFO(get_logger(), "camera file or image sequence reached the end");
        source_.close();
        source_finished_ = true;
        return;
      }
      if (!read_failure_reported_) {
        RCLCPP_WARN(
          get_logger(), "camera read failed; reconnecting: %s",
          source_.config().source.c_str());
        read_failure_reported_ = true;
      }
      source_.close();
      last_open_attempt_ = std::chrono::steady_clock::now();
      return;
    }
    read_failure_reported_ = false;

    std::string encoding;
    if (frame.type() == CV_8UC3) {
      encoding = sensor_msgs::image_encodings::BGR8;
    } else if (frame.type() == CV_8UC1) {
      encoding = sensor_msgs::image_encodings::MONO8;
    } else if (frame.type() == CV_8UC4) {
      encoding = sensor_msgs::image_encodings::BGRA8;
    } else {
      RCLCPP_ERROR_ONCE(get_logger(), "unsupported OpenCV camera frame type: %d", frame.type());
      return;
    }

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id_;
    publisher_.publish(*cv_bridge::CvImage(header, encoding, frame).toImageMsg());
  }

  CameraSource source_;
  std::string topic_;
  std::string frame_id_;
  int reconnect_interval_ms_{1000};
  image_transport::Publisher publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point last_open_attempt_{};
  bool read_failure_reported_{false};
  bool open_failure_reported_{false};
  bool source_finished_{false};
};

}  // namespace auv_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auv_vision::CameraNode>());
  rclcpp::shutdown();
  return 0;
}
