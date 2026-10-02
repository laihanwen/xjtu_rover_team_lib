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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

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
    publish_frame_rate_ = declare_parameter<double>(
      "publish_frame_rate", source_.config().frame_rate);
    if (
      topic_.empty() || frame_id_.empty() || reconnect_interval_ms_ <= 0 ||
      publish_frame_rate_ <= 0.0)
    {
      throw std::invalid_argument(
              "topic, frame_id, reconnect_interval_ms and publish_frame_rate must be valid");
    }

    publisher_ = image_transport::create_publisher(*this, topic_, rclcpp::SensorDataQoS());
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_frame_rate_));
    timer_ = create_wall_timer(period, std::bind(&CameraNode::publish_latest, this));

    if (source_.config().source.empty()) {
      RCLCPP_WARN(
        get_logger(), "camera source is empty; %s remains idle", get_fully_qualified_name());
    } else {
      capture_thread_ = std::thread(&CameraNode::capture_loop, this);
    }
  }

  ~CameraNode() override
  {
    stop_requested_.store(true);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
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

  bool open_source()
  {
    if (source_.open()) {
      const auto format = source_.negotiated_pixel_format();
      RCLCPP_INFO(
        get_logger(),
        "camera source opened: %s -> %s; negotiated=%dx%d %.3f FPS %.4s; publish=%.3f FPS",
        source_.config().source.c_str(), topic_.c_str(), source_.negotiated_width(),
        source_.negotiated_height(), source_.negotiated_frame_rate(), format.c_str(),
        publish_frame_rate_);
      return true;
    }
    RCLCPP_WARN(
      get_logger(), "unable to open camera source: %s; retrying",
      source_.config().source.c_str());
    return false;
  }

  void capture_loop()
  {
    auto next_file_frame = std::chrono::steady_clock::now();
    while (!stop_requested_.load()) {
      if (!source_.is_open()) {
        if (!open_source()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms_));
          continue;
        }
        next_file_frame = std::chrono::steady_clock::now();
      }

      cv::Mat frame;
      if (!source_.read(frame)) {
        if (!source_.is_live() && !source_.config().loop) {
          RCLCPP_INFO(get_logger(), "camera file or image sequence reached the end");
          source_.close();
          return;
        }
        RCLCPP_WARN(
          get_logger(), "camera read failed; reconnecting: %s",
          source_.config().source.c_str());
        source_.close();
        std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms_));
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = std::move(frame);
        latest_stamp_ = now();
        ++captured_sequence_;
      }

      // File and image-sequence sources are not clocked by a device. Preserve
      // their configured playback rate while live V4L2 capture runs freely.
      if (!source_.is_live()) {
        next_file_frame += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / source_.config().frame_rate));
        std::this_thread::sleep_until(next_file_frame);
      }
    }
    source_.close();
  }

  void publish_latest()
  {
    cv::Mat frame;
    rclcpp::Time stamp;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (latest_frame_.empty() || published_sequence_ == captured_sequence_) {
        return;
      }
      frame = latest_frame_.clone();
      stamp = latest_stamp_;
      published_sequence_ = captured_sequence_;
    }

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
    header.stamp = stamp;
    header.frame_id = frame_id_;
    publisher_.publish(*cv_bridge::CvImage(header, encoding, frame).toImageMsg());
  }

  CameraSource source_;
  std::string topic_;
  std::string frame_id_;
  int reconnect_interval_ms_{1000};
  double publish_frame_rate_{30.0};
  image_transport::Publisher publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread capture_thread_;
  std::atomic<bool> stop_requested_{false};
  std::mutex frame_mutex_;
  cv::Mat latest_frame_;
  rclcpp::Time latest_stamp_;
  std::uint64_t captured_sequence_{0U};
  std::uint64_t published_sequence_{0U};
};

}  // namespace auv_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auv_vision::CameraNode>());
  rclcpp::shutdown();
  return 0;
}
