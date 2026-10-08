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

#include "auv_vision/valve_detector_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "auv_interfaces/msg/valve_detection.hpp"
#include "auv_vision/valve_detector.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace auv_vision
{

class ValveDetectorNode final : public rclcpp::Node
{
public:
  explicit ValveDetectorNode(const rclcpp::NodeOptions & options)
  : Node("auv_valve_detector", options)
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/front/image_raw");
    image_transport_ = declare_parameter<std::string>("image_transport", "raw");
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/valve/detection");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/valve/debug_image");
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", false);
    const double processing_rate = declare_parameter<double>("processing_rate", 15.0);
    if (image_topic_.empty() || image_transport_.empty() || detection_topic_.empty() ||
      (publish_debug_image_ && debug_topic_.empty()) || processing_rate <= 0.0 ||
      !std::isfinite(processing_rate))
    {
      throw std::invalid_argument("valve detector topic and transport parameters must be valid");
    }
    minimum_processing_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / processing_rate));

    ValveDetectorConfig config;
    config.minimum_area_ratio = declare_parameter<double>("minimum_area_ratio", 0.002);
    config.maximum_area_ratio = declare_parameter<double>("maximum_area_ratio", 0.65);
    config.minimum_circularity = declare_parameter<double>("minimum_circularity", 0.72);
    config.minimum_aspect_score = declare_parameter<double>("minimum_aspect_score", 0.72);
    config.minimum_confidence = declare_parameter<double>("minimum_confidence", 0.72);
    config.blur_kernel = declare_parameter<int>("blur_kernel", 5);
    config.canny_low = declare_parameter<double>("canny_low", 50.0);
    config.canny_high = declare_parameter<double>("canny_high", 150.0);
    config.morphology_kernel = declare_parameter<int>("morphology_kernel", 3);
    config.handle_minimum_length_ratio = declare_parameter<double>(
      "handle_minimum_length_ratio", 0.30);
    config.handle_maximum_center_offset_ratio = declare_parameter<double>(
      "handle_maximum_center_offset_ratio", 0.35);
    detector_ = std::make_unique<ValveDetector>(config);

    publisher_ = create_publisher<auv_interfaces::msg::ValveDetection>(
      detection_topic_, rclcpp::SensorDataQoS());
    if (publish_debug_image_) {
      debug_publisher_ = image_transport::create_publisher(
        *this, debug_topic_, rclcpp::SensorDataQoS());
    }
    subscription_ = image_transport::create_subscription(
      *this, image_topic_,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {process_image(message);},
      image_transport_, rclcpp::SensorDataQoS());
    RCLCPP_INFO(
      get_logger(), "valve detector ready: input=%s output=%s",
      image_topic_.c_str(), detection_topic_.c_str());
  }

private:
  void process_image(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_processing_time_ != std::chrono::steady_clock::time_point{} &&
      (now - last_processing_time_) < minimum_processing_period_)
    {
      return;
    }
    last_processing_time_ = now;

    cv_bridge::CvImageConstPtr converted;
    try {
      converted = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "valve image conversion failed: %s", error.what());
      return;
    }
    const ValveDetectorResult result = detector_->process(converted->image);
    auv_interfaces::msg::ValveDetection output;
    output.header = message->header;
    output.detected = result.observation.detected;
    output.confidence = result.observation.confidence;
    output.center.x = result.observation.center.x;
    output.center.y = result.observation.center.y;
    output.radius = result.observation.radius;
    output.circularity = result.observation.circularity;
    output.handle_angle_valid = result.observation.handle_angle_valid;
    output.handle_angle_rad = result.observation.handle_angle_rad;
    publisher_->publish(output);
    if (publish_debug_image_) {
      debug_publisher_.publish(
        *cv_bridge::CvImage(
          message->header, sensor_msgs::image_encodings::BGR8,
          result.debug_image).toImageMsg());
    }
  }

  std::string image_topic_;
  std::string image_transport_;
  std::string detection_topic_;
  std::string debug_topic_;
  bool publish_debug_image_{false};
  std::unique_ptr<ValveDetector> detector_;
  rclcpp::Publisher<auv_interfaces::msg::ValveDetection>::SharedPtr publisher_;
  image_transport::Subscriber subscription_;
  image_transport::Publisher debug_publisher_;
  std::chrono::steady_clock::duration minimum_processing_period_{};
  std::chrono::steady_clock::time_point last_processing_time_{};
};

std::shared_ptr<rclcpp::Node> make_valve_detector_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<ValveDetectorNode>(options);
}

}  // namespace auv_vision
