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

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "auv_interfaces/msg/cone_detection.hpp"
#include "auv_interfaces/msg/cone_detection_array.hpp"
#include "auv_vision/cone_detector.hpp"
#include "auv_vision/cone_detector_node.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace auv_vision
{
namespace
{

cv::Scalar read_color_parameter(
  rclcpp::Node & node, const std::string & name, const std::vector<std::int64_t> & defaults)
{
  const auto values = node.declare_parameter<std::vector<std::int64_t>>(name, defaults);
  if (values.size() != 3U) {
    throw std::invalid_argument(name + " must contain three values");
  }
  return cv::Scalar(values[0], values[1], values[2]);
}

}  // namespace

class ConeDetectorNode final : public rclcpp::Node
{
public:
  explicit ConeDetectorNode(const rclcpp::NodeOptions & options)
  : Node("auv_cone_detector", options)
  {
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/mapping/rectified_image");
    image_transport_ = declare_parameter<std::string>("image_transport", "raw");
    detections_topic_ = declare_parameter<std::string>(
      "detections_topic", "/cones/detections");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/cones/debug_image");
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", false);
    if (image_topic_.empty() || image_transport_.empty() || detections_topic_.empty() ||
      (publish_debug_image_ && debug_topic_.empty()))
    {
      throw std::invalid_argument("cone detector topic and transport parameters must be valid");
    }

    ConeDetectorConfig detector_config;
    detector_config.hsv_lower_1 = read_color_parameter(*this, "hsv_lower_1", {0, 80, 50});
    detector_config.hsv_upper_1 = read_color_parameter(*this, "hsv_upper_1", {20, 255, 255});
    detector_config.use_second_hsv_range = declare_parameter<bool>(
      "use_second_hsv_range", true);
    detector_config.hsv_lower_2 = read_color_parameter(*this, "hsv_lower_2", {165, 80, 50});
    detector_config.hsv_upper_2 = read_color_parameter(*this, "hsv_upper_2", {179, 255, 255});
    detector_config.use_lab_mask = declare_parameter<bool>("use_lab_mask", false);
    detector_config.lab_lower = read_color_parameter(*this, "lab_lower", {0, 0, 0});
    detector_config.lab_upper = read_color_parameter(*this, "lab_upper", {255, 255, 255});
    detector_config.clahe_clip_limit = declare_parameter<double>("clahe_clip_limit", 2.0);
    detector_config.morphology_kernel = declare_parameter<int>("morphology_kernel", 5);
    detector_config.cell_margin_ratio = declare_parameter<double>("cell_margin_ratio", 0.12);
    detector_config.minimum_area_ratio = declare_parameter<double>(
      "minimum_area_ratio", 0.025);
    detector_config.maximum_area_ratio = declare_parameter<double>(
      "maximum_area_ratio", 0.65);
    detector_config.polygon_epsilon_ratio = declare_parameter<double>(
      "polygon_epsilon_ratio", 0.035);
    detector_config.minimum_solidity = declare_parameter<double>("minimum_solidity", 0.80);
    detector_config.minimum_aspect_score = declare_parameter<double>(
      "minimum_aspect_score", 0.70);
    detector_config.circle_minimum_circularity = declare_parameter<double>(
      "circle_minimum_circularity", 0.80);
    detector_config.square_minimum_extent = declare_parameter<double>(
      "square_minimum_extent", 0.62);
    detector_config.minimum_confidence = declare_parameter<double>(
      "minimum_confidence", 0.72);
    detector_ = std::make_unique<ConeDetector>(detector_config);

    ConeTrackerConfig tracker_config;
    tracker_config.history_size = declare_parameter<int>("history_size", 5);
    tracker_config.required_votes = declare_parameter<int>("required_votes", 3);
    tracker_config.clear_votes = declare_parameter<int>("clear_votes", 3);
    tracker_ = std::make_unique<ConeTracker>(tracker_config);

    detections_publisher_ = create_publisher<auv_interfaces::msg::ConeDetectionArray>(
      detections_topic_, rclcpp::SensorDataQoS());
    if (publish_debug_image_) {
      debug_publisher_ = image_transport::create_publisher(
        *this, debug_topic_, rclcpp::SensorDataQoS());
    }
    subscription_ = image_transport::create_subscription(
      *this, image_topic_,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {process_image(message);},
      image_transport_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(), "cone detector ready: input=%s transport=%s output=%s",
      image_topic_.c_str(), image_transport_.c_str(), detections_topic_.c_str());
  }

private:
  void process_image(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    cv_bridge::CvImageConstPtr converted;
    try {
      converted = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "cone image conversion failed: %s", error.what());
      return;
    }

    const ConeDetectorResult result = detector_->process(converted->image);
    const auto stable_observations = tracker_->update(result.observations);
    auv_interfaces::msg::ConeDetectionArray output;
    output.header = message->header;
    output.detections.reserve(stable_observations.size());
    for (const auto & observation : stable_observations) {
      auv_interfaces::msg::ConeDetection detection;
      detection.detection.header = message->header;
      detection.detection.class_name = cone_shape_name(observation.shape);
      detection.detection.confidence = observation.confidence;
      detection.detection.center.x = observation.center.x;
      detection.detection.center.y = observation.center.y;
      detection.detection.width = observation.width;
      detection.detection.height = observation.height;
      detection.shape = static_cast<std::uint8_t>(observation.shape);
      detection.row = static_cast<std::int8_t>(observation.row);
      detection.col = static_cast<std::int8_t>(observation.col);
      detection.area = observation.area;
      detection.circularity = observation.circularity;
      output.detections.push_back(std::move(detection));
    }
    detections_publisher_->publish(output);

    if (publish_debug_image_) {
      debug_publisher_.publish(
        *cv_bridge::CvImage(
          message->header, sensor_msgs::image_encodings::BGR8,
          result.debug_image).toImageMsg());
    }
  }

  std::string image_topic_;
  std::string image_transport_;
  std::string detections_topic_;
  std::string debug_topic_;
  bool publish_debug_image_{false};
  std::unique_ptr<ConeDetector> detector_;
  std::unique_ptr<ConeTracker> tracker_;
  rclcpp::Publisher<auv_interfaces::msg::ConeDetectionArray>::SharedPtr detections_publisher_;
  image_transport::Subscriber subscription_;
  image_transport::Publisher debug_publisher_;
};

std::shared_ptr<rclcpp::Node> make_cone_detector_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<ConeDetectorNode>(options);
}

}  // namespace auv_vision
