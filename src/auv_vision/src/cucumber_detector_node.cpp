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
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "auv_interfaces/msg/detection2_d.hpp"
#include "auv_interfaces/msg/object_detection_array.hpp"
#include "auv_vision/cucumber_detector_node.hpp"
#include "auv_vision/yolo_detector.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace auv_vision
{

class CucumberDetectorNode final : public rclcpp::Node
{
public:
  explicit CucumberDetectorNode(const rclcpp::NodeOptions & options)
  : Node("auv_cucumber_detector", options)
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/front/image_raw");
    image_transport_ = declare_parameter<std::string>("image_transport", "raw");
    detections_topic_ = declare_parameter<std::string>(
      "detections_topic", "/cucumber/detections");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/cucumber/debug_image");
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", false);
    const auto model_path = declare_parameter<std::string>("model_path", "");
    const auto model_sha256 = declare_parameter<std::string>("model_sha256", "");
    inference_rate_ = declare_parameter<double>("inference_rate", 5.0);
    if (image_topic_.empty() || image_transport_.empty() || detections_topic_.empty() ||
      inference_rate_ <= 0.0 || (publish_debug_image_ && debug_topic_.empty()))
    {
      throw std::invalid_argument("cucumber detector topics and inference_rate must be valid");
    }

    YoloDetectorConfig config;
    config.input_width = declare_parameter<int>("input_width", 640);
    config.input_height = declare_parameter<int>("input_height", 640);
    config.confidence_threshold = static_cast<float>(
      declare_parameter<double>("confidence_threshold", 0.5));
    config.nms_threshold = static_cast<float>(declare_parameter<double>("nms_threshold", 0.45));
    config.class_names = declare_parameter<std::vector<std::string>>(
      "class_names", {"sea_cucumber", "turtle", "starfish"});
    detector_ = std::make_unique<YoloDetector>(model_path, model_sha256, config);

    detections_publisher_ = create_publisher<auv_interfaces::msg::ObjectDetectionArray>(
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
      get_logger(), "cucumber detector ready: input=%s output=%s rate=%.2f Hz backend=OpenCV-DNN",
      image_topic_.c_str(), detections_topic_.c_str(), inference_rate_);
  }

private:
  void process_image(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    const rclcpp::Time stamp(message->header.stamp);
    if (last_inference_stamp_.nanoseconds() != 0 &&
      (stamp - last_inference_stamp_).seconds() < 1.0 / inference_rate_)
    {
      return;
    }
    last_inference_stamp_ = stamp;
    cv_bridge::CvImageConstPtr converted;
    try {
      converted = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "front image conversion failed: %s", error.what());
      return;
    }

    std::vector<YoloDetection> detections;
    try {
      detections = detector_->detect(converted->image);
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "YOLO inference failed: %s", error.what());
      return;
    }
    auv_interfaces::msg::ObjectDetectionArray output;
    output.header = message->header;
    output.detections.reserve(detections.size());
    for (const auto & detection : detections) {
      auv_interfaces::msg::Detection2D item;
      item.header = message->header;
      item.class_name = detection.class_name;
      item.confidence = detection.confidence;
      item.center.x = detection.box.x + detection.box.width / 2.0F;
      item.center.y = detection.box.y + detection.box.height / 2.0F;
      item.width = detection.box.width;
      item.height = detection.box.height;
      output.detections.push_back(std::move(item));
    }
    detections_publisher_->publish(output);

    if (publish_debug_image_) {
      cv::Mat debug = converted->image.clone();
      for (const auto & detection : detections) {
        cv::rectangle(debug, detection.box, cv::Scalar(0, 255, 0), 2);
        cv::putText(
          debug, detection.class_name + " " + cv::format("%.2f", detection.confidence),
          cv::Point(
            static_cast<int>(detection.box.x),
            std::max(15, static_cast<int>(detection.box.y) - 5)),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
      }
      debug_publisher_.publish(
        *cv_bridge::CvImage(
          message->header, sensor_msgs::image_encodings::BGR8, debug).toImageMsg());
    }
  }

  std::string image_topic_;
  std::string image_transport_;
  std::string detections_topic_;
  std::string debug_topic_;
  bool publish_debug_image_{false};
  double inference_rate_{5.0};
  rclcpp::Time last_inference_stamp_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<YoloDetector> detector_;
  rclcpp::Publisher<auv_interfaces::msg::ObjectDetectionArray>::SharedPtr detections_publisher_;
  image_transport::Subscriber subscription_;
  image_transport::Publisher debug_publisher_;
};

std::shared_ptr<rclcpp::Node> make_cucumber_detector_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<CucumberDetectorNode>(options);
}

}  // namespace auv_vision
