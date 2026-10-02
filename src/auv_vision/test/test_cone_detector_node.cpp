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
#include <thread>
#include <utility>

#include "auv_interfaces/msg/cone_detection.hpp"
#include "auv_interfaces/msg/cone_detection_array.hpp"
#include "auv_vision/cone_detector_node.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "gtest/gtest.h"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{

TEST(ConeDetectorNode, PublishesStableDetectionsAndDebugImage)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("image_topic", "/cone_test/image"),
      rclcpp::Parameter("detections_topic", "/cone_test/detections"),
      rclcpp::Parameter("debug_topic", "/cone_test/debug"),
      rclcpp::Parameter("publish_debug_image", true),
      rclcpp::Parameter("history_size", 1),
      rclcpp::Parameter("required_votes", 1),
      rclcpp::Parameter("clear_votes", 1)});
  auto detector = auv_vision::make_cone_detector_node(options);
  auto driver = std::make_shared<rclcpp::Node>("cone_test_driver");
  auto image_publisher = driver->create_publisher<sensor_msgs::msg::Image>(
    "/cone_test/image", rclcpp::SensorDataQoS());

  auv_interfaces::msg::ConeDetectionArray::SharedPtr received_detections;
  sensor_msgs::msg::Image::SharedPtr received_debug;
  auto detection_subscription =
    driver->create_subscription<auv_interfaces::msg::ConeDetectionArray>(
    "/cone_test/detections", rclcpp::SensorDataQoS(),
    [&received_detections](auv_interfaces::msg::ConeDetectionArray::SharedPtr message) {
      received_detections = std::move(message);
    });
  auto debug_subscription = driver->create_subscription<sensor_msgs::msg::Image>(
    "/cone_test/debug", rclcpp::SensorDataQoS(),
    [&received_debug](sensor_msgs::msg::Image::SharedPtr message) {
      received_debug = std::move(message);
    });
  (void)detection_subscription;
  (void)debug_subscription;

  cv::Mat image(600, 600, CV_8UC3, cv::Scalar(180, 180, 180));
  cv::circle(image, cv::Point(300, 500), 45, cv::Scalar(0, 80, 255), cv::FILLED);
  std_msgs::msg::Header header;
  header.frame_id = "camera_down_optical_frame";
  const auto message = cv_bridge::CvImage(
    header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(detector);
  executor.add_node(driver);
  for (int attempt = 0; attempt < 100 && (!received_detections || !received_debug); ++attempt) {
    image_publisher->publish(*message);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_NE(received_detections, nullptr);
  ASSERT_EQ(received_detections->detections.size(), 1U);
  const auto & detection = received_detections->detections.front();
  EXPECT_EQ(detection.shape, auv_interfaces::msg::ConeDetection::SHAPE_CIRCLE);
  EXPECT_EQ(detection.row, 2);
  EXPECT_EQ(detection.col, 1);
  EXPECT_EQ(detection.detection.class_name, "circle_cone");
  EXPECT_GT(detection.detection.confidence, 0.75F);
  ASSERT_NE(received_debug, nullptr);
  EXPECT_EQ(received_debug->width, 600U);
  EXPECT_EQ(received_debug->height, 600U);

  executor.remove_node(driver);
  executor.remove_node(detector);
  rclcpp::shutdown();
}

}  // namespace
