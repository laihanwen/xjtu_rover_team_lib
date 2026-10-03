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
#include <thread>
#include <utility>

#include "auv_interfaces/msg/valve_detection.hpp"
#include "auv_vision/valve_detector_node.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "gtest/gtest.h"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

TEST(ValveDetectorNode, RejectsInvalidProcessingRate)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("processing_rate", 0.0)});
  EXPECT_THROW(auv_vision::make_valve_detector_node(options), std::invalid_argument);
  rclcpp::shutdown();
}

TEST(ValveDetectorNode, PublishesDetectionAndDebugImage)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("image_topic", "/valve_test/image"),
    rclcpp::Parameter("detection_topic", "/valve_test/detection"),
    rclcpp::Parameter("debug_topic", "/valve_test/debug"),
    rclcpp::Parameter("publish_debug_image", true)});
  auto detector = auv_vision::make_valve_detector_node(options);
  auto driver = std::make_shared<rclcpp::Node>("valve_test_driver");
  auto image_publisher = driver->create_publisher<sensor_msgs::msg::Image>(
    "/valve_test/image", rclcpp::SensorDataQoS());

  auv_interfaces::msg::ValveDetection::SharedPtr received_detection;
  sensor_msgs::msg::Image::SharedPtr received_debug;
  auto detection_subscription =
    driver->create_subscription<auv_interfaces::msg::ValveDetection>(
    "/valve_test/detection", rclcpp::SensorDataQoS(),
    [&received_detection](auv_interfaces::msg::ValveDetection::SharedPtr message) {
      received_detection = std::move(message);
    });
  auto debug_subscription = driver->create_subscription<sensor_msgs::msg::Image>(
    "/valve_test/debug", rclcpp::SensorDataQoS(),
    [&received_debug](sensor_msgs::msg::Image::SharedPtr message) {
      received_debug = std::move(message);
    });
  (void)detection_subscription;
  (void)debug_subscription;

  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::circle(image, cv::Point(320, 240), 90, cv::Scalar(255, 255, 255), 8);
  cv::line(
    image, cv::Point(250, 240), cv::Point(390, 240), cv::Scalar(255, 255, 255), 8);
  std_msgs::msg::Header header;
  header.frame_id = "camera_front_optical_frame";
  const auto message = cv_bridge::CvImage(
    header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(detector);
  executor.add_node(driver);
  for (int attempt = 0; attempt < 100 && (!received_detection || !received_debug); ++attempt) {
    image_publisher->publish(*message);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_NE(received_detection, nullptr);
  EXPECT_TRUE(received_detection->detected);
  EXPECT_EQ(received_detection->header.frame_id, "camera_front_optical_frame");
  EXPECT_NEAR(received_detection->center.x, 320.0F, 8.0F);
  EXPECT_TRUE(received_detection->handle_angle_valid);
  ASSERT_NE(received_debug, nullptr);
  EXPECT_EQ(received_debug->width, 640U);
  EXPECT_EQ(received_debug->height, 480U);

  executor.remove_node(driver);
  executor.remove_node(detector);
  rclcpp::shutdown();
}
