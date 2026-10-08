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

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#include "auv_interfaces/msg/semantic_map.hpp"
#include "auv_interfaces/msg/cone_detection.hpp"
#include "auv_interfaces/msg/cone_detection_array.hpp"
#include "auv_interfaces/msg/grid_cell.hpp"
#include "auv_interfaces/msg/grid_pose.hpp"
#include "auv_mapping/semantic_mapper_node.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "gtest/gtest.h"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{

// A synthetic down-camera frame: a perspective-warped 3x3 grid with three
// white edges, one yellow bottom edge, and white internal divisions over a
// dark-blue pool floor. Mirrors make_grid() in test_grid_mapper.cpp.
cv::Mat make_node_test_grid()
{
  cv::Mat canonical(500, 500, CV_8UC3, cv::Scalar(120, 60, 20));
  cv::rectangle(canonical, cv::Rect(8, 8, 484, 484), cv::Scalar(255, 255, 255), 18);
  cv::line(canonical, cv::Point(8, 492), cv::Point(492, 492), cv::Scalar(0, 255, 255), 18);
  cv::line(canonical, cv::Point(167, 10), cv::Point(167, 490), cv::Scalar(255, 255, 255), 8);
  cv::line(canonical, cv::Point(333, 10), cv::Point(333, 490), cv::Scalar(255, 255, 255), 8);
  cv::line(canonical, cv::Point(10, 167), cv::Point(490, 167), cv::Scalar(255, 255, 255), 8);
  cv::line(canonical, cv::Point(10, 333), cv::Point(490, 333), cv::Scalar(255, 255, 255), 8);

  const std::array<cv::Point2f, 4> source{
    cv::Point2f(0.0F, 0.0F), cv::Point2f(499.0F, 0.0F),
    cv::Point2f(499.0F, 499.0F), cv::Point2f(0.0F, 499.0F)};
  const std::array<cv::Point2f, 4> destination{
    cv::Point2f(115.0F, 80.0F), cv::Point2f(550.0F, 115.0F),
    cv::Point2f(590.0F, 430.0F), cv::Point2f(70.0F, 455.0F)};
  const cv::Mat transform = cv::getPerspectiveTransform(source.data(), destination.data());
  cv::Mat perspective(520, 660, CV_8UC3, cv::Scalar(120, 60, 20));
  cv::warpPerspective(
    canonical, perspective, transform, perspective.size(), cv::INTER_LINEAR,
    cv::BORDER_TRANSPARENT);
  return perspective;
}

TEST(SemanticMapperNode, PublishesCompleteRowMajorMapAndRectifiedImage)
{
  rclcpp::init(0, nullptr);
  EXPECT_THROW(
    {
      const auto uncalibrated_mapper = auv_mapping::make_semantic_mapper_node();
      (void)uncalibrated_mapper;
    },
    std::invalid_argument);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("require_calibration", false),
      rclcpp::Parameter("stable_frames", 1),
      rclcpp::Parameter("yellow_oriented_frames", 1),
      rclcpp::Parameter("expected_cone_count", 1),
      rclcpp::Parameter("image_topic", "/mapping_test/image"),
      rclcpp::Parameter("map_topic", "/mapping_test/map"),
      rclcpp::Parameter("cone_detections_topic", "/mapping_test/cones"),
      rclcpp::Parameter("grid_pose_topic", "/mapping_test/pose"),
      rclcpp::Parameter("visited_cell_topic", "/mapping_test/visited"),
      rclcpp::Parameter("rectified_topic", "/mapping_test/rectified")});
  auto mapper = auv_mapping::make_semantic_mapper_node(options);
  auto driver = std::make_shared<rclcpp::Node>("mapping_test_driver");
  auto image_publisher = driver->create_publisher<sensor_msgs::msg::Image>(
    "/mapping_test/image", rclcpp::SensorDataQoS());
  auto cone_publisher = driver->create_publisher<auv_interfaces::msg::ConeDetectionArray>(
    "/mapping_test/cones", rclcpp::SensorDataQoS());
  auto visited_publisher = driver->create_publisher<auv_interfaces::msg::GridCell>(
    "/mapping_test/visited", rclcpp::QoS(10).reliable());

  auv_interfaces::msg::SemanticMap::SharedPtr received_map;
  sensor_msgs::msg::Image::SharedPtr received_rectified;
  auv_interfaces::msg::GridPose::SharedPtr received_pose;
  auto map_subscription = driver->create_subscription<auv_interfaces::msg::SemanticMap>(
    "/mapping_test/map", rclcpp::QoS(1).reliable().transient_local(),
    [&received_map](auv_interfaces::msg::SemanticMap::SharedPtr message) {
      received_map = std::move(message);
    });
  auto image_subscription = driver->create_subscription<sensor_msgs::msg::Image>(
    "/mapping_test/rectified", rclcpp::SensorDataQoS(),
    [&received_rectified](sensor_msgs::msg::Image::SharedPtr message) {
      received_rectified = std::move(message);
    });
  auto pose_subscription = driver->create_subscription<auv_interfaces::msg::GridPose>(
    "/mapping_test/pose", rclcpp::SensorDataQoS(),
    [&received_pose](auv_interfaces::msg::GridPose::SharedPtr message) {
      received_pose = std::move(message);
    });
  (void)map_subscription;
  (void)image_subscription;
  (void)pose_subscription;

  std_msgs::msg::Header header;
  header.frame_id = "camera_down_optical_frame";
  const auto image = cv_bridge::CvImage(
    header, sensor_msgs::image_encodings::BGR8, make_node_test_grid()).toImageMsg();
  auv_interfaces::msg::ConeDetectionArray cones;
  cones.header = header;
  cones.stable = false;
  auv_interfaces::msg::ConeDetection cone;
  cone.row = 1;
  cone.col = 2;
  cone.shape = auv_interfaces::msg::ConeDetection::SHAPE_CIRCLE;
  cone.detection.confidence = 0.91F;
  cones.detections.push_back(cone);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mapper);
  executor.add_node(driver);
  for (std::size_t attempt = 0U; attempt < 100U && !received_map; ++attempt) {
    cone_publisher->publish(cones);
    image_publisher->publish(*image);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(received_map, nullptr);
  EXPECT_FALSE(received_map->complete);
  received_map.reset();
  cones.stable = true;
  for (std::size_t attempt = 0U;
    attempt < 100U && (!received_map || !received_rectified || !received_pose); ++attempt)
  {
    cone_publisher->publish(cones);
    image_publisher->publish(*image);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_NE(received_map, nullptr);
  EXPECT_TRUE(received_map->complete);
  EXPECT_EQ(received_map->rows, 3U);
  EXPECT_EQ(received_map->cols, 3U);
  ASSERT_EQ(received_map->cells.size(), 9U);
  for (std::size_t index = 0U; index < received_map->cells.size(); ++index) {
    EXPECT_EQ(received_map->cells[index].row, static_cast<int8_t>(index / 3U));
    EXPECT_EQ(received_map->cells[index].col, static_cast<int8_t>(index % 3U));
    const bool cone_cell = index == 5U;
    EXPECT_EQ(
      received_map->cells[index].object_type,
      cone_cell ? "circle_cone" : "unknown");
    if (cone_cell) {
      EXPECT_FLOAT_EQ(received_map->cells[index].confidence, 0.91F);
    }
    EXPECT_FALSE(received_map->cells[index].visited);
  }
  ASSERT_NE(received_rectified, nullptr);
  EXPECT_EQ(received_rectified->width, 600U);
  EXPECT_EQ(received_rectified->height, 600U);
  EXPECT_EQ(received_rectified->header.frame_id, "camera_down_optical_frame");
  ASSERT_NE(received_pose, nullptr);
  EXPECT_TRUE(received_pose->valid);
  EXPECT_GE(received_pose->row, 0.0F);
  EXPECT_LE(received_pose->row, 3.0F);
  EXPECT_GE(received_pose->col, 0.0F);
  EXPECT_LE(received_pose->col, 3.0F);

  auv_interfaces::msg::GridCell visited;
  visited.row = 1;
  visited.col = 2;
  visited_publisher->publish(visited);
  for (std::size_t attempt = 0U;
    attempt < 100U && !received_map->cells[5].visited; ++attempt)
  {
    image_publisher->publish(*image);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(received_map->cells[5].visited);

  executor.remove_node(driver);
  executor.remove_node(mapper);
  rclcpp::shutdown();
}

}  // namespace
