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
#include <cmath>
#include <cstddef>
#include <memory>
#include <thread>

#include "auv_control/route_executor_node.hpp"
#include "auv_interfaces/msg/grid_cell.hpp"
#include "auv_interfaces/msg/grid_pose.hpp"
#include "auv_interfaces/msg/mission_state.hpp"
#include "auv_interfaces/msg/planned_route.hpp"
#include "auv_interfaces/msg/stm32_status.hpp"
#include "auv_interfaces/srv/set_armed.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

namespace
{

TEST(RouteExecutorNode, PublishesMotionVisitsTargetAndRequestsDisarm)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("publish_motion_commands", true),
      rclcpp::Parameter("control_rate_hz", 50.0),
      rclcpp::Parameter("arrival_stable_ticks", 1),
      rclcpp::Parameter("pose_timeout_sec", 2.0),
      rclcpp::Parameter("status_timeout_sec", 2.0)});
  auto executor_node = auv_control::make_route_executor_node(options);
  auto driver = std::make_shared<rclcpp::Node>("route_executor_test_driver");

  const auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
  auto route_publisher = driver->create_publisher<auv_interfaces::msg::PlannedRoute>(
    "/planning/route", transient_qos);
  auto mission_publisher = driver->create_publisher<auv_interfaces::msg::MissionState>(
    "/mission/state", rclcpp::QoS(10).reliable().transient_local());
  auto pose_publisher = driver->create_publisher<auv_interfaces::msg::GridPose>(
    "/mapping/grid_pose", rclcpp::SensorDataQoS());
  auto status_publisher = driver->create_publisher<auv_interfaces::msg::Stm32Status>(
    "/stm32/status", rclcpp::SensorDataQoS());

  geometry_msgs::msg::Twist::SharedPtr velocity;
  auv_interfaces::msg::GridCell::SharedPtr visited;
  bool disarm_requested = false;
  auto velocity_subscription = driver->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(1).reliable(),
    [&velocity](geometry_msgs::msg::Twist::SharedPtr message) {
      velocity = std::move(message);
    });
  auto visited_subscription = driver->create_subscription<auv_interfaces::msg::GridCell>(
    "/planning/visited_cell", rclcpp::QoS(10).reliable(),
    [&visited](auv_interfaces::msg::GridCell::SharedPtr message) {
      visited = std::move(message);
    });
  auto disarm_service = driver->create_service<auv_interfaces::srv::SetArmed>(
    "/stm32/set_armed",
    [&disarm_requested](
      auv_interfaces::srv::SetArmed::Request::SharedPtr request,
      auv_interfaces::srv::SetArmed::Response::SharedPtr response)
    {
      disarm_requested = !request->armed;
      response->accepted = disarm_requested;
      response->message = "test";
    });
  (void)velocity_subscription;
  (void)visited_subscription;
  (void)disarm_service;

  auv_interfaces::msg::PlannedRoute route;
  route.valid = true;
  route.map_revision = 1U;
  auv_interfaces::msg::GridCell target;
  target.row = 0;
  target.col = 0;
  target.object_type = "circle_cone";
  route.path.push_back(target);
  route.targets.push_back(target);
  auv_interfaces::msg::MissionState mission;
  mission.state = "VISIT_CONES";
  auv_interfaces::msg::Stm32Status status;
  status.connected = true;
  status.armed = true;
  status.depth = 1.2F;
  status.yaw = 0.4F;
  auv_interfaces::msg::GridPose pose;
  pose.valid = true;
  pose.row = 1.5F;
  pose.col = 0.5F;

  rclcpp::executors::SingleThreadedExecutor ros_executor;
  ros_executor.add_node(executor_node);
  ros_executor.add_node(driver);
  for (std::size_t attempt = 0U; attempt < 100U && !velocity; ++attempt) {
    route_publisher->publish(route);
    mission_publisher->publish(mission);
    status_publisher->publish(status);
    pose_publisher->publish(pose);
    ros_executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(velocity, nullptr);
  EXPECT_TRUE(std::isfinite(velocity->linear.x));
  EXPECT_NE(velocity->linear.x, 0.0);

  pose.row = 0.5F;
  for (std::size_t attempt = 0U;
    attempt < 100U && (!visited || !disarm_requested); ++attempt)
  {
    status_publisher->publish(status);
    pose_publisher->publish(pose);
    ros_executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(visited, nullptr);
  EXPECT_EQ(visited->row, 0);
  EXPECT_EQ(visited->col, 0);
  EXPECT_TRUE(disarm_requested);

  ros_executor.remove_node(driver);
  ros_executor.remove_node(executor_node);
  rclcpp::shutdown();
}

}  // namespace
