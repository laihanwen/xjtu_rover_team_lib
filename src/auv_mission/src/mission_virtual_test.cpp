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
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auv_interfaces/msg/april_tag_detection_array.hpp"
#include "auv_interfaces/msg/mission_state.hpp"
#include "auv_interfaces/msg/planned_route.hpp"
#include "auv_interfaces/msg/semantic_map.hpp"
#include "auv_interfaces/msg/stm32_status.hpp"
#include "auv_interfaces/srv/mission_command.hpp"
#include "auv_mission/mission_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

auv_interfaces::msg::SemanticMap make_map(bool visited)
{
  auv_interfaces::msg::SemanticMap map;
  map.header.frame_id = "virtual_grid";
  map.rows = 1U;
  map.cols = 1U;
  map.complete = true;
  auv_interfaces::msg::SemanticCell cell;
  cell.row = 0;
  cell.col = 0;
  cell.object_type = "circle_cone";
  cell.confidence = 1.0F;
  cell.visited = visited;
  map.cells.push_back(cell);
  return map;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("state_topic", "/virtual/mission/state"),
    rclcpp::Parameter("command_service", "/virtual/mission/command"),
    rclcpp::Parameter("status_topic", "/virtual/stm32/status"),
    rclcpp::Parameter("apriltag_topic", "/virtual/apriltag/detections"),
    rclcpp::Parameter("map_topic", "/virtual/semantic_map"),
    rclcpp::Parameter("route_topic", "/virtual/planning/route"),
    rclcpp::Parameter("tick_period_ms", 20),
    rclcpp::Parameter("status_timeout_sec", 0.5)});
  auto mission = auv_mission::make_mission_node(options);
  auto driver = std::make_shared<rclcpp::Node>("mission_virtual_driver");

  auto status_publisher = driver->create_publisher<auv_interfaces::msg::Stm32Status>(
    "/virtual/stm32/status", rclcpp::SensorDataQoS());
  auto tag_publisher = driver->create_publisher<auv_interfaces::msg::AprilTagDetectionArray>(
    "/virtual/apriltag/detections", rclcpp::SensorDataQoS());
  auto map_publisher = driver->create_publisher<auv_interfaces::msg::SemanticMap>(
    "/virtual/semantic_map", rclcpp::QoS(1).reliable().transient_local());
  auto route_publisher = driver->create_publisher<auv_interfaces::msg::PlannedRoute>(
    "/virtual/planning/route", rclcpp::QoS(1).reliable().transient_local());
  auto command_client = driver->create_client<auv_interfaces::srv::MissionCommand>(
    "/virtual/mission/command");

  std::string state;
  std::vector<std::string> states;
  auto state_subscription = driver->create_subscription<auv_interfaces::msg::MissionState>(
    "/virtual/mission/state", rclcpp::QoS(10).reliable().transient_local(),
    [&state, &states](auv_interfaces::msg::MissionState::ConstSharedPtr message) {
      state = message->state;
      if (states.empty() || states.back() != state) {
        states.push_back(state);
        std::cout << "VIRTUAL_MISSION " << message->previous_state << " -> " <<
          message->state << ": " << message->detail << std::endl;
      }
    });
  (void)state_subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mission);
  executor.add_node(driver);
  bool start_sent = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline && state != "COMPLETE") {
    auv_interfaces::msg::Stm32Status status;
    status.connected = true;
    status.armed = false;
    status.leak_detected = false;
    status.error_flags = 0U;
    status_publisher->publish(status);

    if (!start_sent && command_client->service_is_ready()) {
      auto request = std::make_shared<auv_interfaces::srv::MissionCommand::Request>();
      request->command = auv_interfaces::srv::MissionCommand::Request::START;
      (void)command_client->async_send_request(request);
      start_sent = true;
    } else if (state == "SEARCH_APRILTAG") {
      auv_interfaces::msg::AprilTagDetectionArray detections;
      detections.detections.resize(1U);
      detections.detections[0].id = 23;
      tag_publisher->publish(detections);
    } else if (state == "BUILD_MAP") {
      map_publisher->publish(make_map(false));
    } else if (state == "PLAN_CONES") {
      auv_interfaces::msg::PlannedRoute route;
      route.valid = true;
      route.reason = "virtual route";
      route.targets.resize(1U);
      route.targets[0].row = 0;
      route.targets[0].col = 0;
      route.targets[0].object_type = "circle_cone";
      route.path = route.targets;
      route_publisher->publish(route);
    } else if (state == "VISIT_CONES") {
      map_publisher->publish(make_map(true));
    }
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  const std::vector<std::string> expected{
    "INIT", "SELF_CHECK", "SEARCH_APRILTAG", "BUILD_MAP", "PLAN_CONES",
    "VISIT_CONES", "COMPLETE"};
  const bool passed = states == expected;
  std::cout << (passed ? "VIRTUAL_MISSION_RESULT=PASS" : "VIRTUAL_MISSION_RESULT=FAIL") <<
    std::endl;
  executor.remove_node(driver);
  executor.remove_node(mission);
  rclcpp::shutdown();
  return passed ? 0 : 1;
}
