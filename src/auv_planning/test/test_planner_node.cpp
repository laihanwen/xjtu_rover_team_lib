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
#include <cstddef>
#include <memory>
#include <thread>
#include <utility>

#include "auv_interfaces/msg/planned_route.hpp"
#include "auv_interfaces/msg/semantic_map.hpp"
#include "auv_planning/planner_node.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

namespace
{

auv_interfaces::msg::SemanticMap make_map()
{
  auv_interfaces::msg::SemanticMap map;
  map.header.frame_id = "grid";
  map.rows = 3U;
  map.cols = 3U;
  map.complete = true;
  for (std::int8_t row = 0; row < 3; ++row) {
    for (std::int8_t col = 0; col < 3; ++col) {
      auv_interfaces::msg::SemanticCell cell;
      cell.row = row;
      cell.col = col;
      cell.object_type = "unknown";
      map.cells.push_back(cell);
    }
  }
  map.cells[0].object_type = "circle_cone";
  return map;
}

TEST(PlannerNode, PublishesRouteAndOnlyReplansChangedMaps)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("map_topic", "/planner_test/map"),
      rclcpp::Parameter("route_topic", "/planner_test/route"),
      rclcpp::Parameter("start_row", 2),
      rclcpp::Parameter("start_col", 2)});
  auto planner = auv_planning::make_planner_node(options);
  auto driver = std::make_shared<rclcpp::Node>("planner_test_driver");
  auto publisher = driver->create_publisher<auv_interfaces::msg::SemanticMap>(
    "/planner_test/map", rclcpp::QoS(1).reliable().transient_local());
  auv_interfaces::msg::PlannedRoute::SharedPtr received;
  std::size_t received_count = 0U;
  auto subscription = driver->create_subscription<auv_interfaces::msg::PlannedRoute>(
    "/planner_test/route", rclcpp::QoS(1).reliable().transient_local(),
    [&received, &received_count](auv_interfaces::msg::PlannedRoute::SharedPtr message) {
      received = std::move(message);
      ++received_count;
    });
  (void)subscription;

  auto map = make_map();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(driver);
  for (std::size_t attempt = 0U; attempt < 100U && !received; ++attempt) {
    publisher->publish(map);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(received, nullptr);
  EXPECT_TRUE(received->valid) << received->reason;
  EXPECT_EQ(received->map_revision, 1U);
  EXPECT_EQ(received->header.frame_id, "grid");
  ASSERT_EQ(received->targets.size(), 1U);
  EXPECT_EQ(received->targets[0].row, 0);
  EXPECT_EQ(received->targets[0].col, 0);
  EXPECT_FLOAT_EQ(received->total_cost, 4.0F);

  const std::size_t before_duplicate = received_count;
  publisher->publish(map);
  for (int attempt = 0; attempt < 10; ++attempt) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(received_count, before_duplicate);

  map.cells[0].visited = true;
  publisher->publish(map);
  for (std::size_t attempt = 0U; attempt < 100U && received->map_revision < 2U; ++attempt) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(received->map_revision, 2U);
  EXPECT_TRUE(received->targets.empty());
  EXPECT_FLOAT_EQ(received->total_cost, 0.0F);

  executor.remove_node(driver);
  executor.remove_node(planner);
  rclcpp::shutdown();
}

}  // namespace
