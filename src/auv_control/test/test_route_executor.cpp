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

#include <stdexcept>

#include "auv_control/route_executor.hpp"
#include "gtest/gtest.h"

namespace
{

auv_interfaces::msg::GridCell cell(std::int8_t row, std::int8_t col, const char * type = "unknown")
{
  auv_interfaces::msg::GridCell result;
  result.row = row;
  result.col = col;
  result.object_type = type;
  return result;
}

auv_interfaces::msg::PlannedRoute route()
{
  auv_interfaces::msg::PlannedRoute result;
  result.valid = true;
  result.map_revision = 7U;
  result.path = {cell(2, 0), cell(1, 0), cell(0, 0, "circle_cone")};
  result.targets = {cell(0, 0, "circle_cone")};
  return result;
}

TEST(RouteExecutor, RequiresMissionArmPoseAndRoute)
{
  auv_control::RouteExecutor executor;
  EXPECT_EQ(executor.step().state, auv_control::RouteStep::State::kIdle);
  executor.set_mission_active(true);
  EXPECT_EQ(executor.step().state, auv_control::RouteStep::State::kFault);
  executor.set_route(route());
  EXPECT_EQ(executor.step().state, auv_control::RouteStep::State::kWaitingForArm);
  executor.set_vehicle_ready(true);
  EXPECT_EQ(executor.step().state, auv_control::RouteStep::State::kFault);
}

TEST(RouteExecutor, TraversesWaypointsAndMarksTargetVisited)
{
  auv_control::RouteExecutorConfig config;
  config.arrival_stable_ticks = 2;
  auv_control::RouteExecutor executor(config);
  executor.set_route(route());
  executor.set_mission_active(true);
  executor.set_vehicle_ready(true);

  executor.set_pose(true, 2.5, 0.5);
  EXPECT_EQ(executor.step().state, auv_control::RouteStep::State::kRunning);
  EXPECT_EQ(executor.step().waypoint_index, 1U);
  executor.set_pose(true, 1.5, 0.5);
  (void)executor.step();
  EXPECT_EQ(executor.step().waypoint_index, 2U);
  executor.set_pose(true, 0.5, 0.5);
  (void)executor.step();
  const auto completed = executor.step();
  EXPECT_EQ(completed.state, auv_control::RouteStep::State::kComplete);
  ASSERT_TRUE(completed.visited_cell.has_value());
  EXPECT_EQ(completed.visited_cell->row, 0);
  EXPECT_EQ(completed.visited_cell->col, 0);
}

TEST(RouteExecutor, ProducesBoundedGridErrorCommands)
{
  auv_control::RouteExecutor executor;
  executor.set_route(route());
  executor.set_mission_active(true);
  executor.set_vehicle_ready(true);
  executor.set_pose(true, 0.0, 2.9);
  const auto output = executor.step();
  EXPECT_EQ(output.state, auv_control::RouteStep::State::kRunning);
  EXPECT_LE(std::abs(output.surge), 0.20);
  EXPECT_LE(std::abs(output.sway), 0.20);
}

TEST(RouteExecutor, ValidatesConfiguration)
{
  auv_control::RouteExecutorConfig config;
  config.maximum_speed = 0.0;
  EXPECT_THROW({auv_control::RouteExecutor executor(config);}, std::invalid_argument);
}

}  // namespace
