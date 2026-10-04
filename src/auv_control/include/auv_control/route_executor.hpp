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

#ifndef AUV_CONTROL__ROUTE_EXECUTOR_HPP_
#define AUV_CONTROL__ROUTE_EXECUTOR_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "auv_planning/grid_planner.hpp"

namespace auv_control
{

struct RouteExecutorConfig
{
  double surge_from_row{0.25};
  double surge_from_col{0.0};
  double sway_from_row{0.0};
  double sway_from_col{0.25};
  double maximum_speed{0.20};
  double arrival_tolerance{0.15};
  int arrival_stable_ticks{5};
};

struct RouteStep
{
  enum class State : std::uint8_t {kIdle, kWaitingForArm, kRunning, kComplete, kFault};
  State state{State::kIdle};
  std::string detail{"idle"};
  double surge{0.0};
  double sway{0.0};
  double row_error{0.0};
  double col_error{0.0};
  std::size_t waypoint_index{0U};
  std::optional<auv_planning::GridCell> visited_cell;
};

class RouteExecutor
{
public:
  explicit RouteExecutor(RouteExecutorConfig config = RouteExecutorConfig());

  void set_route(const auv_planning::PlanResult & route, std::uint32_t map_revision);
  void set_mission_active(bool active);
  void set_vehicle_ready(bool ready);
  void set_pose(bool valid, double row, double col);
  RouteStep step();
  void reset();

private:
  bool is_target(const auv_planning::GridCell & cell) const;

  RouteExecutorConfig config_;
  auv_planning::PlanResult route_;
  std::uint32_t map_revision_{0U};
  bool route_ready_{false};
  bool mission_active_{false};
  bool vehicle_ready_{false};
  bool pose_valid_{false};
  double row_{0.0};
  double col_{0.0};
  std::size_t waypoint_index_{0U};
  int arrival_ticks_{0};
};

}  // namespace auv_control

#endif  // AUV_CONTROL__ROUTE_EXECUTOR_HPP_
