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

#include "auv_control/route_executor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace auv_control
{

RouteExecutor::RouteExecutor(RouteExecutorConfig config)
: config_(std::move(config))
{
  if (config_.maximum_speed <= 0.0 || config_.arrival_tolerance <= 0.0 ||
    config_.arrival_tolerance >= 0.5 || config_.arrival_stable_ticks <= 0)
  {
    throw std::invalid_argument("route executor configuration is invalid");
  }
}

void RouteExecutor::set_route(const auv_interfaces::msg::PlannedRoute & route)
{
  if (mission_active_ && route_ready_) {
    return;
  }
  if (route.map_revision == route_.map_revision && route_ready_) {
    return;
  }
  route_ = route;
  route_ready_ = route.valid && !route.path.empty();
  waypoint_index_ = 0U;
  arrival_ticks_ = 0;
}

void RouteExecutor::set_mission_active(bool active) {mission_active_ = active;}
void RouteExecutor::set_vehicle_ready(bool ready) {vehicle_ready_ = ready;}

void RouteExecutor::set_pose(bool valid, double row, double col)
{
  pose_valid_ = valid && std::isfinite(row) && std::isfinite(col);
  row_ = row;
  col_ = col;
}

bool RouteExecutor::is_target(const auv_interfaces::msg::GridCell & cell) const
{
  return std::any_of(route_.targets.begin(), route_.targets.end(), [&cell](const auto & target) {
             return target.row == cell.row && target.col == cell.col;
  });
}

RouteStep RouteExecutor::step()
{
  RouteStep output;
  output.waypoint_index = waypoint_index_;
  if (!mission_active_) {
    output.detail = "mission is not in VISIT_CONES";
    return output;
  }
  if (!route_ready_) {
    output.state = RouteStep::State::kFault;
    output.detail = "no valid route";
    return output;
  }
  if (waypoint_index_ >= route_.path.size()) {
    output.state = RouteStep::State::kComplete;
    output.detail = "route complete";
    return output;
  }
  if (!vehicle_ready_) {
    output.state = RouteStep::State::kWaitingForArm;
    output.detail = "waiting for safe armed STM32";
    return output;
  }
  if (!pose_valid_) {
    output.state = RouteStep::State::kFault;
    output.detail = "grid pose unavailable";
    return output;
  }

  const auto & waypoint = route_.path[waypoint_index_];
  output.row_error = static_cast<double>(waypoint.row) + 0.5 - row_;
  output.col_error = static_cast<double>(waypoint.col) + 0.5 - col_;
  if (std::abs(output.row_error) <= config_.arrival_tolerance &&
    std::abs(output.col_error) <= config_.arrival_tolerance)
  {
    ++arrival_ticks_;
    if (arrival_ticks_ >= config_.arrival_stable_ticks) {
      if (is_target(waypoint)) {
        output.visited_cell = waypoint;
      }
      ++waypoint_index_;
      arrival_ticks_ = 0;
      output.waypoint_index = waypoint_index_;
      if (waypoint_index_ >= route_.path.size()) {
        output.state = RouteStep::State::kComplete;
        output.detail = "route complete";
        return output;
      }
    }
  } else {
    arrival_ticks_ = 0;
  }

  output.state = RouteStep::State::kRunning;
  output.detail = "tracking grid waypoint";
  output.surge = std::clamp(
    config_.surge_from_row * output.row_error + config_.surge_from_col * output.col_error,
    -config_.maximum_speed, config_.maximum_speed);
  output.sway = std::clamp(
    config_.sway_from_row * output.row_error + config_.sway_from_col * output.col_error,
    -config_.maximum_speed, config_.maximum_speed);
  return output;
}

void RouteExecutor::reset()
{
  route_ready_ = false;
  mission_active_ = false;
  vehicle_ready_ = false;
  pose_valid_ = false;
  waypoint_index_ = 0U;
  arrival_ticks_ = 0;
}

}  // namespace auv_control
