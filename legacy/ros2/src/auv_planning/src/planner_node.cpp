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

#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "auv_interfaces/msg/grid_cell.hpp"
#include "auv_interfaces/msg/planned_route.hpp"
#include "auv_interfaces/msg/semantic_map.hpp"
#include "auv_planning/grid_planner.hpp"
#include "auv_planning/planner_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auv_planning
{
namespace
{

auv_interfaces::msg::GridCell to_message(const GridCell & cell)
{
  auv_interfaces::msg::GridCell message;
  message.row = cell.row;
  message.col = cell.col;
  message.object_type = cell.object_type;
  return message;
}

std::string map_signature(const auv_interfaces::msg::SemanticMap & message)
{
  std::ostringstream stream;
  stream << static_cast<int>(message.rows) << ':' << static_cast<int>(message.cols) << ':' <<
    message.complete;
  for (const auto & cell : message.cells) {
    stream << '|' << static_cast<int>(cell.row) << ',' << static_cast<int>(cell.col) << ',' <<
      cell.object_type << ',' << cell.visited;
  }
  return stream.str();
}

}  // namespace

class PlannerNode final : public rclcpp::Node
{
public:
  explicit PlannerNode(const rclcpp::NodeOptions & options)
  : Node("auv_planner", options)
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/semantic_map");
    route_topic_ = declare_parameter<std::string>("route_topic", "/planning/route");
    start_row_ = declare_parameter<int>("start_row", -1);
    start_col_ = declare_parameter<int>("start_col", -1);
    GridPlannerConfig config;
    config.target_object_types = declare_parameter<std::vector<std::string>>(
      "target_object_types", {"circle_cone", "square_cone"});
    config.blocked_object_types = declare_parameter<std::vector<std::string>>(
      "blocked_object_types", {"obstacle", "forbidden"});
    const int maximum_targets = declare_parameter<int>("maximum_targets", 8);
    if (map_topic_.empty() || route_topic_.empty()) {
      throw std::invalid_argument("planner topic names must not be empty");
    }
    if (maximum_targets <= 0) {
      throw std::invalid_argument("maximum_targets must be positive");
    }
    config.maximum_targets = static_cast<std::size_t>(maximum_targets);
    planner_ = std::make_unique<GridPlanner>(config);

    route_publisher_ = create_publisher<auv_interfaces::msg::PlannedRoute>(
      route_topic_, rclcpp::QoS(1).reliable().transient_local());
    map_subscription_ = create_subscription<auv_interfaces::msg::SemanticMap>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](auv_interfaces::msg::SemanticMap::ConstSharedPtr message) {
        process_map(*message);
      });
    RCLCPP_INFO(
      get_logger(), "planner ready: map=%s route=%s start=(%d,%d)",
      map_topic_.c_str(), route_topic_.c_str(), start_row_, start_col_);
  }

private:
  void process_map(const auv_interfaces::msg::SemanticMap & message)
  {
    const std::string signature = map_signature(message);
    if (signature == last_map_signature_) {
      return;
    }
    last_map_signature_ = signature;
    ++map_revision_;

    PlanningGrid grid;
    grid.rows = message.rows;
    grid.cols = message.cols;
    grid.complete = message.complete;
    grid.cells.reserve(message.cells.size());
    for (const auto & cell : message.cells) {
      grid.cells.push_back({
          {cell.row, cell.col, cell.object_type}, cell.visited});
    }
    const PlanResult result = planner_->plan(
      grid,
      {static_cast<std::int8_t>(start_row_), static_cast<std::int8_t>(start_col_), "unknown"});

    auv_interfaces::msg::PlannedRoute route;
    route.header = message.header;
    route.valid = result.valid;
    route.reason = result.reason;
    route.start = to_message(result.start);
    route.total_cost = result.total_cost;
    route.map_revision = map_revision_;
    route.targets.reserve(result.targets.size());
    for (const auto & target : result.targets) {
      route.targets.push_back(to_message(target));
    }
    route.path.reserve(result.path.size());
    for (const auto & cell : result.path) {
      route.path.push_back(to_message(cell));
    }
    route_publisher_->publish(route);
  }

  std::string map_topic_;
  std::string route_topic_;
  std::string last_map_signature_;
  int start_row_{-1};
  int start_col_{-1};
  std::uint32_t map_revision_{0U};
  std::unique_ptr<GridPlanner> planner_;
  rclcpp::Publisher<auv_interfaces::msg::PlannedRoute>::SharedPtr route_publisher_;
  rclcpp::Subscription<auv_interfaces::msg::SemanticMap>::SharedPtr map_subscription_;
};

std::shared_ptr<rclcpp::Node> make_planner_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<PlannerNode>(options);
}

}  // namespace auv_planning
