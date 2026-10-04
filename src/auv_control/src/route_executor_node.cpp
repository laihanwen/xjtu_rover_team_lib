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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "auv_control/route_executor.hpp"
#include "auv_control/route_executor_node.hpp"
#include "auv_interfaces/msg/grid_cell.hpp"
#include "auv_interfaces/msg/grid_pose.hpp"
#include "auv_interfaces/msg/mission_state.hpp"
#include "auv_interfaces/msg/planned_route.hpp"
#include "auv_interfaces/msg/route_execution_state.hpp"
#include "auv_interfaces/msg/stm32_status.hpp"
#include "auv_interfaces/srv/set_armed.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace auv_control
{

static auv_planning::PlanResult to_core_route(const auv_interfaces::msg::PlannedRoute & route)
{
  auv_planning::PlanResult result;
  result.valid = route.valid;
  result.reason = route.reason;
  result.start = {route.start.row, route.start.col, route.start.object_type};
  result.total_cost = route.total_cost;
  for (const auto & cell : route.targets) {
    result.targets.push_back({cell.row, cell.col, cell.object_type});
  }
  for (const auto & cell : route.path) {
    result.path.push_back({cell.row, cell.col, cell.object_type});
  }
  return result;
}

class RouteExecutorNode final : public rclcpp::Node
{
public:
  explicit RouteExecutorNode(const rclcpp::NodeOptions & options)
  : Node("auv_route_executor", options)
  {
    RouteExecutorConfig config;
    config.surge_from_row = declare_parameter<double>("surge_from_row", 0.25);
    config.surge_from_col = declare_parameter<double>("surge_from_col", 0.0);
    config.sway_from_row = declare_parameter<double>("sway_from_row", 0.0);
    config.sway_from_col = declare_parameter<double>("sway_from_col", 0.25);
    config.maximum_speed = declare_parameter<double>("maximum_speed", 0.20);
    config.arrival_tolerance = declare_parameter<double>("arrival_tolerance", 0.15);
    config.arrival_stable_ticks = declare_parameter<int>("arrival_stable_ticks", 5);
    executor_ = std::make_unique<RouteExecutor>(config);

    publish_motion_commands_ = declare_parameter<bool>("publish_motion_commands", false);
    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 20.0);
    pose_timeout_sec_ = declare_parameter<double>("pose_timeout_sec", 0.5);
    status_timeout_sec_ = declare_parameter<double>("status_timeout_sec", 0.5);
    if (control_rate_hz <= 0.0 || pose_timeout_sec_ <= 0.0 || status_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("route executor rates and timeouts must be positive");
    }

    const auto command_qos = rclcpp::QoS(1).reliable();
    velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", command_qos);
    depth_publisher_ = create_publisher<std_msgs::msg::Float32>("/cmd_depth", command_qos);
    yaw_publisher_ = create_publisher<std_msgs::msg::Float32>("/cmd_yaw", command_qos);
    visited_publisher_ = create_publisher<auv_interfaces::msg::GridCell>(
      "/planning/visited_cell", rclcpp::QoS(10).reliable());
    state_publisher_ = create_publisher<auv_interfaces::msg::RouteExecutionState>(
      "/planning/execution_state", rclcpp::QoS(10).reliable().transient_local());
    disarm_client_ = create_client<auv_interfaces::srv::SetArmed>("/stm32/set_armed");

    route_subscription_ = create_subscription<auv_interfaces::msg::PlannedRoute>(
      "/planning/route", rclcpp::QoS(1).reliable().transient_local(),
      [this](auv_interfaces::msg::PlannedRoute::ConstSharedPtr message) {
        if (mission_active_ && route_.valid) {
          return;
        }
        route_revision_ = message->map_revision;
        route_ = *message;
        executor_->set_route(to_core_route(*message), message->map_revision);
      });
    pose_subscription_ = create_subscription<auv_interfaces::msg::GridPose>(
      "/mapping/grid_pose", rclcpp::SensorDataQoS(),
      [this](auv_interfaces::msg::GridPose::ConstSharedPtr message) {
        pose_ = *message;
        pose_received_ = now();
      });
    status_subscription_ = create_subscription<auv_interfaces::msg::Stm32Status>(
      "/stm32/status", rclcpp::SensorDataQoS(),
      [this](auv_interfaces::msg::Stm32Status::ConstSharedPtr message) {
        status_ = *message;
        status_received_ = now();
      });
    mission_subscription_ = create_subscription<auv_interfaces::msg::MissionState>(
      "/mission/state", rclcpp::QoS(10).reliable().transient_local(),
      [this](auv_interfaces::msg::MissionState::ConstSharedPtr message) {
        const bool was_active = mission_active_;
        mission_active_ = message->state == "VISIT_CONES";
        if (!mission_active_ && was_active) {
          executor_->reset();
          if (route_.valid) {
            executor_->set_route(to_core_route(route_), route_.map_revision);
          }
        }
        executor_->set_mission_active(mission_active_);
      });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_rate_hz)),
      [this]() {control_tick();});

    RCLCPP_INFO(
      get_logger(), "route executor ready: motion publishing=%s",
      publish_motion_commands_ ? "ENABLED" : "disabled (dry-run)");
  }

private:
  void publish_motion(double surge, double sway)
  {
    if (!publish_motion_commands_) {
      return;
    }
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = surge;
    velocity.linear.y = sway;
    std_msgs::msg::Float32 depth;
    depth.data = hold_targets_valid_ ? hold_depth_ : status_.depth;
    std_msgs::msg::Float32 yaw;
    yaw.data = hold_targets_valid_ ? hold_yaw_ : status_.yaw;
    velocity_publisher_->publish(velocity);
    depth_publisher_->publish(depth);
    yaw_publisher_->publish(yaw);
  }

  void request_disarm_once()
  {
    if (disarm_requested_ || !status_.armed || !disarm_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<auv_interfaces::srv::SetArmed::Request>();
    request->armed = false;
    disarm_requested_ = true;
    (void)disarm_client_->async_send_request(
      request,
      [this](rclcpp::Client<auv_interfaces::srv::SetArmed>::SharedFuture future) {
        if (!future.get()->accepted) {
          disarm_requested_ = false;
        }
      });
  }

  void control_tick()
  {
    const auto current_time = now();
    const bool pose_fresh = pose_received_.nanoseconds() > 0 &&
      (current_time - pose_received_).seconds() <= pose_timeout_sec_;
    const bool status_fresh = status_received_.nanoseconds() > 0 &&
      (current_time - status_received_).seconds() <= status_timeout_sec_;
    const bool telemetry_finite = std::isfinite(status_.depth) && std::isfinite(status_.yaw);
    const bool vehicle_ready = status_fresh && status_.connected && status_.armed &&
      !status_.leak_detected && status_.error_flags == 0U && telemetry_finite;
    executor_->set_vehicle_ready(vehicle_ready);
    executor_->set_pose(pose_fresh && pose_.valid, pose_.row, pose_.col);

    const RouteStep step = executor_->step();
    if (step.state == RouteStep::State::kRunning) {
      if (!hold_targets_valid_) {
        hold_depth_ = status_.depth;
        hold_yaw_ = status_.yaw;
        hold_targets_valid_ = true;
      }
      disarm_requested_ = false;
      publish_motion(step.surge, step.sway);
    } else {
      publish_motion(0.0, 0.0);
      if (!vehicle_ready) {
        hold_targets_valid_ = false;
      }
      if (status_.armed) {
        request_disarm_once();
      }
    }
    if (step.visited_cell.has_value()) {
      auv_interfaces::msg::GridCell visited;
      visited.row = step.visited_cell->row;
      visited.col = step.visited_cell->col;
      visited.object_type = step.visited_cell->object_type;
      visited_publisher_->publish(visited);
    }

    auv_interfaces::msg::RouteExecutionState state;
    state.header.stamp = current_time;
    state.header.frame_id = "grid";
    state.state = static_cast<std::uint8_t>(step.state);
    state.detail = publish_motion_commands_ ? step.detail : "dry-run: " + step.detail;
    state.route_revision = route_revision_;
    state.waypoint_index = static_cast<std::uint32_t>(step.waypoint_index);
    if (route_.valid && step.waypoint_index < route_.path.size()) {
      state.target_row = route_.path[step.waypoint_index].row;
      state.target_col = route_.path[step.waypoint_index].col;
    } else {
      state.target_row = -1;
      state.target_col = -1;
    }
    state.row_error = static_cast<float>(step.row_error);
    state.col_error = static_cast<float>(step.col_error);
    state_publisher_->publish(state);
  }

  bool publish_motion_commands_{false};
  bool mission_active_{false};
  bool disarm_requested_{false};
  bool hold_targets_valid_{false};
  double pose_timeout_sec_{0.5};
  double status_timeout_sec_{0.5};
  std::uint32_t route_revision_{0U};
  float hold_depth_{0.0F};
  float hold_yaw_{0.0F};
  rclcpp::Time pose_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time status_received_{0, 0, RCL_ROS_TIME};
  auv_interfaces::msg::GridPose pose_;
  auv_interfaces::msg::Stm32Status status_;
  auv_interfaces::msg::PlannedRoute route_;
  std::unique_ptr<RouteExecutor> executor_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_publisher_;
  rclcpp::Publisher<auv_interfaces::msg::GridCell>::SharedPtr visited_publisher_;
  rclcpp::Publisher<auv_interfaces::msg::RouteExecutionState>::SharedPtr state_publisher_;
  rclcpp::Client<auv_interfaces::srv::SetArmed>::SharedPtr disarm_client_;
  rclcpp::Subscription<auv_interfaces::msg::PlannedRoute>::SharedPtr route_subscription_;
  rclcpp::Subscription<auv_interfaces::msg::GridPose>::SharedPtr pose_subscription_;
  rclcpp::Subscription<auv_interfaces::msg::Stm32Status>::SharedPtr status_subscription_;
  rclcpp::Subscription<auv_interfaces::msg::MissionState>::SharedPtr mission_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace auv_control

std::shared_ptr<rclcpp::Node> auv_control::make_route_executor_node(
  const rclcpp::NodeOptions & options)
{
  return std::make_shared<RouteExecutorNode>(options);
}
