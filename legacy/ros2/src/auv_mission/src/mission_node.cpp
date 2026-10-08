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
#include <cstdint>
#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include "auv_interfaces/msg/april_tag_detection_array.hpp"
#include "auv_interfaces/msg/mission_state.hpp"
#include "auv_interfaces/msg/planned_route.hpp"
#include "auv_interfaces/msg/semantic_map.hpp"
#include "auv_interfaces/msg/stm32_status.hpp"
#include "auv_interfaces/srv/mission_command.hpp"
#include "auv_mission/mission_fsm.hpp"
#include "auv_mission/mission_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auv_mission
{

class MissionNode final : public rclcpp::Node
{
public:
  explicit MissionNode(const rclcpp::NodeOptions & options)
  : Node("auv_mission_manager", options)
  {
    const auto state_topic = declare_parameter<std::string>("state_topic", "/mission/state");
    const auto command_service = declare_parameter<std::string>(
      "command_service", "/mission/command");
    const auto status_topic = declare_parameter<std::string>("status_topic", "/stm32/status");
    const auto apriltag_topic = declare_parameter<std::string>(
      "apriltag_topic", "/apriltag/detections");
    const auto map_topic = declare_parameter<std::string>("map_topic", "/semantic_map");
    const auto route_topic = declare_parameter<std::string>("route_topic", "/planning/route");
    const bool auto_start = declare_parameter<bool>("auto_start", false);
    const int tick_period_ms = declare_parameter<int>("tick_period_ms", 100);
    if (state_topic.empty() || command_service.empty() || status_topic.empty() ||
      apriltag_topic.empty() || map_topic.empty() || route_topic.empty() || tick_period_ms <= 0)
    {
      throw std::invalid_argument("mission topics and tick_period_ms must be valid");
    }

    MissionFsmConfig config;
    config.self_check_timeout_sec = declare_parameter<double>("self_check_timeout_sec", 10.0);
    config.apriltag_timeout_sec = declare_parameter<double>("apriltag_timeout_sec", 30.0);
    config.map_timeout_sec = declare_parameter<double>("map_timeout_sec", 30.0);
    config.planning_timeout_sec = declare_parameter<double>("planning_timeout_sec", 10.0);
    config.cone_visit_timeout_sec = declare_parameter<double>(
      "cone_visit_timeout_sec", 120.0);
    config.status_timeout_sec = declare_parameter<double>("status_timeout_sec", 1.0);
    config.allow_armed_during_visit = declare_parameter<bool>(
      "allow_armed_during_visit", false);
    fsm_ = std::make_unique<MissionFsm>(config);

    state_publisher_ = create_publisher<auv_interfaces::msg::MissionState>(
      state_topic, rclcpp::QoS(10).reliable().transient_local());
    status_subscription_ = create_subscription<auv_interfaces::msg::Stm32Status>(
      status_topic, rclcpp::SensorDataQoS(),
      [this](auv_interfaces::msg::Stm32Status::ConstSharedPtr message) {
        fsm_->update_status(
          message->connected, message->armed,
          message->error_flags, seconds_now());
      });
    apriltag_subscription_ =
      create_subscription<auv_interfaces::msg::AprilTagDetectionArray>(
      apriltag_topic, rclcpp::SensorDataQoS(),
      [this](auv_interfaces::msg::AprilTagDetectionArray::ConstSharedPtr message) {
        fsm_->update_apriltag(!message->detections.empty(), seconds_now());
      });
    map_subscription_ = create_subscription<auv_interfaces::msg::SemanticMap>(
      map_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](auv_interfaces::msg::SemanticMap::ConstSharedPtr message) {
        bool found_cone = false;
        bool all_cones_visited = true;
        for (const auto & cell : message->cells) {
          if (cell.object_type == "circle_cone" || cell.object_type == "square_cone") {
            found_cone = true;
            all_cones_visited = all_cones_visited && cell.visited;
          }
        }
        fsm_->update_map(
          message->complete, found_cone && all_cones_visited, seconds_now());
      });
    route_subscription_ = create_subscription<auv_interfaces::msg::PlannedRoute>(
      route_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](auv_interfaces::msg::PlannedRoute::ConstSharedPtr message) {
        fsm_->update_route(message->valid, !message->targets.empty(), seconds_now());
      });
    command_service_ = create_service<auv_interfaces::srv::MissionCommand>(
      command_service,
      [this](
        const auv_interfaces::srv::MissionCommand::Request::SharedPtr request,
        auv_interfaces::srv::MissionCommand::Response::SharedPtr response)
      {
        const auto command = decode_command(request->command);
        if (!command.has_value()) {
          response->accepted = false;
          response->message = "unknown mission command";
          return;
        }
        const CommandResult result = fsm_->command(command.value(), seconds_now());
        response->accepted = result.accepted;
        response->message = result.message;
        publish_if_changed();
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(tick_period_ms), [this]() {
        fsm_->tick(seconds_now());
        publish_if_changed();
      });

    publish_if_changed(true);
    if (auto_start) {
      (void)fsm_->command(MissionCommand::kStart, seconds_now());
      publish_if_changed();
    }
    RCLCPP_INFO(
      get_logger(), "mission manager ready: auto_start=%s propulsion commands=disabled",
      auto_start ? "true" : "false");
  }

private:
  std::optional<MissionCommand> decode_command(std::uint8_t command) const
  {
    using Service = auv_interfaces::srv::MissionCommand;
    switch (command) {
      case Service::Request::START:
        return MissionCommand::kStart;
      case Service::Request::PAUSE:
        return MissionCommand::kPause;
      case Service::Request::RESUME:
        return MissionCommand::kResume;
      case Service::Request::ABORT:
        return MissionCommand::kAbort;
      case Service::Request::RESET:
        return MissionCommand::kReset;
      default:
        return std::nullopt;
    }
  }

  double seconds_now() const
  {
    return now().seconds();
  }

  void publish_if_changed(bool force = false)
  {
    const MissionSnapshot snapshot = fsm_->snapshot();
    if (!force && snapshot.revision == published_revision_) {
      return;
    }
    auv_interfaces::msg::MissionState message;
    message.header.stamp = now();
    message.header.frame_id = "mission";
    message.state = mission_phase_name(snapshot.phase);
    message.previous_state = mission_phase_name(snapshot.previous_phase);
    message.detail = snapshot.detail;
    message.faulted = snapshot.faulted;
    state_publisher_->publish(message);
    published_revision_ = snapshot.revision;
    RCLCPP_INFO(
      get_logger(), "MISSION %s -> %s: %s",
      message.previous_state.c_str(), message.state.c_str(), message.detail.c_str());
  }

  std::unique_ptr<MissionFsm> fsm_;
  std::uint64_t published_revision_{std::numeric_limits<std::uint64_t>::max()};
  rclcpp::Publisher<auv_interfaces::msg::MissionState>::SharedPtr state_publisher_;
  rclcpp::Subscription<auv_interfaces::msg::Stm32Status>::SharedPtr status_subscription_;
  rclcpp::Subscription<auv_interfaces::msg::AprilTagDetectionArray>::SharedPtr
    apriltag_subscription_;
  rclcpp::Subscription<auv_interfaces::msg::SemanticMap>::SharedPtr map_subscription_;
  rclcpp::Subscription<auv_interfaces::msg::PlannedRoute>::SharedPtr route_subscription_;
  rclcpp::Service<auv_interfaces::srv::MissionCommand>::SharedPtr command_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<rclcpp::Node> make_mission_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<MissionNode>(options);
}

}  // namespace auv_mission
