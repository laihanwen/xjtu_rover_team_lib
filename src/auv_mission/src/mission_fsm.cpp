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

#include "auv_mission/mission_fsm.hpp"

#include <stdexcept>
#include <utility>

namespace auv_mission
{

std::string mission_phase_name(MissionPhase phase)
{
  switch (phase) {
    case MissionPhase::kInit:
      return "INIT";
    case MissionPhase::kSelfCheck:
      return "SELF_CHECK";
    case MissionPhase::kSearchAprilTag:
      return "SEARCH_APRILTAG";
    case MissionPhase::kBuildMap:
      return "BUILD_MAP";
    case MissionPhase::kPlanCones:
      return "PLAN_CONES";
    case MissionPhase::kVisitCones:
      return "VISIT_CONES";
    case MissionPhase::kPaused:
      return "PAUSED";
    case MissionPhase::kComplete:
      return "COMPLETE";
    case MissionPhase::kFault:
      return "FAULT";
    case MissionPhase::kAborted:
      return "ABORTED";
  }
  return "UNKNOWN";
}

MissionFsm::MissionFsm(MissionFsmConfig config)
: config_(std::move(config))
{
  if (config_.self_check_timeout_sec <= 0.0 || config_.apriltag_timeout_sec <= 0.0 ||
    config_.map_timeout_sec <= 0.0 || config_.planning_timeout_sec <= 0.0 ||
    config_.cone_visit_timeout_sec <= 0.0 || config_.status_timeout_sec <= 0.0)
  {
    throw std::invalid_argument("mission timeouts must be positive");
  }
}

void MissionFsm::transition(
  MissionPhase phase, const std::string & detail, double now_sec)
{
  snapshot_.previous_phase = snapshot_.phase;
  snapshot_.phase = phase;
  snapshot_.detail = detail;
  snapshot_.faulted = phase == MissionPhase::kFault;
  ++snapshot_.revision;
  phase_entered_sec_ = now_sec;
}

void MissionFsm::fault(const std::string & detail, double now_sec)
{
  transition(MissionPhase::kFault, detail, now_sec);
}

bool MissionFsm::is_active() const
{
  return snapshot_.phase == MissionPhase::kSelfCheck ||
         snapshot_.phase == MissionPhase::kSearchAprilTag ||
         snapshot_.phase == MissionPhase::kBuildMap ||
         snapshot_.phase == MissionPhase::kPlanCones ||
         snapshot_.phase == MissionPhase::kVisitCones;
}

double MissionFsm::phase_timeout() const
{
  switch (snapshot_.phase) {
    case MissionPhase::kSelfCheck:
      return config_.self_check_timeout_sec;
    case MissionPhase::kSearchAprilTag:
      return config_.apriltag_timeout_sec;
    case MissionPhase::kBuildMap:
      return config_.map_timeout_sec;
    case MissionPhase::kPlanCones:
      return config_.planning_timeout_sec;
    case MissionPhase::kVisitCones:
      return config_.cone_visit_timeout_sec;
    default:
      return 0.0;
  }
}

CommandResult MissionFsm::command(MissionCommand command_value, double now_sec)
{
  switch (command_value) {
    case MissionCommand::kStart:
      if (snapshot_.phase != MissionPhase::kInit) {
        return {false, "START is only valid in INIT"};
      }
      transition(MissionPhase::kSelfCheck, "mission start requested", now_sec);
      return {true, "mission started"};
    case MissionCommand::kPause:
      if (!is_active()) {
        return {false, "PAUSE requires an active mission"};
      }
      paused_phase_ = snapshot_.phase;
      transition(MissionPhase::kPaused, "mission paused by operator", now_sec);
      return {true, "mission paused"};
    case MissionCommand::kResume:
      if (snapshot_.phase != MissionPhase::kPaused) {
        return {false, "RESUME is only valid in PAUSED"};
      }
      transition(paused_phase_, "mission resumed by operator", now_sec);
      return {true, "mission resumed"};
    case MissionCommand::kAbort:
      if (!is_active() && snapshot_.phase != MissionPhase::kPaused) {
        return {false, "ABORT requires an active or paused mission"};
      }
      transition(MissionPhase::kAborted, "mission aborted by operator", now_sec);
      return {true, "mission aborted"};
    case MissionCommand::kReset:
      if (snapshot_.phase != MissionPhase::kComplete &&
        snapshot_.phase != MissionPhase::kFault &&
        snapshot_.phase != MissionPhase::kAborted)
      {
        return {false, "RESET requires COMPLETE, FAULT, or ABORTED"};
      }
      connected_ = false;
      armed_ = false;
      leak_detected_ = false;
      error_flags_ = 0U;
      apriltag_found_ = false;
      map_complete_ = false;
      all_cones_visited_ = false;
      route_valid_ = false;
      route_has_targets_ = false;
      status_received_sec_ = -1.0;
      apriltag_received_sec_ = -1.0;
      map_received_sec_ = -1.0;
      route_received_sec_ = -1.0;
      transition(MissionPhase::kInit, "mission reset; waiting for START", now_sec);
      return {true, "mission reset"};
  }
  return {false, "unknown command"};
}

void MissionFsm::update_status(
  bool connected, bool armed, bool leak_detected, std::uint32_t error_flags,
  double now_sec)
{
  connected_ = connected;
  armed_ = armed;
  leak_detected_ = leak_detected;
  error_flags_ = error_flags;
  status_received_sec_ = now_sec;
}

void MissionFsm::update_apriltag(bool found, double now_sec)
{
  apriltag_found_ = found;
  apriltag_received_sec_ = now_sec;
}

void MissionFsm::update_map(bool complete, bool all_cones_visited, double now_sec)
{
  map_complete_ = complete;
  all_cones_visited_ = all_cones_visited;
  map_received_sec_ = now_sec;
}

void MissionFsm::update_route(bool valid, bool has_targets, double now_sec)
{
  route_valid_ = valid;
  route_has_targets_ = has_targets;
  route_received_sec_ = now_sec;
}

void MissionFsm::tick(double now_sec)
{
  const bool running_or_paused = is_active() || snapshot_.phase == MissionPhase::kPaused;
  if (running_or_paused && status_received_sec_ >= 0.0) {
    if (leak_detected_) {
      fault("leak detected", now_sec);
      return;
    }
    if (error_flags_ != 0U) {
      fault("STM32 error flags are nonzero", now_sec);
      return;
    }
    if (armed_) {
      fault("P11 orchestration requires propulsion to remain DISARMED", now_sec);
      return;
    }
  }
  if ((snapshot_.phase != MissionPhase::kSelfCheck) && running_or_paused &&
    (status_received_sec_ < 0.0 || now_sec - status_received_sec_ > config_.status_timeout_sec))
  {
    fault("STM32 status timeout", now_sec);
    return;
  }
  if ((snapshot_.phase != MissionPhase::kSelfCheck) && running_or_paused && !connected_) {
    fault("STM32 disconnected", now_sec);
    return;
  }
  if (snapshot_.phase == MissionPhase::kPaused) {
    return;
  }
  if (is_active() && now_sec - phase_entered_sec_ > phase_timeout()) {
    fault(mission_phase_name(snapshot_.phase) + " timeout", now_sec);
    return;
  }

  switch (snapshot_.phase) {
    case MissionPhase::kSelfCheck:
      if (status_received_sec_ >= 0.0 && connected_ && !armed_ && !leak_detected_ &&
        error_flags_ == 0U)
      {
        transition(MissionPhase::kSearchAprilTag, "self-check passed", now_sec);
      }
      break;
    case MissionPhase::kSearchAprilTag:
      if (apriltag_received_sec_ >= 0.0 && apriltag_found_) {
        transition(MissionPhase::kBuildMap, "AprilTag found", now_sec);
      }
      break;
    case MissionPhase::kBuildMap:
      if (map_received_sec_ >= 0.0 && map_complete_) {
        transition(MissionPhase::kPlanCones, "semantic map complete", now_sec);
      }
      break;
    case MissionPhase::kPlanCones:
      if (route_received_sec_ >= 0.0 && route_valid_) {
        if (route_has_targets_) {
          transition(MissionPhase::kVisitCones, "cone route ready", now_sec);
        } else {
          transition(MissionPhase::kComplete, "no unvisited cone targets", now_sec);
        }
      }
      break;
    case MissionPhase::kVisitCones:
      if (map_received_sec_ >= 0.0 && map_complete_ && all_cones_visited_) {
        transition(MissionPhase::kComplete, "all cone targets visited", now_sec);
      }
      break;
    default:
      break;
  }
}

MissionSnapshot MissionFsm::snapshot() const
{
  return snapshot_;
}

}  // namespace auv_mission
