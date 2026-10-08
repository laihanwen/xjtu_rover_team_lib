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
#include <cmath>
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
    case MissionPhase::kSearchCucumber:
      return "SEARCH_CUCUMBER";
    case MissionPhase::kAlignCucumber:
      return "ALIGN_CUCUMBER";
    case MissionPhase::kGrab:
      return "GRAB";
    case MissionPhase::kTransport:
      return "TRANSPORT";
    case MissionPhase::kRelease:
      return "RELEASE";
    case MissionPhase::kSearchValve:
      return "SEARCH_VALVE";
    case MissionPhase::kAlignValve:
      return "ALIGN_VALVE";
    case MissionPhase::kRotateValve:
      return "ROTATE_VALVE";
    case MissionPhase::kReturnHome:
      return "RETURN_HOME";
    case MissionPhase::kSurface:
      return "SURFACE";
    case MissionPhase::kSurfaceForCones:
      return "SURFACE_FOR_CONES";
    case MissionPhase::kRelocalizeSurface:
      return "RELOCALIZE_SURFACE";
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
    config_.cone_visit_timeout_sec <= 0.0 || config_.cucumber_search_timeout_sec <= 0.0 ||
    config_.cucumber_align_timeout_sec <= 0.0 || config_.gripper_timeout_sec <= 0.0 ||
    config_.transport_timeout_sec <= 0.0 || config_.valve_search_timeout_sec <= 0.0 ||
    config_.valve_align_timeout_sec <= 0.0 || config_.valve_rotate_timeout_sec <= 0.0 ||
    config_.return_home_timeout_sec <= 0.0 || config_.surface_timeout_sec <= 0.0 ||
    config_.status_timeout_sec <= 0.0 || !std::isfinite(config_.surface_relocalize_timeout_sec) ||
    config_.surface_relocalize_timeout_sec <= 0.0 ||
    (config_.surface_before_visit && (config_.stop_after_map || config_.full_mission)))
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

void MissionFsm::force_fault(const std::string & detail, double now_sec)
{
  if (snapshot_.phase != MissionPhase::kFault) {
    fault(detail, now_sec);
  }
}

bool MissionFsm::is_active() const
{
  return snapshot_.phase == MissionPhase::kSelfCheck ||
         snapshot_.phase == MissionPhase::kSearchAprilTag ||
         snapshot_.phase == MissionPhase::kBuildMap ||
         snapshot_.phase == MissionPhase::kPlanCones ||
         snapshot_.phase == MissionPhase::kVisitCones ||
         snapshot_.phase == MissionPhase::kSearchCucumber ||
         snapshot_.phase == MissionPhase::kAlignCucumber ||
         snapshot_.phase == MissionPhase::kGrab ||
         snapshot_.phase == MissionPhase::kTransport ||
         snapshot_.phase == MissionPhase::kRelease ||
         snapshot_.phase == MissionPhase::kSearchValve ||
         snapshot_.phase == MissionPhase::kAlignValve ||
         snapshot_.phase == MissionPhase::kRotateValve ||
         snapshot_.phase == MissionPhase::kReturnHome ||
         snapshot_.phase == MissionPhase::kSurface ||
         snapshot_.phase == MissionPhase::kSurfaceForCones ||
         snapshot_.phase == MissionPhase::kRelocalizeSurface;
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
    case MissionPhase::kSearchCucumber:
      return config_.cucumber_search_timeout_sec;
    case MissionPhase::kAlignCucumber:
      return config_.cucumber_align_timeout_sec;
    case MissionPhase::kGrab:
    case MissionPhase::kRelease:
      return config_.gripper_timeout_sec;
    case MissionPhase::kTransport:
      return config_.transport_timeout_sec;
    case MissionPhase::kSearchValve:
      return config_.valve_search_timeout_sec;
    case MissionPhase::kAlignValve:
      return config_.valve_align_timeout_sec;
    case MissionPhase::kRotateValve:
      return config_.valve_rotate_timeout_sec;
    case MissionPhase::kReturnHome:
      return config_.return_home_timeout_sec;
    case MissionPhase::kSurface:
    case MissionPhase::kSurfaceForCones:
      return config_.surface_timeout_sec;
    case MissionPhase::kRelocalizeSurface:
      return config_.surface_relocalize_timeout_sec;
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
      cucumber_received_sec_ = gripper_received_sec_ = transport_received_sec_ = -1.0;
      valve_received_sec_ = home_received_sec_ = surface_received_sec_ = -1.0;
      cucumber_found_ = cucumber_aligned_ = grabbed_ = released_ = false;
      transport_complete_ = valve_found_ = valve_aligned_ = valve_rotated_ = false;
      home_reached_ = surfaced_ = false;
      surface_pose_ready_=false;surface_pose_received_sec_=-1;
      transition(MissionPhase::kInit, "mission reset; waiting for START", now_sec);
      return {true, "mission reset"};
  }
  return {false, "unknown command"};
}

void MissionFsm::update_status(
  bool connected, bool armed, std::uint32_t error_flags,
  double now_sec)
{
  connected_ = connected;
  armed_ = armed;
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

void MissionFsm::update_cucumber(bool found, bool aligned, double now_sec)
{
  cucumber_found_ = found;
  cucumber_aligned_ = found && aligned;
  cucumber_received_sec_ = now_sec;
}

void MissionFsm::update_gripper(bool grabbed, bool released, double now_sec)
{
  grabbed_ = grabbed;
  released_ = released;
  gripper_received_sec_ = now_sec;
}

void MissionFsm::update_transport(bool complete, double now_sec)
{
  transport_complete_ = complete;
  transport_received_sec_ = now_sec;
}

void MissionFsm::update_valve(bool found, bool aligned, bool rotated, double now_sec)
{
  valve_found_ = found;
  valve_aligned_ = found && aligned;
  valve_rotated_ = aligned && rotated;
  valve_received_sec_ = now_sec;
}

void MissionFsm::update_home(bool reached, double now_sec)
{
  home_reached_ = reached;
  home_received_sec_ = now_sec;
}

void MissionFsm::update_surface(bool surfaced, double now_sec)
{
  surfaced_ = surfaced;
  surface_received_sec_ = now_sec;
}

void MissionFsm::update_surface_pose(bool ready,double now_sec) {
  surface_pose_ready_=ready;surface_pose_received_sec_=now_sec;
}

void MissionFsm::tick(double now_sec)
{
  const bool running_or_paused = is_active() || snapshot_.phase == MissionPhase::kPaused;
  if (running_or_paused && status_received_sec_ >= 0.0) {
    if (error_flags_ != 0U) {
      fault("STM32 error flags are nonzero", now_sec);
      return;
    }
    const bool after_route = snapshot_.phase >= MissionPhase::kVisitCones &&
      snapshot_.phase <= MissionPhase::kSurface;
    const bool allowed_visit_arm = config_.allow_armed_during_visit && after_route;
    const bool allowed_observation_arm = config_.allow_armed_during_observation &&
      (snapshot_.phase == MissionPhase::kSearchAprilTag || snapshot_.phase == MissionPhase::kBuildMap);
    const bool allowed_surface_arm = config_.surface_before_visit && config_.allow_armed_during_visit &&
      (snapshot_.phase==MissionPhase::kSurfaceForCones || snapshot_.phase==MissionPhase::kRelocalizeSurface ||
       snapshot_.phase==MissionPhase::kPlanCones);
    const bool pause_disarm_grace = snapshot_.phase == MissionPhase::kPaused &&
      now_sec - phase_entered_sec_ <= 0.5;
    if (armed_ && !allowed_visit_arm && !allowed_observation_arm && !allowed_surface_arm && !pause_disarm_grace) {
      fault("propulsion armed outside the permitted visit phase", now_sec);
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
      if (status_received_sec_ >= 0.0 && connected_ && !armed_ &&
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
        surfaced_=false;surface_pose_ready_=false;surface_pose_received_sec_=-1;
        transition(config_.stop_after_map ? MissionPhase::kComplete :
          (config_.surface_before_visit ? MissionPhase::kSurfaceForCones : MissionPhase::kPlanCones),
          config_.stop_after_map ? "A1 observation complete; propulsion stop" : "semantic map complete", now_sec);
      }
      break;
    case MissionPhase::kSurfaceForCones:
      if(surface_received_sec_>=phase_entered_sec_ && surfaced_) {
        surface_pose_ready_=false;surface_pose_received_sec_=-1;
        transition(MissionPhase::kRelocalizeSurface,"ascent confirmed; require new surface grid observations",now_sec);
      }
      break;
    case MissionPhase::kRelocalizeSurface:
      if(surface_pose_received_sec_>=phase_entered_sec_ && surface_pose_ready_)
        transition(MissionPhase::kPlanCones,"absolute surface grid reacquired",now_sec);
      break;
    case MissionPhase::kPlanCones:
      if (route_received_sec_ >= 0.0 && route_valid_) {
        if (route_has_targets_) {
          transition(MissionPhase::kVisitCones, "cone route ready", now_sec);
        } else {
          fault("semantic map completed without an unvisited cone route", now_sec);
        }
      }
      break;
    case MissionPhase::kVisitCones:
      if (map_received_sec_ >= 0.0 && map_complete_ && all_cones_visited_) {
        transition(
          config_.full_mission ? MissionPhase::kSearchCucumber : MissionPhase::kComplete,
          config_.full_mission ? "cone targets visited; searching for cucumber" :
          "all cone targets visited", now_sec);
      }
      break;
    case MissionPhase::kSearchCucumber:
      if (cucumber_received_sec_ >= 0.0 && cucumber_found_) {
        transition(MissionPhase::kAlignCucumber, "sea cucumber found", now_sec);
      }
      break;
    case MissionPhase::kAlignCucumber:
      if (cucumber_received_sec_ >= 0.0 && cucumber_aligned_) {
        transition(MissionPhase::kGrab, "sea cucumber aligned", now_sec);
      }
      break;
    case MissionPhase::kGrab:
      if (gripper_received_sec_ >= 0.0 && grabbed_) {
        transition(MissionPhase::kTransport, "grab confirmed", now_sec);
      }
      break;
    case MissionPhase::kTransport:
      if (transport_received_sec_ >= 0.0 && transport_complete_) {
        transition(MissionPhase::kRelease, "transport destination reached", now_sec);
      }
      break;
    case MissionPhase::kRelease:
      if (gripper_received_sec_ >= 0.0 && released_) {
        transition(MissionPhase::kSearchValve, "release confirmed", now_sec);
      }
      break;
    case MissionPhase::kSearchValve:
      if (valve_received_sec_ >= 0.0 && valve_found_) {
        transition(MissionPhase::kAlignValve, "valve found", now_sec);
      }
      break;
    case MissionPhase::kAlignValve:
      if (valve_received_sec_ >= 0.0 && valve_aligned_) {
        transition(MissionPhase::kRotateValve, "valve aligned", now_sec);
      }
      break;
    case MissionPhase::kRotateValve:
      if (valve_received_sec_ >= 0.0 && valve_rotated_) {
        transition(MissionPhase::kReturnHome, "valve rotation confirmed", now_sec);
      }
      break;
    case MissionPhase::kReturnHome:
      if (home_received_sec_ >= 0.0 && home_reached_) {
        transition(MissionPhase::kSurface, "home reached", now_sec);
      }
      break;
    case MissionPhase::kSurface:
      if (surface_received_sec_ >= 0.0 && surfaced_) {
        transition(MissionPhase::kComplete, "surface confirmed", now_sec);
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
