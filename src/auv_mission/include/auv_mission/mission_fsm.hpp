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

#ifndef AUV_MISSION__MISSION_FSM_HPP_
#define AUV_MISSION__MISSION_FSM_HPP_

#include <cstdint>
#include <string>

namespace auv_mission
{

enum class MissionPhase
{
  kInit,
  kSelfCheck,
  kSearchAprilTag,
  kBuildMap,
  kPlanCones,
  kVisitCones,
  kPaused,
  kComplete,
  kFault,
  kAborted
};

enum class MissionCommand
{
  kStart,
  kPause,
  kResume,
  kAbort,
  kReset
};

struct MissionFsmConfig
{
  double self_check_timeout_sec{10.0};
  double apriltag_timeout_sec{30.0};
  double map_timeout_sec{30.0};
  double planning_timeout_sec{10.0};
  double cone_visit_timeout_sec{120.0};
  double status_timeout_sec{1.0};
  bool allow_armed_during_visit{false};
};

struct CommandResult
{
  bool accepted{false};
  std::string message;
};

struct MissionSnapshot
{
  MissionPhase phase{MissionPhase::kInit};
  MissionPhase previous_phase{MissionPhase::kInit};
  std::string detail{"waiting for START"};
  bool faulted{false};
  std::uint64_t revision{0U};
};

class MissionFsm
{
public:
  explicit MissionFsm(MissionFsmConfig config = {});

  CommandResult command(MissionCommand command, double now_sec);
  void tick(double now_sec);
  void update_status(
    bool connected, bool armed, bool leak_detected, std::uint32_t error_flags,
    double now_sec);
  void update_apriltag(bool found, double now_sec);
  void update_map(bool complete, bool all_cones_visited, double now_sec);
  void update_route(bool valid, bool has_targets, double now_sec);

  MissionSnapshot snapshot() const;

private:
  void transition(MissionPhase phase, const std::string & detail, double now_sec);
  void fault(const std::string & detail, double now_sec);
  double phase_timeout() const;
  bool is_active() const;

  MissionFsmConfig config_;
  MissionSnapshot snapshot_;
  MissionPhase paused_phase_{MissionPhase::kInit};
  double phase_entered_sec_{0.0};
  double status_received_sec_{-1.0};
  double apriltag_received_sec_{-1.0};
  double map_received_sec_{-1.0};
  double route_received_sec_{-1.0};
  bool connected_{false};
  bool armed_{false};
  bool leak_detected_{false};
  std::uint32_t error_flags_{0U};
  bool apriltag_found_{false};
  bool map_complete_{false};
  bool all_cones_visited_{false};
  bool route_valid_{false};
  bool route_has_targets_{false};
};

std::string mission_phase_name(MissionPhase phase);

}  // namespace auv_mission

#endif  // AUV_MISSION__MISSION_FSM_HPP_
