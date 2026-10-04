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

#include "auv_mission/mission_fsm.hpp"
#include "gtest/gtest.h"

namespace
{

using auv_mission::MissionCommand;
using auv_mission::MissionFsm;
using auv_mission::MissionFsmConfig;
using auv_mission::MissionPhase;

void provide_safe_status(MissionFsm & fsm, double now)
{
  fsm.update_status(true, false, false, 0U, now);
}

TEST(MissionFsm, RunsConeMissionToCompletion)
{
  MissionFsm fsm;
  EXPECT_TRUE(fsm.command(MissionCommand::kStart, 0.0).accepted);
  provide_safe_status(fsm, 0.1);
  fsm.tick(0.1);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kSearchAprilTag);
  provide_safe_status(fsm, 0.2);
  fsm.update_apriltag(true, 0.2);
  fsm.tick(0.2);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kBuildMap);
  provide_safe_status(fsm, 0.3);
  fsm.update_map(true, false, 0.3);
  fsm.tick(0.3);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kPlanCones);
  provide_safe_status(fsm, 0.4);
  fsm.update_route(true, true, 0.4);
  fsm.tick(0.4);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kVisitCones);
  provide_safe_status(fsm, 0.5);
  fsm.update_map(true, true, 0.5);
  fsm.tick(0.5);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kComplete);
  EXPECT_FALSE(fsm.snapshot().faulted);
}

TEST(MissionFsm, FaultsWhenPlannerHasNoTargets)
{
  MissionFsm fsm;
  ASSERT_TRUE(fsm.command(MissionCommand::kStart, 0.0).accepted);
  provide_safe_status(fsm, 0.1);
  fsm.tick(0.1);
  fsm.update_apriltag(true, 0.2);
  provide_safe_status(fsm, 0.2);
  fsm.tick(0.2);
  fsm.update_map(true, false, 0.3);
  provide_safe_status(fsm, 0.3);
  fsm.tick(0.3);
  fsm.update_route(true, false, 0.4);
  provide_safe_status(fsm, 0.4);
  fsm.tick(0.4);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kFault);
}

TEST(MissionFsm, AllowsArmedVehicleOnlyDuringVisitWhenConfigured)
{
  MissionFsmConfig config;
  config.allow_armed_during_visit = true;
  MissionFsm fsm(config);
  ASSERT_TRUE(fsm.command(MissionCommand::kStart, 0.0).accepted);
  provide_safe_status(fsm, 0.1);
  fsm.tick(0.1);
  fsm.update_apriltag(true, 0.2);
  provide_safe_status(fsm, 0.2);
  fsm.tick(0.2);
  fsm.update_map(true, false, 0.3);
  provide_safe_status(fsm, 0.3);
  fsm.tick(0.3);
  fsm.update_route(true, true, 0.4);
  provide_safe_status(fsm, 0.4);
  fsm.tick(0.4);
  ASSERT_EQ(fsm.snapshot().phase, MissionPhase::kVisitCones);
  fsm.update_status(true, true, false, 0U, 0.5);
  fsm.tick(0.5);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kVisitCones);
}

TEST(MissionFsm, SupportsPauseResumeAbortAndReset)
{
  MissionFsm fsm;
  EXPECT_FALSE(fsm.command(MissionCommand::kPause, 0.0).accepted);
  ASSERT_TRUE(fsm.command(MissionCommand::kStart, 0.0).accepted);
  ASSERT_TRUE(fsm.command(MissionCommand::kPause, 0.1).accepted);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kPaused);
  provide_safe_status(fsm, 0.2);
  ASSERT_TRUE(fsm.command(MissionCommand::kResume, 0.2).accepted);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kSelfCheck);
  ASSERT_TRUE(fsm.command(MissionCommand::kAbort, 0.3).accepted);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kAborted);
  ASSERT_TRUE(fsm.command(MissionCommand::kReset, 0.4).accepted);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kInit);
}

TEST(MissionFsm, PauseAllowsBriefDisarmThenFaultsIfStillArmed)
{
  MissionFsm fsm;
  ASSERT_TRUE(fsm.command(MissionCommand::kStart, 0.0).accepted);
  ASSERT_TRUE(fsm.command(MissionCommand::kPause, 0.1).accepted);
  fsm.update_status(true, true, false, 0U, 0.2);
  fsm.tick(0.2);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kPaused);
  fsm.update_status(true, true, false, 0U, 0.7);
  fsm.tick(0.7);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kFault);
}

TEST(MissionFsm, FaultsOnLeakArmErrorsDisconnectAndStatusTimeout)
{
  MissionFsm leak_fsm;
  ASSERT_TRUE(leak_fsm.command(MissionCommand::kStart, 0.0).accepted);
  leak_fsm.update_status(true, false, true, 0U, 0.1);
  leak_fsm.tick(0.1);
  EXPECT_EQ(leak_fsm.snapshot().phase, MissionPhase::kFault);

  MissionFsm armed_fsm;
  ASSERT_TRUE(armed_fsm.command(MissionCommand::kStart, 0.0).accepted);
  armed_fsm.update_status(true, true, false, 0U, 0.1);
  armed_fsm.tick(0.1);
  EXPECT_EQ(armed_fsm.snapshot().phase, MissionPhase::kFault);

  MissionFsm error_fsm;
  ASSERT_TRUE(error_fsm.command(MissionCommand::kStart, 0.0).accepted);
  error_fsm.update_status(true, false, false, 4U, 0.1);
  error_fsm.tick(0.1);
  EXPECT_EQ(error_fsm.snapshot().phase, MissionPhase::kFault);

  MissionFsm timeout_fsm;
  ASSERT_TRUE(timeout_fsm.command(MissionCommand::kStart, 0.0).accepted);
  provide_safe_status(timeout_fsm, 0.1);
  timeout_fsm.tick(0.1);
  timeout_fsm.tick(1.2);
  EXPECT_EQ(timeout_fsm.snapshot().phase, MissionPhase::kFault);
}

TEST(MissionFsm, FaultsWhenAStageTimesOut)
{
  MissionFsmConfig config;
  config.apriltag_timeout_sec = 0.5;
  config.status_timeout_sec = 2.0;
  MissionFsm fsm(config);
  ASSERT_TRUE(fsm.command(MissionCommand::kStart, 0.0).accepted);
  provide_safe_status(fsm, 0.1);
  fsm.tick(0.1);
  provide_safe_status(fsm, 0.7);
  fsm.tick(0.7);
  EXPECT_EQ(fsm.snapshot().phase, MissionPhase::kFault);
  EXPECT_EQ(fsm.snapshot().detail, "SEARCH_APRILTAG timeout");
}

TEST(MissionFsm, ValidatesTimeoutConfiguration)
{
  MissionFsmConfig config;
  config.map_timeout_sec = 0.0;
  EXPECT_THROW(MissionFsm fsm(config), std::invalid_argument);
}

}  // namespace
