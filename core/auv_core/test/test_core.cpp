#include "auv_core/status_decoder.hpp"
#include "auv_core/semantic_map.hpp"
#include "auv_control/route_executor.hpp"
#include "auv_stm32_bridge/protocol.h"
#include <stdexcept>
static void require(bool value) { if (!value) throw std::runtime_error("core check failed"); }
#include <cmath>
#include <limits>
int main() {
  std::vector<std::uint8_t> bytes(30,0);
  auv_protocol_write_f32_le(bytes.data()+9,12.0F);
  auv_protocol_write_f32_le(bytes.data()+13,1.0F);
  auv_core::Stm32Status status;
  require(auv_core::decode_status(bytes,status));
  require(status.voltage == 12.0F && !status.armed && status.telemetry_valid);
  require(!status.dual_mode && !status.autonomous_mode);
  bytes[4]=16;
  require(auv_core::decode_status(bytes,status) && status.dual_mode && !status.autonomous_mode);
  bytes[4]=48;
  require(auv_core::decode_status(bytes,status) && status.dual_mode && status.autonomous_mode);
  bytes[4]=0;
  bytes[29] = 9;
  require(!auv_core::decode_status(bytes,status));
  bytes[29] = 0;
  auv_protocol_write_f32_le(bytes.data()+13,std::numeric_limits<float>::quiet_NaN());
  require(auv_core::decode_status(bytes,status) && !status.telemetry_valid);
  auv_protocol_write_f32_le(bytes.data()+13,1.0F);
  auv_protocol_write_f32_le(bytes.data()+9,std::numeric_limits<float>::quiet_NaN());
  require(auv_core::decode_status(bytes,status) && status.telemetry_valid && !status.voltage_valid);
  auv_mapping::GridResult grid;
  grid.stable = true;
  grid.orientation_valid = true;
  auv_vision::ConeObservation cone;
  cone.shape = auv_vision::ConeShape::kCircle; cone.row = 0; cone.col = 1; cone.confidence = 0.9F;
  auto map = auv_core::fuse_semantic_map(grid,{cone},true,{},1);
  require(map.complete && map.grid.cells[1].cell.object_type == "circle_cone");
  auv_planning::GridPlanner planner;
  auto plan = planner.plan(map.grid,{0,0,"unknown"});
  require(plan.valid && !plan.targets.empty());
  auv_control::RouteExecutor executor;
  executor.set_route(plan,1);
  executor.set_mission_active(true);
  require(executor.step().state == auv_control::RouteStep::State::kWaitingForArm);
  executor.set_vehicle_ready(true);
  executor.set_pose(true,std::numeric_limits<double>::quiet_NaN(),0.5);
  require(executor.step().state == auv_control::RouteStep::State::kFault);
  executor.reset();
  plan.path.front().row = 9;
  executor.set_route(plan,2);
  executor.set_mission_active(true);
  require(executor.step().state == auv_control::RouteStep::State::kFault);
}
