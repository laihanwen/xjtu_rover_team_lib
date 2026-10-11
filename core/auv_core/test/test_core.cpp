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
  auv_mapping::LocalMapAccumulator local;
  local.reset(1,"observed-grid");
  auto partial=auv_core::fuse_local_semantic_map(local,{},4);
  require(!partial.complete && partial.cone_count==0 && partial.grid.cells[0].cell.object_type=="unknown");
  auv_mapping::LocalMapObservation sample;
  sample.epoch=1;sample.grid_frame="observed-grid";sample.association_unique=true;
  for(int row=0;row<3;++row)for(int col=0;col<3;++col)
    sample.cells.push_back({row,col,row==2?auv_mapping::LocalCellClass::square:
      row==1&&col==0?auv_mapping::LocalCellClass::circle:auv_mapping::LocalCellClass::empty,true,.9,{}});
  std::string reason;
  for(int n=1;n<=3;++n){sample.sequence=n;sample.stamp=.1*n;require(local.update(sample,sample.stamp,reason));}
  std::array<bool,9> local_visited{};local_visited[6]=true;
  auto accumulated=auv_core::fuse_local_semantic_map(local,local_visited,4);
  require(accumulated.complete && accumulated.grid.complete && accumulated.cone_count==4);
  require(accumulated.grid.cells[6].cell.object_type=="square_cone" && accumulated.visited[6]);
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
