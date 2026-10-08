#include "auv_control/surface_traversal.hpp"
#include "auv_mission/mission_fsm.hpp"
#include "auv_stm32_bridge/depth_sample.hpp"
#include "auv_stm32_bridge/telemetry_decoder.hpp"
#include <iostream>
#include <stdexcept>
#include <set>

static void check(bool x,const char* detail){if(!x)throw std::runtime_error(detail);}
using namespace auv_control;
using namespace auv_planning;
static SurfaceTraversalConfig config() {
  SurfaceTraversalConfig c;c.enabled=c.ascent_verified=c.surface_verified=true;c.ascent_row=c.ascent_col=1;
  c.boundary_clearance_m=.1;c.ascent_rate_mps=.1;
  c.center_approach_verified=true;c.center_approach_radius_cells=.3;
  auto& g=c.underwater;g.verified=true;g.cell_size_m=.5;g.pool_depth_m=2;g.width=640;g.height=480;
  g.camera_matrix={300,0,320,0,300,240,0,0,1};g.distortion={0,0,0,0,0};
  g.camera_to_body={0,1,0,1,0,0,0,0,-1};g.camera_offset_m={0,0,0};g.depth_offset_m={0,0,0};
  c.surface=g;c.validate();return c;
}
static PlanningGrid grid(unsigned targets) {
  PlanningGrid g;g.rows=g.cols=3;g.complete=true;
  for(int i=0;i<9;++i)g.cells.push_back({{static_cast<std::int8_t>(i/3),static_cast<std::int8_t>(i%3),
    targets&(1U<<i)?"circle_cone":"empty"},false});return g;
}
static void planning() {
  GridPlannerConfig c;c.forbid_target_reentry=true;GridPlanner planner(c);
  int count=0;
  for(unsigned mask=0;mask<512;++mask) {
    int bits=0;for(int i=0;i<9;++i)bits+=(mask>>i)&1U;if(bits!=4)continue;
    for(int start=0;start<9;++start) {
      const auto p=planner.plan(grid(mask),{static_cast<std::int8_t>(start/3),static_cast<std::int8_t>(start%3),"empty"});
      check(p.valid&&p.targets.size()==4,"strict route unavailable");
      unsigned entered=0;
      for(std::size_t i=0;i<p.path.size();++i) {
        const int id=p.path[i].row*3+p.path[i].col;
        if(mask&(1U<<id)){check(!(entered&(1U<<id)),"planned target reentry");entered|=1U<<id;}
        if(i)check(std::abs(p.path[i].row-p.path[i-1].row)+std::abs(p.path[i].col-p.path[i-1].col)==1,"diagonal planned leg");
      }
      check(entered==mask,"missing target");++count;
    }
  }
  check(count==1134,"exhaustive four-cone layouts");
  auto blocked=grid((1<<0)|(1<<2)|(1<<6)|(1<<8));
  for(int i:{1,3,5,7})blocked.cells[i].cell.object_type="obstacle";
  check(!planner.plan(blocked,{1,1,"empty"}).valid,"unreachable should reject");
  auto prior=grid((1<<0)|(1<<2)|(1<<6)|(1<<8));prior.cells[0].visited=true;
  check(!planner.plan(prior,{0,0,"circle_cone"}).valid,"visited target start should reject");
}
static void geometry() {
  const auto c=config();const auto& calibration=c.surface;
  const std::vector<cv::Point3d> world{{0,0,0},{1.5,0,0},{1.5,1.5,0},{0,1.5,0}};
  for(double yaw:{0.0,1.5707963267948966,3.141592653589793,4.71238898038469}) {
    cv::Mat r;cv::Rodrigues(cv::Vec3d(0,0,yaw),r);
    cv::Matx33d rotation;for(int i=0;i<9;++i)rotation.val[i]=r.at<double>(i/3,i%3);
    const auto t=-(rotation*cv::Vec3d(.75,.75,-2));
    std::vector<cv::Point2d> pixels;cv::projectPoints(world,cv::Vec3d(0,0,yaw),t,cv::Matx33d(calibration.camera_matrix.data()),cv::noArray(),pixels);
    auv_mapping::GridResult g;g.geometry_valid=g.stable=g.orientation_valid=true;g.yellow_edge=2;g.confidence=1;
    for(int i=0;i<4;++i)g.corners[i]=pixels[i];
    auto pose=auv_mapping::metric_grid_pose(g,calibration,{640,480},0,10,1);
    if(!pose.valid)throw std::runtime_error("metric PnP yaw="+std::to_string(yaw)+" "+pose.reason+" error="+std::to_string(pose.reprojection_px));
    check(std::abs(pose.row-1.5)<.001&&std::abs(pose.col-1.5)<.001,"metric body coordinate");
    check(!auv_mapping::metric_grid_pose(g,calibration,{320,240},0,10,1).valid,"resolution mismatch");
    check(!auv_mapping::metric_grid_pose(g,calibration,{640,480},.8,10,1).valid,"height mismatch");
    g.orientation_valid=false;check(!auv_mapping::metric_grid_pose(g,calibration,{640,480},0,10,1).valid,"unoriented pose");
  }
}
static auv_mapping::MetricGridPose pose(double row,double col,double time,std::uint64_t sequence) {
  auv_mapping::MetricGridPose p;p.valid=true;p.row=row;p.col=col;p.stamp=time;p.sequence=sequence;
  p.body_from_grid={0,1,0,1,0,0,0,0,-1};return p;
}
static void ascent() {
  auto c=config();ControlledAscent rise(c);auto p=pose(1.5,1.5,10,1);
  CenterApproach center(c);auto offset=pose(1.7,1.5,9,1);
  check(center.step(offset,9).surge<0,"center approach direction");
  center.reset();check(!center.step(p,10).complete,"center single observation");
  check(!center.step(p,10.1).complete,"duplicate center observation");
  check(center.step(pose(1.5,1.5,10.4,2),10.4).complete,"center confirmation");
  check(center.step(pose(2.0,1.5,10.5,3),10.5).fault,"center clearance violated");
  check(!rise.begin(pose(.5,.5,10,1),.2,10),"unsafe ascent cell");
  check(rise.begin(p,.2,10),"ascent rejected");
  AscentStep step;double old=.2;
  for(int i=1;i<=30;++i) {
    step=rise.step(std::max(0.0,.2-.01*i),true,10+.1*i,1);
    check(!step.fault&&old-step.target_depth<=.010001,"ascent rate bound");old=step.target_depth;
  }
  check(!step.complete,"duplicate pressure broadcasts confirmed surface");
  step=rise.step(0,true,13.1,2);check(!step.complete,"two depth samples insufficient");
  step=rise.step(0,true,13.2,3);check(!step.complete,"old pre-surface sample was counted");
  step=rise.step(0,true,13.3,4);check(step.complete,"three actual depth samples should confirm");
  check(rise.step(0,false,13.4,5).fault,"unsafe ascent continued");
  std::vector<std::uint8_t> bytes(17,0);bytes[8]=1;
  auv_protocol_write_u32_le(bytes.data()+9,8);auv_protocol_write_u32_le(bytes.data()+13,2000);
  auto decoded=auv_stm32_bridge::decode_depth_sample(bytes);check(decoded.valid&&decoded.sensor_sequence==8&&decoded.age_sec==2,"sample telemetry");
  auv_stm32_bridge::DepthTelemetry ros;check(auv_stm32_bridge::decode_depth_telemetry(bytes,ros)&&ros.valid,"ROS extended depth compatibility");
  bytes.resize(9);check(!auv_stm32_bridge::decode_depth_sample(bytes).valid,"legacy lacks sample identity");
}
static void execution() {
  auto c=config();GridPlannerConfig pc;pc.forbid_target_reentry=true;
  auto plan=GridPlanner(pc).plan(grid((1<<0)|(1<<2)|(1<<6)|(1<<8)),{1,1,"empty"});
  SurfaceRouteExecutor executor(c);check(executor.set_route(plan),"route rejected");
  double row=1.5,col=1.5,time=20;std::uint64_t seq=1;std::size_t waypoint=0;std::set<int> entries,visits;bool complete=false;
  for(int i=0;i<1500;++i) {
    const auto p=pose(row,col,time,seq++);const auto out=executor.step(p,time);
    if(out.fault)throw std::runtime_error(out.detail);
    if(out.entered)entries.insert(out.entered->row*3+out.entered->col);
    if(out.visited)visits.insert(out.visited->row*3+out.visited->col);
    if(out.complete){complete=true;break;}
    waypoint=out.waypoint;
    if(waypoint<plan.path.size()) {
      row+=std::clamp(plan.path[waypoint].row+.5-row,-.02,.02);
      col+=std::clamp(plan.path[waypoint].col+.5-col,-.02,.02);
    }
    time+=.1;
  }
  check(complete&&visits.size()==4&&entries.size()>=4,"measured traversal incomplete");
  executor.set_route(plan);check(executor.step(pose(1.5,1.5,50,1),50).new_sample,"new frame");
  check(executor.step(pose(1.5,1.5,50,1),50.1).waypoint==0,"duplicate frame advanced waypoint");
  check(executor.step(pose(1.5,1.5,50,1),51).fault,"stale grid continued");
  executor.set_route(plan);executor.step(pose(1.5,1.5,60,1),60);
  check(executor.step(pose(.5,.5,60.1,2),60.1).fault,"jump accepted");
  executor.set_route(plan);executor.step(pose(1.5,1.5,70,1),70);
  auto flipped=pose(1.5,1.5,70.1,2);flipped.body_from_grid={0,-1,0,-1,0,0,0,0,-1};
  check(executor.step(flipped,70.1).fault,"reference flip accepted");
  auto duplicate=plan;duplicate.targets[1]=duplicate.targets[0];check(!executor.set_route(duplicate),"duplicate targets accepted");
  auto observed_plan=GridPlanner(pc).plan(grid((1<<0)|(1<<2)|(1<<6)|(1<<8)),{0,0,"circle_cone"});
  check(executor.set_route(observed_plan),"observed reentry fixture");
  time=80;seq=1;SurfaceRouteStep sample;
  for(int i=0;i<6;++i){sample=executor.step(pose(.5,.5,time,seq++),time);time+=.1;check(!sample.fault,"initial observed cone");}
  // Enter the next empty cell without reaching its center; return along the
  // same allowed corridor. This detects actual reentry, not path corruption.
  for(double x=.55;x<=1.100001;x+=.05){sample=executor.step(pose(.5,x,time,seq++),time);time+=.1;check(!sample.fault,"outbound measured corridor");}
  bool repeated=false;
  for(double x=1.05;x>=.89999;x-=.05) {
    sample=executor.step(pose(.5,x,time,seq++),time);time+=.1;
    if(sample.fault){repeated=sample.detail=="actual cone reentry detected";break;}
  }
  check(repeated,"actual repeated cone entry not detected");
  plan.path.push_back(plan.path.front());check(!executor.set_route(plan),"malformed path accepted");
}
static void fsm() {
  using namespace auv_mission;MissionFsmConfig c;c.surface_before_visit=true;c.allow_armed_during_observation=c.allow_armed_during_visit=true;
  MissionFsm m(c);m.command(MissionCommand::kStart,1);m.update_status(true,false,0,1);m.tick(1);
  m.update_status(true,true,0,1.1);m.update_apriltag(true,1.1);m.tick(1.1);
  m.update_map(true,false,1.2);m.tick(1.2);check(m.snapshot().phase==MissionPhase::kSurfaceForCones,"map bypassed ascent");
  m.update_route(true,true,1.3);m.update_surface_pose(true,1.3);m.tick(1.3);check(m.snapshot().phase==MissionPhase::kSurfaceForCones,"route bypassed ascent");
  m.update_surface(true,1.4);m.tick(1.4);check(m.snapshot().phase==MissionPhase::kRelocalizeSurface,"surface transition");
  m.tick(1.5);check(m.snapshot().phase==MissionPhase::kRelocalizeSurface,"old pose reused across ascent");
  m.update_surface_pose(true,1.6);m.tick(1.6);check(m.snapshot().phase==MissionPhase::kPlanCones,"surface pose transition");
  m.tick(1.7);check(m.snapshot().phase==MissionPhase::kVisitCones,"route transition");
  m.update_map(true,true,1.8);m.tick(1.8);check(m.snapshot().phase==MissionPhase::kComplete,"A2 stage complete");
}
int main(){try{planning();geometry();ascent();execution();fsm();std::cout<<"A2 geometry, ascent, 1134 layouts, observed traversal and FSM passed\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
