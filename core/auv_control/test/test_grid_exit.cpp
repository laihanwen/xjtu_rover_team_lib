#include "auv_control/grid_exit.hpp"
#include "auv_mission/mission_fsm.hpp"
#include <stdexcept>
static void check(bool v,const char* reason){if(!v)throw std::runtime_error(reason);}
int main() {
  auv_control::GridExitConfig c;c.enabled=c.verified=true;c.allowed_sides=1;
  c.cell_size_m=.82;c.body_radius_m=.1;c.clearance_m=.05;
  auv_mapping::MetricGridPose p;p.valid=p.surface_frame=p.landmark_observation=true;p.stamp=1;p.sequence=1;
  p.row=p.col=.5;p.body_from_grid={1,0,0,0,1,0,0,0,1};
  std::array<bool,9> empty{};empty.fill(true);unsigned visited=1U|(1U<<2)|(1U<<5)|(1U<<8);
  auv_control::GridExit exit(c);check(exit.begin(p,empty,visited,1),"last cone may depart boundary");
  check(exit.step(p,1.1).surge==0,"old-stage frame cannot move");
  bool complete=false;double now=1.1;
  for(int i=0;i<600&&!complete;++i) {
    p.stamp=now;p.sequence++;auto step=exit.step(p,now);check(!step.fault,"dynamic exit must stay in corridor");
    p.row+=step.sway*.1/c.cell_size_m;p.col+=step.surge*.1/c.cell_size_m;complete=step.complete;now+=.1;
  }
  check(complete&&p.row<=-(c.body_radius_m+c.clearance_m)/c.cell_size_m,"whole body confirmed beyond boundary");
  p.row=p.col=1.5;p.stamp=now;p.sequence++;
  empty.fill(false);visited=(1U<<4)|(1U<<0)|(1U<<2)|(1U<<8);
  check(!exit.begin(p,empty,visited,now),"blocked exit must not cross visited cones");
  empty.fill(true);p.row=2.15;p.col=1.5;p.surface_frame=false;p.stamp=now;p.sequence++;
  check(exit.begin_transit(p,empty,4,now),"partial map can route to ascent cell");
  complete=false;
  for(int i=0;i<600&&!complete;++i) {
    now+=.1;p.stamp=now;p.sequence++;auto step=exit.step(p,now);
    check(!step.fault,"ascent transit remains in empty corridor");
    p.row+=step.sway*.1/c.cell_size_m;p.col+=step.surge*.1/c.cell_size_m;complete=step.complete;
  }
  check(complete&&std::hypot(p.row-1.5,p.col-1.5)<.08,"ascent destination measured");
  auv_mission::MissionFsmConfig config;config.surface_before_visit=config.exit_after_visit=true;
  auv_mission::MissionFsm fsm(config);using Phase=auv_mission::MissionPhase;
  fsm.update_status(true,false,0,1);fsm.command(auv_mission::MissionCommand::kStart,1);fsm.tick(1);
  fsm.update_apriltag(true,1.1);fsm.tick(1.1);fsm.update_map(true,false,1.2);fsm.tick(1.2);
  fsm.update_surface(true,1.3);fsm.tick(1.3);fsm.update_surface_pose(true,1.4);fsm.tick(1.4);
  fsm.update_route(true,true,1.5);fsm.tick(1.5);fsm.update_exit(true,1);
  fsm.update_map(true,true,1.6);fsm.tick(1.6);check(fsm.snapshot().phase==Phase::kExitGrid,"four cones are not task completion");
  fsm.tick(1.7);check(fsm.snapshot().phase==Phase::kExitGrid,"old exit evidence rejected");
  fsm.update_exit(true,1.8);fsm.tick(1.8);check(fsm.snapshot().phase==Phase::kComplete,"fresh exit evidence completes task");
}
