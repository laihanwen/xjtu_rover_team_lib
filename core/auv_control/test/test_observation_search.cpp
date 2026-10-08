#include "auv_control/observation_search.hpp"
#include "auv_mission/mission_fsm.hpp"
#include <stdexcept>
void test_single_edge_grid();
static void require(bool x){if(!x)throw std::runtime_error("observation search regression");}
int main(){
  using namespace auv_control;
  ObservationSearchConfig cfg;cfg.enabled=cfg.calibrated=true;
  cfg.maximum_radius_m=3;cfg.depth_m=.3;cfg.waypoints={{1,0},{1,1}};
  ObservationSearch search(cfg);
  ObservationPose pose{true,true,1,10,0,0,std::acos(-1)/2};
  auto step=search.step(pose,10,false);
  require(std::abs(step.surge)<1e-8 && step.sway<0 && !step.fault);
  require(search.step(pose,11,false).fault); // Old pose never drives a search.
  pose.stamp=11;pose.session=2;require(search.step(pose,11,false).fault);
  search.reset();pose.session=1;pose.x=4;require(search.step(pose,11,false).fault);
  search.reset();pose.x=0;step=search.step(pose,11,true);require(step.surge==0 && step.sway==0);
  pose.stamp=12;pose.x=.5;step=search.step(pose,12,true);require(step.sway>0);
  cfg.waypoints={{4,0}};bool rejected=false;try{ObservationSearch invalid(cfg);}catch(const std::invalid_argument&){rejected=true;}require(rejected);
  using namespace auv_mission;
  MissionFsmConfig m;m.stop_after_map=true;m.allow_armed_during_observation=true;
  MissionFsm f(m);require(f.command(MissionCommand::kStart,1).accepted);
  f.update_status(true,false,0,1);f.tick(1);require(f.snapshot().phase==MissionPhase::kSearchAprilTag);
  f.update_status(true,true,0,1.1);f.update_map(true,false,1.1);f.tick(1.1);
  require(f.snapshot().phase==MissionPhase::kSearchAprilTag); // Map cannot bypass tag trigger.
  f.update_apriltag(true,1.2);f.tick(1.2);require(f.snapshot().phase==MissionPhase::kBuildMap);
  f.tick(1.3);require(f.snapshot().phase==MissionPhase::kComplete);
  MissionFsm strict;strict.command(MissionCommand::kStart,1);strict.update_status(true,false,0,1);strict.tick(1);
  strict.update_status(true,true,0,1.1);strict.tick(1.1);require(strict.snapshot().phase==MissionPhase::kFault);
  test_single_edge_grid();
}
