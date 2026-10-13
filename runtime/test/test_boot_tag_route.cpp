#include "../src/boot_tag_route.hpp"
#include <iostream>
#include <stdexcept>

static void require(bool ok,const char* why){if(!ok){std::cerr<<"FAILED: "<<why<<'\n';throw std::runtime_error(why);}}
static BootTagRoute::Input sample(double now,double depth,std::uint64_t seq){
  BootTagRoute::Input x;x.now=now;x.depth=depth;x.depth_sequence=seq;
  x.yaw=.2;x.depth_fresh=x.status_safe=x.cameras_fresh=x.recording_ready=true;
  return x;
}
int main(){
  BootTagRoute route({});
  for(int i=1;i<=12;++i)require(!route.step(sample(i,-.18,i)).start,"air must not START");
  for(int i=1;i<=10;++i)require(!route.step(sample(12+i,.5,12+i)).start,"requires full stable interval");
  auto x=sample(23,.5,23);auto y=route.step(x);
  require(y.start&&y.arm&&route.phase==BootTagRoute::Phase::WaitArm,"underwater stability requests one ARM");
  require(!route.step(x).start,"one-shot start");
  x.now=23.1;x.armed=true;y=route.step(x);
  require(y.event=="ASCEND_START","ARM confirmation starts ascent");
  x.now=24;x.depth=.3;x.depth_sequence=24;y=route.step(x);
  require(y.event=="ASCEND_DONE"&&route.phase==BootTagRoute::Phase::Forward,"ascent then forward");
  const double forward_yaw=y.target_yaw;
  x.now=24.2;x.depth_sequence=25;y=route.step(x);
  require(y.surge>0,"forward target positive");
  x.now=25;x.depth_sequence=26;x.down_tag=true;y=route.step(x);
  require(y.event=="FORWARD_END"&&route.phase==BootTagRoute::Phase::Return&&route.open_loop_return,
    "down tag immediately stops and selects documented fallback");
  x.down_tag=false;x.now=25.2;x.depth_sequence=27;y=route.step(x);
  require(y.surge<0&&y.target_yaw==forward_yaw,"return uses reverse thrust and locked heading");
  x.now=27;x.depth_sequence=28;y=route.step(x);
  require(y.event=="RETURN_END"&&route.phase==BootTagRoute::Phase::Hold,"reverse duration completes");
  bool completed=false;
  for(int i=0;i<4;++i){x.now=27.5+i;x.depth_sequence=29+i;y=route.step(x);completed|=y.event=="HOLD_STABLE"&&y.disarm;}
  require(completed&&route.phase==BootTagRoute::Phase::Done,"stable hold DISARMs");
  BootTagRoute bad({});x=sample(1,.5,1);
  for(int i=1;i<=12;++i){x.now=i;x.depth_sequence=i;bad.step(x);}
  x.now=12.1;x.armed=true;bad.step(x);x.now=13;x.depth=.3;bad.step(x);
  x.now=13.1;x.yaw=1; y=bad.step(x);
  require(y.disarm&&bad.phase==BootTagRoute::Phase::Fault,"yaw deviation faults");
  BootTagRoute stale({});x=sample(1,.5,1);
  for(int i=1;i<=12;++i){x.now=i;x.depth_sequence=i;stale.step(x);}
  x.now=12.1;x.depth_fresh=false;y=stale.step(x);
  require(y.disarm&&stale.phase==BootTagRoute::Phase::Fault,"depth loss after start faults");
  BootTagRoute no_tag({});x=sample(1,.5,1);
  for(int i=1;i<=12;++i){x.now=i;x.depth_sequence=i;no_tag.step(x);}
  x.now=12.1;x.armed=true;no_tag.step(x);
  x.now=13;x.depth=.3;no_tag.step(x);
  x.now=28;x.depth_sequence=30;y=no_tag.step(x);
  require(y.event=="FORWARD_END"&&no_tag.phase==BootTagRoute::Phase::Return,
    "no tag at 15 seconds returns rather than continuing forward");
  BootTagRoute runaway({});x=sample(1,.5,1);
  for(int i=1;i<=12;++i){x.now=i;x.depth_sequence=i;runaway.step(x);}
  x.now=12.1;x.armed=true;runaway.step(x);
  x.now=13;x.depth=.3;x.pose_valid=true;runaway.step(x);
  x.now=13.1;x.pose_valid=true;x.x=.6;y=runaway.step(x);
  require(y.disarm&&runaway.phase==BootTagRoute::Phase::Fault,"displacement limit faults");
  BootTagRoute measured({});x=sample(1,.5,1);x.pose_valid=true;
  for(int i=1;i<=12;++i){x.now=i;x.depth_sequence=i;measured.step(x);}
  x.now=12.1;x.armed=true;measured.step(x);
  x.now=13;x.depth=.3;measured.step(x);
  x.now=13.5;x.x=.2;x.front_tag=true;y=measured.step(x);
  require(y.event=="FORWARD_END"&&!measured.open_loop_return,"fresh measured pose selected");
  x.front_tag=false;x.now=13.6;x.x=.18;y=measured.step(x);
  require(y.surge<0,"measured return commands reverse without yaw turn");
  x.now=13.7;x.x=0;y=measured.step(x);
  require(y.event=="RETURN_END"&&measured.phase==BootTagRoute::Phase::Hold,
          "measured origin closes return");
  BootTagRoute timed_out({});x=sample(1,.5,1);
  for(int i=1;i<=12;++i){x.now=i;x.depth_sequence=i;timed_out.step(x);}
  x.now=12.1;x.armed=true;timed_out.step(x);
  x.now=13;x.depth=.3;timed_out.step(x);
  x.now=13.5;x.front_tag=true;timed_out.step(x);
  x.front_tag=false;x.now=40;y=timed_out.step(x);
  require(y.disarm&&timed_out.phase==BootTagRoute::Phase::Fault,"return watchdog faults");
  std::cout<<"boot route state and fault checks passed\n";
}
