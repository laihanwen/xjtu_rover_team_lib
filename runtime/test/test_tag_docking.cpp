#include "../src/runtime_config.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
static void check(bool v,const char* why){if(!v)throw std::runtime_error(why);}
static TagDockConfig fixture() {
  TagDockConfig c;c.geometry_verified=c.corridor_verified=c.depth_verified=true;
  c.radius=2;c.depth_targets={.61,.60,.605};c.depth_rate=.1;c.depth_hold=.1;
  c.front_rotation={0,0,1,-1,0,0,0,-1,0};c.down_rotation={0,-1,0,-1,0,0,0,0,-1};
  c.front_offset=c.down_offset={0,0,0};c.center_hold=.2;return c;
}
static auv_vision::AprilTagObservation project(const TagDockConfig& c,cv::Vec3d r,cv::Vec3d t) {
  const double h=c.size_m/2;
  std::vector<cv::Point3d> object{{-h,h,0},{h,h,0},{h,-h,0},{-h,-h,0}};
  cv::Matx33d k(400,0,160,0,400,120,0,0,1);std::vector<cv::Point2d> pixels;
  cv::projectPoints(object,r,t,k,cv::Mat(),pixels);
  auv_vision::AprilTagObservation o;o.id=18;
  for(int i=0;i<4;++i){o.corners[i]=pixels[i];o.center+=o.corners[i]*.25f;}return o;
}
int main(int argc,char** argv) {
  check(argc==2,"source directory");auto c=fixture();c.validate(true);
  const std::vector<double> k{400,0,160,0,400,120,0,0,1};
  auto a=dock_observation(project(c,{CV_PI,0,0},{-.1,-.2,.5}),1,1,{320,240},k,k,c,false);
  check(a.metric_valid&&std::abs(a.forward-.2)<1e-5&&std::abs(a.left-.1)<1e-5,"down optical/body conversion");
  auto f=dock_observation(project(c,{CV_PI/2,0,0},{.05,.2,1}),1,1,{320,240},k,k,c,true);
  check(f.metric_valid&&std::abs(f.forward-1)<1e-4&&std::abs(f.left+.05)<1e-4,"front metric target");
  auto unverified=c;unverified.geometry_verified=false;
  check(!dock_observation(project(c,{CV_PI,0,0},{0,0,.5}),1,1,{320,240},k,k,unverified,false).metric_valid,"unverified geometry gate");
  auto tiny=project(c,{CV_PI,0,0},{0,0,10});
  check(!dock_observation(tiny,1,1,{320,240},k,k,c,false).metric_valid,"tiny tag cannot drive motion");
  auto clipped=project(c,{CV_PI,0,0},{-.19,0,.5});
  check(!dock_observation(clipped,1,1,{320,240},k,k,c,false).metric_valid,"border-clipped tag cannot drive motion");
  TagDockTask task(c);auv_control::ObservationPose p{true,true,7,1,0,0,0};
  check(task.start(1,.6,p),"start");
  double now=1,depth=.6;std::uint64_t sensor=1,seq=1;int reports=0;
  auto tick=[&](DockObservation front={},DockObservation down={},bool ready=true){
    now+=.05;p.stamp=now;front.stamp=down.stamp=now;
    front.sequence=down.sequence=++seq;
    auto output=task.step(now,depth,++sensor,true,ready,p,front,down);depth=output.depth;
    reports+=output.report;return output;
  };
  tick();for(int i=0;i<100&&task.phase==TagDockTask::Phase::Depth;++i)tick();
  check(task.phase==TagDockTask::Phase::Front&&reports==3,"three settled depth reports");
  DockObservation front;front.detected=front.metric_valid=true;front.id=18;front.forward=.5;
  auto output=tick(front);check(output.surge==0,"multi-frame front gate");tick(front);output=tick(front);
  check(output.surge>0&&output.surge<=c.speed,"bounded front approach");
  check(output.surge<=c.target_rate*.05+1e-9,"first approach target obeys slew limit");
  output=tick();check(output.surge==0&&output.sway==0,"loss immediately stops lateral motion");
  // Reacquire before timeout and hand over only to the same ID in down view.
  output=tick(front);check(output.surge==0,"reacquisition requires new multi-frame confirmation");
  for(int i=0;i<2;++i)tick(front);
  DockObservation down=front;down.pixel_error=30;down.forward=.1;down.left=-.05;
  down.id=19;for(int i=0;i<4;++i)tick(front,down);
  check(task.phase==TagDockTask::Phase::Front,"wrong down ID cannot hand over");
  down.id=18;for(int i=0;i<3;++i)output=tick(front,down);
  check(task.phase==TagDockTask::Phase::Down&&output.sway<0,"down handover and signed correction");
  down.pixel_error=0;for(int i=0;i<8;++i)tick({},down);
  check(task.phase==TagDockTask::Phase::Hold&&task.active(),"center enters active hover");
  output=tick({},down);check(output.surge==0&&output.sway==0&&std::abs(output.depth-.605)<1e-6,"centered hover maintains working depth");
  down.pixel_error=12;output=tick({},down);check(output.surge==0&&output.sway==0,"hover hysteresis rejects center jitter");
  down.pixel_error=30;output=tick({},down);
  check(output.surge>0&&output.sway<0&&task.phase==TagDockTask::Phase::Hold,"hover corrects renewed position drift");
  check(std::hypot(output.surge,output.sway)<=c.hover_speed,"hover target magnitude cap");
  now=1+c.timeout+1;output=tick({},down);
  check(task.phase==TagDockTask::Phase::Hold,"approach timeout does not terminate established hover");
  output=tick();check(output.surge==0&&output.sway==0,"hover target loss stops lateral correction");
  now+=c.lost_timeout;tick();check(task.phase==TagDockTask::Phase::Fault,"hover target loss disarms via fault");
  check(!task.start(now,depth,p),"no automatic restart");
  // Stale/repeated centered observations cannot supply a two-second hold.
  TagDockTask centered(c);p={true,true,7,1,0,0,0};check(centered.start(1,.6,p),"center fixture");
  centered.phase=TagDockTask::Phase::Down;
  down.stamp=1;down.sequence=1;down.pixel_error=0;
  for(int i=1;i<=6;++i){p.stamp=1+i*.05;centered.step(p.stamp,.6,i,true,true,p,{},down);}
  check(centered.phase!=TagDockTask::Phase::Hold,"repeated centered frame cannot enter hover");
  p.stamp=3.1;centered.step(3.1,.6,10,true,true,p,{},{});
  check(centered.phase==TagDockTask::Phase::Fault,"tag loss timeout");
  TagDockTask disarmed(c);p={true,true,7,1,0,0,0};check(disarmed.start(1,.6,p),"disarm fixture");
  p.stamp=1.05;disarmed.step(1.05,.6,1,true,true,p,{},{});
  p.stamp=1.1;disarmed.step(1.1,.6,2,false,true,p,{},{});
  check(disarmed.phase==TagDockTask::Phase::Fault,"unexpected DISARM terminates task");
  TagDockTask already_under(c);p={true,true,7,1,0,0,0};
  check(already_under.start(1,.6,p),"already-under fixture");already_under.phase=TagDockTask::Phase::Front;
  down.pixel_error=30;
  for(int i=1;i<=3;++i){p.stamp=1+i*.05;down.stamp=p.stamp;down.sequence=i;
    already_under.step(p.stamp,.6,i,true,true,p,{},down);}
  check(already_under.phase==TagDockTask::Phase::Down,"verified down tag takes over without front reacquisition");
  for(int failure=0;failure<4;++failure) {
    TagDockTask t(c);p={true,true,7,1,0,0,0};check(t.start(1,.6,p),"failure fixture");
    p.stamp=1.05;t.step(1.05,.6,1,true,true,p,{},{});
    p.stamp=1.1;if(failure==0)p.session=8;if(failure==1)p.x=3;
    t.step(1.1,failure==2?1.21:.6,2,true,failure!=3,p,{},{});
    check(t.phase==TagDockTask::Phase::Fault,"session/radius/depth/stale fault");
  }
  // Repeated depth sequence cannot complete a settling stage.
  TagDockTask frozen(c);p={true,true,7,1,0,0,0};check(frozen.start(1,.61,p),"frozen fixture");
  for(int i=1;i<30;++i){p.stamp=1+i*.05;frozen.step(p.stamp,.61,1,true,true,p,{},{});}
  check(frozen.phase==TagDockTask::Phase::Depth&&frozen.report_samples==0,"independent pressure samples");
  auto y=YAML::LoadFile((std::filesystem::path(argv[1])/"runtime/config/pi-auv-tag-docking.yaml").string());
  const auto temp=std::filesystem::temp_directory_path()/"tag-docking-config-test.yaml";
  auto load=[&](){std::ofstream out(temp);out<<y;out.close();return load_config(temp.string());};
  auto cfg=load();check(!cfg.motion_enabled&&cfg.docking.depth_targets==std::vector<double>{.8,.6,.7},"safe pool template");
  y["motion"]["motion_commands_enabled"]=true;
  bool rejected=false;try{load();}catch(const std::exception&){rejected=true;}
  check(rejected,"template cannot bypass commissioning");
  std::filesystem::remove(temp);std::cout<<"tag docking regression passed\n";
}
