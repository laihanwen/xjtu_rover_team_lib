#include "../src/runtime_config.hpp"
#include "../src/yellow_edge_watch.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
static void check(bool v,const char* why){if(!v){std::cerr<<"FAILED: "<<why<<'\n';throw std::runtime_error(why);}}
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
  YellowEdgeWatch yellow_watch;
  cv::Mat stripe(240,320,CV_8UC3,cv::Scalar(15,15,15));
  cv::line(stripe,{35,170},{265,170},cv::Scalar(0,220,230),7);
  auto yellow=yellow_watch.update(stripe,{15,60,60},{40,255,255});
  check(yellow.candidate&&!yellow.confirmed,"single yellow stripe is only a candidate");
  yellow_watch.update(stripe,{15,60,60},{40,255,255});
  yellow=yellow_watch.update(stripe,{15,60,60},{40,255,255});
  check(yellow.newly_confirmed&&yellow.confirmed,"three yellow stripe frames latch observation cue");
  cv::Mat blank(240,320,CV_8UC3,cv::Scalar(15,15,15));
  YellowEdgeWatch no_yellow;
  check(!no_yellow.update(blank,{15,60,60},{40,255,255}).candidate,"dark floor is not yellow edge");
  check(c.depth_fresh(true,2,1.5,.1),"slow independent M10 sample stays fresh beyond image budget");
  check(!c.depth_fresh(true,4.1,1.5,0),"stale M10 sample rejected");
  check(!c.depth_fresh(false,2,1.5,0),"invalid M10 sample rejected");
  check(!c.depth_fresh(true,1,1.5,0)&&!c.depth_fresh(true,2,1.5,-.1),"invalid pressure time rejected");
  check(!c.depth_fresh(true,2,1.5,std::numeric_limits<double>::quiet_NaN()),"nonfinite pressure age rejected");
  auto bad_timeout=c;bad_timeout.depth_sample_timeout=3.1;
  bool timeout_rejected=false;try{bad_timeout.validate(false);}catch(const std::exception&){timeout_rejected=true;}
  check(timeout_rejected,"pressure timeout cannot exceed MCU freshness window");
  const std::vector<double> k{400,0,160,0,400,120,0,0,1};
  // Keep all four corners inside the image; the previous y=-.2 projection
  // placed its center above the frame and must be rejected by the border gate.
  auto a=dock_observation(project(c,{CV_PI,0,0},{-.1,-.06,.5}),1,1,{320,240},k,k,c,false);
  check(a.metric_valid&&std::abs(a.forward-.06)<1e-5&&std::abs(a.left-.1)<1e-5,"down optical/body conversion");
  auto f=dock_observation(project(c,{CV_PI/2,0,0},{.05,.2,1}),1,1,{320,240},k,k,c,true);
  check(f.metric_valid&&std::abs(f.forward-1)<1e-4&&std::abs(f.left+.05)<1e-4,"front metric target");
  auto corrected=c;corrected.front_body_bias={-.01,.02,.03};
  auto corrected_front=dock_observation(project(c,{CV_PI/2,0,0},{.05,.2,1}),1,1,{320,240},k,k,corrected,true);
  check(corrected_front.metric_valid&&std::abs(corrected_front.tag_forward-f.tag_forward+.01)<1e-5&&
        std::abs(corrected_front.tag_left-f.tag_left-.02)<1e-5,"accepted body correction reaches control observation");
  auto raw_probe=dock_geometry_probe(project(c,{CV_PI/2,0,0},{.05,.2,1}),1,1,{320,240},k,k,corrected,true);
  check(raw_probe.metric_valid&&std::abs(raw_probe.tag_forward-f.tag_forward)<1e-5,
        "diagnostic probe remains raw for browser correction");
  auto bad_bias=corrected;bad_bias.front_body_bias={.3,0,0};
  bool bias_rejected=false;try{bad_bias.validate(false);}catch(const std::exception&){bias_rejected=true;}
  check(bias_rejected,"out-of-range body correction rejected");
  auto missing_accepted_bias=c;missing_accepted_bias.geometry_user_accepted=true;
  bool missing_bias_rejected=false;try{missing_accepted_bias.validate(true);}catch(const std::exception&){missing_bias_rejected=true;}
  check(missing_bias_rejected,"accepted geometry needs measured front and down corrections");
  auto unverified=c;unverified.geometry_verified=false;
  check(!dock_observation(project(c,{CV_PI,0,0},{0,0,.5}),1,1,{320,240},k,k,unverified,false).metric_valid,"unverified geometry gate");
  auto probe=dock_geometry_probe(project(c,{CV_PI,0,0},{-.1,-.06,.5}),1,1,{320,240},k,k,unverified,false);
  check(probe.metric_valid&&std::abs(probe.tag_forward-.06)<1e-5&&std::abs(probe.tag_left-.1)<1e-5&&std::abs(probe.tag_up+.5)<1e-5,"diagnostic tag centre uses body coordinates");
  check(!unverified.geometry_verified,"diagnostic does not certify production config");
  auto invalid_probe_cfg=unverified;invalid_probe_cfg.down_rotation.pop_back();
  check(!dock_geometry_probe(project(c,{CV_PI,0,0},{0,0,.5}),1,1,{320,240},k,k,invalid_probe_cfg,false).metric_valid,"bad diagnostic transform rejected");
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
  // Measure loss from a processed missing observation, rather than assuming
  // the skipped simulation interval was observed by the controller.
  p.stamp=3.1+c.lost_timeout+.05;
  centered.step(p.stamp,.6,11,true,true,p,{},{});
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
  // A short front-to-down blind gap may advance slowly, but never beyond its
  // measured pose/time envelope or without a prior confirmed close front tag.
  auto blind_test=[&](double start){
    TagDockTask t(c);auv_control::ObservationPose q{true,true,7,start,0,0,0};
    check(t.start(start,.6,q),"blind fixture");t.phase=TagDockTask::Phase::Front;
    DockObservation close=front;close.forward=.2;
    for(int i=1;i<=3;++i){q.stamp=start+i*.05;close.stamp=q.stamp;close.sequence=i;
      t.step(q.stamp,.6,i,true,true,q,close,{});}
    return t;
  };
  auto blind=blind_test(10);auv_control::ObservationPose q{true,true,7,10.20,0,0,0};
  auto b=blind.step(10.20,.6,4,true,true,q,{},{});
  check(blind.phase==TagDockTask::Phase::Blind&&b.surge==0,"front loss enters neutral handoff tick");
  q.stamp=10.25;q.x=.01;b=blind.step(10.25,.6,5,true,true,q,{},{});
  check(b.surge>0&&b.surge<=c.blind_speed&&b.sway==0,"bounded low speed blind advance");
  down.id=18;for(int i=1;i<=3;++i){q.stamp=10.25+i*.05;q.x+=.01;down.stamp=q.stamp;down.sequence=i;
    b=blind.step(q.stamp,.6,5+i,true,true,q,{},down);}
  check(blind.phase==TagDockTask::Phase::Down,"confirmed down tag takes over blind advance");
  auto blind_limit=blind_test(20);q={true,true,7,20.20,0,0,0};
  blind_limit.step(20.20,.6,4,true,true,q,{},{});
  q.stamp=20.25;q.x=.13;b=blind_limit.step(20.25,.6,5,true,true,q,{},{});
  check(blind_limit.phase==TagDockTask::Phase::Fault&&b.surge==0,"blind distance limit stops and faults");
  auto blind_timeout=blind_test(30);q={true,true,7,30.20,0,0,0};
  blind_timeout.step(30.20,.6,4,true,true,q,{},{});
  q.stamp=31.21;b=blind_timeout.step(31.21,.6,5,true,true,q,{},{});
  check(blind_timeout.phase==TagDockTask::Phase::Fault&&b.surge==0,"blind time limit stops and faults");
  TagDockTask unseen(c);q={true,true,7,40,0,0,0};
  check(unseen.start(40,.6,q),"unseen fixture");unseen.phase=TagDockTask::Phase::Front;
  q.stamp=40.05;b=unseen.step(40.05,.6,1,true,true,q,{},{});
  check(unseen.phase!=TagDockTask::Phase::Blind&&b.surge==0,"no blind advance without confirmed front tag");
  auto blind_pose=blind_test(50);q={true,true,7,50.20,0,0,0};
  blind_pose.step(50.20,.6,4,true,true,q,{},{});
  q.stamp=50.25;q.valid=false;b=blind_pose.step(50.25,.6,5,true,true,q,{},{});
  check(blind_pose.phase==TagDockTask::Phase::Fault&&b.surge==0,"lost localization stops blind handoff");
  auto short_cfg=c;short_cfg.approach_only=true;short_cfg.depth_targets.clear();short_cfg.validate(true);
  TagDockTask short_task(short_cfg);q={true,true,7,60,0,0,0};
  check(short_task.start(60,.6,q),"approach-only starts without depth sweep");
  q.stamp=60.05;short_task.step(60.05,.6,1,true,true,q,{},{});
  check(short_task.phase==TagDockTask::Phase::Front,"approach-only enters front approach directly");
  down.id=18;for(int i=1;i<=3;++i){q.stamp=60.05+i*.05;down.stamp=q.stamp;down.sequence=i;
    b=short_task.step(q.stamp,.6,i+1,true,true,q,{},down);}
  check(short_task.phase==TagDockTask::Phase::Complete&&!short_task.active()&&b.surge==0&&b.sway==0,
        "approach-only stops when down camera confirms tag");
  auv_control::ObservationSearchConfig search_cfg;
  search_cfg.enabled=search_cfg.calibrated=true;
  search_cfg.maximum_radius_m=.5;search_cfg.maximum_speed=.04;
  search_cfg.depth_m=.6;search_cfg.waypoints={{.2,0}};
  auto search_dock=short_cfg;search_dock.radius=.5;
  TagDockTask wrong_depth(search_dock,search_cfg);
  q={true,true,7,69,0,0,0};check(!wrong_depth.start(69,.4,q),
    "measured search requires observed working depth before starting");
  TagDockTask searching(search_dock,search_cfg);
  q={true,true,7,70,0,0,0};check(searching.start(70,.6,q),"search start without tag");
  q.stamp=70.05;searching.step(q.stamp,.6,1,true,true,q,{},{});
  check(searching.phase==TagDockTask::Phase::Search,"measured search begins without initial tag");
  q.stamp=70.10;b=searching.step(q.stamp,.6,2,true,true,q,{},{});
  check(b.surge>0&&b.surge<=search_cfg.maximum_speed&&b.sway==0,"search target limited and directed to measured waypoint");
  q.stamp=70.15;front.stamp=q.stamp;front.sequence=1;
  searching.step(q.stamp,.6,3,true,true,q,front,{});
  q.stamp=70.20;front.stamp=q.stamp;front.sequence=2;
  searching.step(q.stamp,.6,4,true,true,q,front,{});
  q.stamp=70.25;front.stamp=q.stamp;front.sequence=3;
  b=searching.step(q.stamp,.6,5,true,true,q,front,{});
  check(searching.phase==TagDockTask::Phase::Front&&b.surge==0,"confirmed front tag stops search before approach");
  TagDockTask search_down(search_dock,search_cfg);
  q={true,true,7,80,0,0,0};check(search_down.start(80,.6,q),"down search fixture");
  q.stamp=80.05;search_down.step(q.stamp,.6,1,true,true,q,{},{});
  for(int i=1;i<=3;++i){q.stamp=80.05+i*.05;down.stamp=q.stamp;down.sequence=i;
    b=search_down.step(q.stamp,.6,i+1,true,true,q,{},down);}
  check(search_down.phase==TagDockTask::Phase::Complete&&b.surge==0,
        "down tag directly completes approach-only search");
  auto routed_cfg=search_dock;routed_cfg.finish_waypoints={{0,0}};
  TagDockTask routed(routed_cfg,search_cfg);
  q={true,true,7,90,.1,0,0};check(routed.start(90,.6,q),"routed start");
  q.stamp=90.05;routed.step(q.stamp,.6,1,true,true,q,{},{});
  for(int i=1;i<=3;++i){q.stamp=90.05+i*.05;down.stamp=q.stamp;down.sequence=i;
    b=routed.step(q.stamp,.6,i+1,true,true,q,{},down);}
  check(routed.phase==TagDockTask::Phase::Return&&b.surge==0,"confirmed down tag enters neutral return tick");
  q.x=.3;q.stamp=90.25;b=routed.step(q.stamp,.6,5,true,true,q,{},{});
  check(b.surge<0&&routed.phase==TagDockTask::Phase::Return,"return moves toward measured finish");
  q.x=.1;
  for(int i=1;i<=15&&routed.phase==TagDockTask::Phase::Return;++i){q.stamp=90.25+i*.05;
    b=routed.step(q.stamp,.6,5+i,true,true,q,{},{});}
  check(routed.phase==TagDockTask::Phase::Complete&&b.surge==0&&b.sway==0,
        "finish arrival completes and neutralizes route");
  auto home_cfg=short_cfg;home_cfg.radius=.5;home_cfg.return_to_origin=true;
  TagDockTask home(home_cfg);
  q={true,true,7,95,0,0,0};check(home.start(95,.6,q),"red-frame origin start");
  q.stamp=95.05;home.step(q.stamp,.6,1,true,true,q,{},{});
  q.x=.2;q.stamp=95.10;home.step(q.stamp,.6,2,true,true,q,{},{});
  for(int i=1;i<=3;++i){q.stamp=95.10+i*.05;down.stamp=q.stamp;down.sequence=i;
    b=home.step(q.stamp,.6,i+2,true,true,q,{},down);}
  check(home.phase==TagDockTask::Phase::Return&&b.surge==0,
        "down confirmation starts neutral retrace without finish coordinates");
  q.stamp=95.30;b=home.step(q.stamp,.6,6,true,true,q,{},{});
  check(b.surge<0,"retrace targets red-frame origin");
  q.x=0;for(int i=1;i<=15&&home.phase==TagDockTask::Phase::Return;++i){
    q.stamp=95.30+i*.05;b=home.step(q.stamp,.6,6+i,true,true,q,{},{});}
  check(home.phase==TagDockTask::Phase::Complete&&b.surge==0,
        "red-frame origin reached and outputs neutralized");
  auto observed_cfg=home_cfg;observed_cfg.observation_sec=.25;
  auto unsafe_observation=observed_cfg;unsafe_observation.observation_sec=121;
  bool observation_rejected=false;try{unsafe_observation.validate(true);}catch(const std::exception&){observation_rejected=true;}
  check(observation_rejected,"timed observation cannot run without bounded duration");
  TagDockTask observed(observed_cfg);
  down.pixel_error=0;down.forward=0;down.left=0;
  q={true,true,7,100,0,0,0};check(observed.start(100,.6,q),"timed observation start");
  q.stamp=100.05;observed.step(q.stamp,.6,1,true,true,q,{},{});
  q.x=.2;
  for(int i=1;i<=3;++i){q.stamp=100.05+i*.05;down.stamp=q.stamp;down.sequence=100+i;
    observed.step(q.stamp,.6,i+1,true,true,q,{},down);}
  check(observed.phase==TagDockTask::Phase::Down,"tag must be centered before observation");
  for(int i=1;i<=6&&observed.phase==TagDockTask::Phase::Down;++i){q.stamp+=.05;down.stamp=q.stamp;down.sequence=110+i;
    observed.step(q.stamp,.6,4+i,true,true,q,{},down);}
  check(observed.phase==TagDockTask::Phase::Hold,"centered tag starts timed observation");
  const double hold_started=q.stamp;
  while(q.stamp<hold_started+.2){q.stamp+=.05;down.stamp=q.stamp;++down.sequence;
    b=observed.step(q.stamp,.6,20,true,true,q,{},down);}
  check(observed.phase==TagDockTask::Phase::Hold&&b.surge==0,"hold stays neutral before timer expires");
  q.stamp=hold_started+.3;down.stamp=q.stamp;++down.sequence;
  b=observed.step(q.stamp,.6,21,true,true,q,{},down);
  check(observed.phase==TagDockTask::Phase::Return&&b.surge==0,"timed hold enters neutral return tick");
  auto unsafe_search=search_cfg;unsafe_search.maximum_radius_m=3;
  bool unsafe_rejected=false;try{TagDockTask unsafe(search_dock,unsafe_search);}catch(const std::exception&){unsafe_rejected=true;}
  check(unsafe_rejected,"search cannot exceed verified docking radius");
  auto unsafe_finish=search_dock;unsafe_finish.finish_waypoints={{.6,0}};
  bool finish_rejected=false;try{unsafe_finish.validate(true);}catch(const std::exception&){finish_rejected=true;}
  check(finish_rejected,"finish waypoint cannot exceed verified docking radius");
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
  auto cfg=load();check(!cfg.motion_enabled&&cfg.docking.approach_only&&cfg.docking.return_to_origin&&
                        cfg.docking.front_body_bias.size()==3&&cfg.docking.observation_sec==8,
                        "safe approach template retains measured correction");
  y["observation_search"]["enabled"]=true;
  y["observation_search"]["corridor_calibrated"]=true;
  y["observation_search"]["maximum_radius_m"]=.4;
  y["observation_search"]["maximum_speed"]=.04;
  y["observation_search"]["depth_target_m"]=.6;
  y["observation_search"]["waypoints_m"].push_back(std::vector<double>{.2,0});
  y["tag_docking"]["finish_waypoints_m"].push_back(std::vector<double>{0,0});
  auto route_cfg=load();
  check(route_cfg.search.enabled&&route_cfg.docking.finish_waypoints.size()==1,
        "measured shortest-route YAML is accepted without enabling motion");
  y["observation_search"]["maximum_radius_m"]=.6;
  bool route_rejected=false;try{load();}catch(const std::exception&){route_rejected=true;}
  check(route_rejected,"search YAML cannot exceed docking corridor");
  check(cfg.docking.depth_sample_timeout==2.5&&cfg.docking.frame_timeout==.3,"independent pressure/image YAML budgets");
  auto template_geometry=cfg.docking;template_geometry.geometry_verified=true;
  template_geometry.validate(false); // Valid transforms without certifying the actual rig.
  check(template_geometry.down_rotation.size()==9,"template down rotation is a 3x3 matrix");
  y["motion"]["motion_commands_enabled"]=true;
  bool rejected=false;try{load();}catch(const std::exception&){rejected=true;}
  check(rejected,"template cannot bypass commissioning");
  auto boot_yaml=YAML::LoadFile((std::filesystem::path(argv[1])/"runtime/config/pi-auv-boot-tag-route.yaml").string());
  y=boot_yaml;auto boot_config=load();
  check(boot_config.mission_profile=="boot_tag_route"&&boot_config.auto_arm&&boot_config.motion_enabled&&
        boot_config.boot_route.explicit_motion_authorization&&!boot_config.directions_calibrated&&
        !boot_config.limits_calibrated,"explicit boot trial retains unverified calibration flags");
  y["boot_tag_route"]["explicit_motion_authorization"]=false;
  bool unauthorized_rejected=false;try{load();}catch(const std::exception&){unauthorized_rejected=true;}
  check(unauthorized_rejected,"boot motion requires dedicated authorization");
  std::filesystem::remove(temp);std::cout<<"tag docking regression passed\n";
}
