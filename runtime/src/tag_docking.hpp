#pragma once
#include "auv_control/observation_search.hpp"
#include "auv_vision/apriltag_detector.hpp"
#include <opencv2/calib3d.hpp>
#include <limits>

// X forward, Y left, Z up. Transforms must be measured optical-to-body rotations.
struct TagDockConfig {
  int id{18}, votes{3};
  double size_m{.138}, max_depth{1.2}, min_depth{0.1};
  double depth_rate{.03}, depth_tolerance{.03}, depth_hold{2}, depth_timeout{30};
  double speed{.08}, gain{.3}, radius{0}, timeout{90}, lost_timeout{2};
  double down_speed{.04}, hover_speed{.03}, target_rate{.08}, resume_px{15};
  double minimum_edge_px{8}, border_px{3};
  double frame_timeout{.3}, reprojection{2}, center_px{10}, center_hold{2}, max_tilt{.15};
  bool geometry_verified{}, corridor_verified{}, depth_verified{};
  std::vector<double> depth_targets, front_rotation, front_offset, down_rotation, down_offset;
  void validate(bool motion) const {
    for(double v:{size_m,max_depth,min_depth,depth_rate,depth_tolerance,depth_hold,depth_timeout,
        speed,gain,radius,timeout,lost_timeout,frame_timeout,reprojection,center_px,center_hold,max_tilt,
        down_speed,hover_speed,target_rate,resume_px,minimum_edge_px,border_px})
      if(!std::isfinite(v))throw std::runtime_error("nonfinite tag docking parameter");
    if(id<0||votes<2||size_m<=0||min_depth<0||max_depth<=min_depth||max_depth>1.2||
       depth_rate<=0||depth_rate>.1||depth_tolerance<=0||depth_hold<=0||depth_timeout<=depth_hold||
       speed<=0||speed>.2||gain<=0||radius<0||timeout<=0||lost_timeout<=frame_timeout||
       frame_timeout<=0||reprojection<=0||center_px<=0||center_hold<=0||max_tilt<=0||max_tilt>.3)
      throw std::runtime_error("invalid tag docking limits");
    if(down_speed<=0||down_speed>speed||hover_speed<=0||hover_speed>down_speed||target_rate<=0||
       target_rate>.5||resume_px<=center_px||minimum_edge_px<4||border_px<0)
      throw std::runtime_error("invalid docking approach/hover quality and slew limits");
    for(double d:depth_targets)if(!std::isfinite(d)||d<min_depth||d>max_depth)
      throw std::runtime_error("tag depth target outside allowed envelope");
    for(const auto* a:{&front_rotation,&front_offset,&down_rotation,&down_offset})
      for(double v:*a)if(!std::isfinite(v))throw std::runtime_error("nonfinite camera transform");
    if(geometry_verified||motion) {
      for(const auto* a:{&front_rotation,&down_rotation}) {
        if(a->size()!=9)throw std::runtime_error("tag docking needs both camera rotations");
        cv::Mat r(3,3,CV_64F,const_cast<double*>(a->data()));
        if(cv::norm(r*r.t()-cv::Mat::eye(3,3,CV_64F))>1e-6||std::abs(cv::determinant(r)-1)>1e-6)
          throw std::runtime_error("camera transform must be a proper rotation");
      }
      if(front_offset.size()!=3||down_offset.size()!=3)
        throw std::runtime_error("tag docking needs measured camera offsets");
    }
    if(motion&&(!geometry_verified||!corridor_verified||!depth_verified||radius<=0||depth_targets.size()<3))
      throw std::runtime_error("tag docking motion requires geometry, corridor, depth calibration and three depth targets");
    if(motion) {
      // Explicit down/up/working-depth sequence; no guessed excursion.
      if(!(depth_targets[0]>depth_targets[1])||depth_targets.size()!=3)
        throw std::runtime_error("depth test requires deeper, shallower, working depth targets");
      double travel=0;for(std::size_t i=1;i<depth_targets.size();++i)travel+=std::abs(depth_targets[i]-depth_targets[i-1])/depth_rate;
      if(timeout<=travel+depth_targets.size()*depth_hold)
        throw std::runtime_error("task timeout too short for depth sweep");
    }
  }
};
struct DockObservation {
  bool detected{}, metric_valid{};
  std::string quality_reason{"tag_not_detected"};
  int id{-1}; std::uint64_t sequence{}; double stamp{}, pixel_error{};
  double forward{}, left{}, reprojection{};
};
// Caller supplies already rectified pixels and their original K: zero D here.
inline DockObservation dock_observation(const auv_vision::AprilTagObservation& tag,
    std::uint64_t seq,double stamp,cv::Size dimensions,const std::vector<double>& k,
    const std::vector<double>& down_k,const TagDockConfig& cfg,bool front) {
  DockObservation out;out.detected=true;out.id=tag.id;out.sequence=seq;out.stamp=stamp;
  out.quality_reason="pixel_quality_rejected";
  out.pixel_error=std::hypot(tag.center.x-dimensions.width*.5,tag.center.y-dimensions.height*.5);
  // Full decoded square required: tiny foreshortened edges and clipped corners
  // are unsuitable for control even when a detector returns the expected ID.
  for(std::size_t i=0;i<4;++i) {
    const auto& p=tag.corners[i];const auto& q=tag.corners[(i+1)%4];
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||p.x<cfg.border_px||p.y<cfg.border_px||
       p.x>=dimensions.width-cfg.border_px||p.y>=dimensions.height-cfg.border_px||
       cv::norm(p-q)<cfg.minimum_edge_px)return out;
  }
  if(!cfg.geometry_verified||k.size()!=9||down_k.size()!=9){out.quality_reason="geometry_unverified";return out;}
  out.quality_reason="floor_pose_or_reprojection_rejected";
  const double h=cfg.size_m*.5;
  const std::vector<cv::Point3d> object{{-h,h,0},{h,h,0},{h,-h,0},{-h,-h,0}};
  const std::vector<cv::Point2f> pixels(tag.corners.begin(),tag.corners.end());
  cv::Mat camera(3,3,CV_64F,const_cast<double*>(k.data()));
  std::vector<cv::Mat> rotations,translations;
  cv::solvePnPGeneric(object,pixels,camera,cv::Mat(),rotations,translations,false,cv::SOLVEPNP_IPPE_SQUARE);
  // IPPE's Rodrigues conversion is ill-conditioned near an exact 180-degree
  // frontoparallel pose. Keep a homography-initialized iterative candidate too;
  // every candidate must still pass positive depth, floor normal and pixel error.
  cv::Mat iterative_r,iterative_t;
  if(cv::solvePnP(object,pixels,camera,cv::Mat(),iterative_r,iterative_t,false,cv::SOLVEPNP_ITERATIVE)) {
    rotations.push_back(iterative_r);translations.push_back(iterative_t);
  }
  const auto& rv=front?cfg.front_rotation:cfg.down_rotation;
  const auto& offset=front?cfg.front_offset:cfg.down_offset;
  cv::Matx33d rotation(rv.data());
  double best=std::numeric_limits<double>::infinity();cv::Vec3d target,normal;
  for(std::size_t i=0;i<translations.size();++i) {
    cv::Mat r;cv::Rodrigues(rotations[i],r);cv::Vec3d t(translations[i]);bool positive=true;
    for(const auto& p:object)if((r.at<double>(2,0)*p.x+r.at<double>(2,1)*p.y+t[2])<=0)positive=false;
    if(!positive)continue;
    const cv::Vec3d body_normal=rotation*cv::Vec3d(r.at<double>(0,2),r.at<double>(1,2),r.at<double>(2,2));
    if(std::abs(body_normal[2])<std::cos(2*cfg.max_tilt))continue; // Pool-floor tag, not a vertical decoy.
    std::vector<cv::Point2d> projected;cv::projectPoints(object,rotations[i],translations[i],camera,cv::Mat(),projected);
    double error=0;for(std::size_t j=0;j<4;++j)error+=cv::norm(projected[j]-cv::Point2d(pixels[j]))*cv::norm(projected[j]-cv::Point2d(pixels[j]));
    error=std::sqrt(error/4);if(error<best){best=error;target=t;normal=body_normal;}
  }
  if(!std::isfinite(best)||best>cfg.reprojection)return out;
  const cv::Vec3d body=rotation*target+cv::Vec3d(offset[0],offset[1],offset[2]);
  // Down camera's image center need not equal its principal point. Intersect its
  // center ray with the observed pool-floor plane, at the current camera height.
  const cv::Vec3d ray=cv::Matx33d(cfg.down_rotation.data())*
      cv::Vec3d((dimensions.width*.5-down_k[2])/down_k[0],
                (dimensions.height*.5-down_k[5])/down_k[4],1);
  if(ray[2]>=-1e-6||body[2]>=cfg.down_offset[2]||std::abs(normal.dot(ray))<1e-6)return out;
  const cv::Vec3d down_offset(cfg.down_offset[0],cfg.down_offset[1],cfg.down_offset[2]);
  const double distance=normal.dot(body-down_offset)/normal.dot(ray);
  if(distance<=0)return out;
  out.forward=body[0]-cfg.down_offset[0]-distance*ray[0];
  out.left=body[1]-cfg.down_offset[1]-distance*ray[1];out.reprojection=best;
  out.metric_valid=std::isfinite(out.forward)&&std::isfinite(out.left);
  if(out.metric_valid)out.quality_reason="valid";return out;
}

class TagDockTask {
 public:
  enum class Phase {Idle,WaitArm,Depth,Front,Down,Hold,Fault};
  struct Output {double surge{},sway{},depth{}; bool report{};};
  explicit TagDockTask(TagDockConfig c):cfg(std::move(c)){}
  const char* name() const {
    switch(phase){case Phase::Idle:return "INIT";case Phase::WaitArm:return "WAIT_ARM";
      case Phase::Depth:return "DEPTH_TEST";case Phase::Front:return "FRONT_APPROACH";
      case Phase::Down:return "DOWN_CENTER";case Phase::Hold:return "HOVER";default:return "FAULT";}
  }
  bool active()const{return phase==Phase::Depth||phase==Phase::Front||phase==Phase::Down||phase==Phase::Hold;}
  bool start(double now,double depth,const auv_control::ObservationPose& p) {
    if(phase!=Phase::Idle||cfg.depth_targets.size()!=3||!p.valid||!p.continuous||!p.session||
       !std::isfinite(depth)||depth<cfg.min_depth||depth>cfg.max_depth)return false;
    phase=Phase::WaitArm;started=entered=last_tick=now;target_depth=depth;
    origin_x=p.x;origin_y=p.y;session=p.session;return true;
  }
  void fail(const std::string& why){phase=Phase::Fault;reason=why;}
  Output step(double now,double depth,std::uint64_t depth_seq,bool armed,bool ready,
      const auv_control::ObservationPose& p,const DockObservation& front,const DockObservation& down) {
    Output out;out.depth=target_depth;
    const double previous_surge=applied_surge,previous_sway=applied_sway;
    applied_surge=applied_sway=0; // Every early return is an immediate lateral stop.
    if(phase==Phase::Idle||phase==Phase::Fault)return out;
    const double dt=std::clamp(now-last_tick,0.,.1);last_tick=now;
    if(!std::isfinite(depth)||depth<cfg.min_depth||depth>cfg.max_depth)fail("depth outside allowed envelope");
    else if(!p.valid||!p.continuous||p.session!=session||!std::isfinite(p.x)||!std::isfinite(p.y)||
       !std::isfinite(p.yaw)||!std::isfinite(p.stamp))fail("localization invalid or origin continuity lost");
    else if(now<p.stamp||now-p.stamp>cfg.frame_timeout)fail("localization observation stale");
    else if(std::hypot(p.x-origin_x,p.y-origin_y)>cfg.radius)fail("position outside measured safe corridor");
    else if(phase!=Phase::Hold&&now-started>cfg.timeout)fail("approach task timeout");
    else if(!ready)fail("telemetry, pressure, recording, camera or attitude safety gate");
    if(phase==Phase::Fault)return out;
    if(phase==Phase::WaitArm){if(armed){phase=Phase::Depth;entered=now;}else if(now-entered>5)fail("ARM acknowledgement timeout");return out;}
    if(!armed){fail("unexpected DISARM during task");return out;}
    if(phase==Phase::Depth) {
      if(now-entered>cfg.depth_timeout){fail("depth stage timeout");return out;}
      const double goal=cfg.depth_targets[depth_index];
      target_depth+=std::clamp(goal-target_depth,-cfg.depth_rate*dt,cfg.depth_rate*dt);out.depth=target_depth;
      // Require independent sensor updates, never count repeated STATUS as samples.
      if(depth_seq!=last_depth_seq) {
        last_depth_seq=depth_seq;
        if(std::abs(goal-depth)<=cfg.depth_tolerance&&std::abs(goal-target_depth)<1e-6){
          if(!stable_since)stable_since=now;
          sample_sum+=depth;++sample_count;sample_min=std::min(sample_min,depth);sample_max=std::max(sample_max,depth);
          if(sample_count>=3&&now-stable_since>=cfg.depth_hold) {
            report_target=goal;report_mean=sample_sum/sample_count;report_min=sample_min;report_max=sample_max;
            report_samples=sample_count;out.report=true;clear_stability();entered=now;
            if(++depth_index==cfg.depth_targets.size()){phase=Phase::Front;last_visible=now;}
          }
        } else clear_stability();
      }
      return out;
    }
    const bool front_fresh=fresh(front,now),down_fresh=fresh(down,now);
    count_votes(front,front_fresh,front_sequence,front_votes);
    count_votes(down,down_fresh,down_sequence,down_votes);
    // The same tag may already be under the robot after its depth sweep. Do not
    // insist on seeing it in front again before accepting verified down frames.
    if(phase==Phase::Front&&down_votes>=cfg.votes){phase=Phase::Down;stable_since=0;}
    const bool use_down=phase==Phase::Down||phase==Phase::Hold;
    const auto& observed=use_down?down:front;
    const bool valid=use_down?down_fresh:front_fresh;
    if(!valid){stable_since=0;if(now-last_visible>cfg.lost_timeout)fail("target lost; lateral motion stopped");return out;}
    last_visible=now;
    if((phase==Phase::Front&&front_votes<cfg.votes)||(use_down&&down_votes<cfg.votes))return out;
    if(use_down&&observed.pixel_error<=cfg.center_px) {
      hover_correcting=false;
      if(!stable_since)stable_since=now;
      if(phase==Phase::Down&&now-stable_since>=cfg.center_hold&&observed.sequence!=center_sequence){
        phase=Phase::Hold;reason="hover over tag; depth/yaw hold and live down-camera position correction";}
      center_sequence=observed.sequence;return out;
    }
    stable_since=0;
    if(phase==Phase::Hold) {
      if(!hover_correcting&&observed.pixel_error<=cfg.resume_px)return out;
      hover_correcting=true;
    }
    const double distance=std::hypot(observed.forward,observed.left);
    const double world_dx=std::cos(p.yaw)*observed.forward-std::sin(p.yaw)*observed.left;
    const double world_dy=std::sin(p.yaw)*observed.forward+std::cos(p.yaw)*observed.left;
    goal_x=p.x+world_dx;goal_y=p.y+world_dy;
    if(std::hypot(goal_x-origin_x,goal_y-origin_y)>cfg.radius){fail("observed target outside measured safe corridor");return out;}
    const double limit=phase==Phase::Hold?cfg.hover_speed:use_down?cfg.down_speed:cfg.speed;
    if(distance>1e-6){const double scale=std::min(limit,cfg.gain*distance)/distance;
      out.surge=observed.forward*scale;out.sway=observed.left*scale;}
    const double change=std::hypot(out.surge-previous_surge,out.sway-previous_sway);
    if(change>cfg.target_rate*dt&&change>0){
      const double scale=cfg.target_rate*dt/change;
      out.surge=previous_surge+(out.surge-previous_surge)*scale;
      out.sway=previous_sway+(out.sway-previous_sway)*scale;
    }
    // Crossing into a slower phase must respect its absolute output cap too.
    const double magnitude=std::hypot(out.surge,out.sway);
    if(magnitude>limit){out.surge*=limit/magnitude;out.sway*=limit/magnitude;}
    applied_surge=out.surge;applied_sway=out.sway;
    return out;
  }
  Phase phase{Phase::Idle};std::string reason;
  double report_target{},report_mean{},report_min{},report_max{},goal_x{},goal_y{};
  unsigned report_samples{};
 private:
  bool fresh(const DockObservation& o,double now)const{return o.detected&&o.metric_valid&&o.id==cfg.id&&o.sequence&&
      o.stamp>0&&now>=o.stamp&&now-o.stamp<=cfg.frame_timeout;}
  void count_votes(const DockObservation& o,bool good,std::uint64_t& seq,int& votes)const{
    if(!good){votes=0;return;}if(o.sequence!=seq){seq=o.sequence;votes=std::min(votes+1,cfg.votes);}}
  void clear_stability(){stable_since=0;sample_sum=0;sample_count=0;sample_min=std::numeric_limits<double>::infinity();sample_max=-sample_min;}
  TagDockConfig cfg;double started{},entered{},last_tick{},target_depth{},stable_since{},last_visible{};
  double origin_x{},origin_y{},sample_sum{},sample_min{std::numeric_limits<double>::infinity()},sample_max{-std::numeric_limits<double>::infinity()};
  unsigned sample_count{};std::size_t depth_index{};bool hover_correcting{};
  double applied_surge{},applied_sway{};
  std::uint64_t session{},last_depth_seq{},front_sequence{},down_sequence{},center_sequence{};
  int front_votes{},down_votes{};
};
