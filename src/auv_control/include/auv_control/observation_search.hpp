#pragma once
#include <array>
#include <algorithm>
#include <utility>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace auv_control {
// Relative to the pre-ARM origin, X forward/Y left. Only measured safe corridors
// belong in this list; no default trajectory is supplied for unknown hardware.
struct ObservationSearchConfig {
  bool enabled{false}, calibrated{false};
  double maximum_radius_m{0}, maximum_speed{0.1}, gain{0.4};
  double tolerance_m{0.10}, stable_sec{0.5}, pose_timeout_sec{0.3};
  double waypoint_timeout_sec{20}, depth_m{0}, yaw_rad{0};
  std::vector<std::array<double,2>> waypoints;
  void validate() const {
    for (double x : {maximum_radius_m,maximum_speed,gain,tolerance_m,stable_sec,
                    pose_timeout_sec,waypoint_timeout_sec,depth_m,yaw_rad})
      if (!std::isfinite(x)) throw std::invalid_argument("nonfinite observation search parameter");
    if (maximum_speed<=0 || maximum_speed>0.2 || gain<=0 || tolerance_m<=0 ||
        stable_sec<=0 || pose_timeout_sec<=0 || waypoint_timeout_sec<=stable_sec ||
        depth_m<0 || std::abs(yaw_rad)>std::acos(-1))
      throw std::invalid_argument("invalid observation search limits");
    if (enabled && (!calibrated || maximum_radius_m<=0 || waypoints.empty() || depth_m<=0))
      throw std::invalid_argument("observation search requires measured corridor and depth");
    for (const auto& p:waypoints)
      if (!std::isfinite(p[0]) || !std::isfinite(p[1]) ||
          std::hypot(p[0],p[1])>maximum_radius_m)
        throw std::invalid_argument("search waypoint outside configured envelope");
  }
};
struct ObservationPose {
  bool valid{false}, continuous{false};
  std::uint64_t session{0};
  double stamp{0}, x{0}, y{0}, yaw{0};
};
struct ObservationStep {
  double surge{0}, sway{0};
  bool fault{false}, exhausted{false};
  std::size_t waypoint{0};
  std::string reason;
};
class ObservationSearch {
 public:
  explicit ObservationSearch(ObservationSearchConfig cfg):cfg_(std::move(cfg)){cfg_.validate();}
  void reset(){session_=0;index_=0;entered_=stable_since_=0;hold_=false;}
  ObservationStep step(const ObservationPose& pose,double now,bool hold) {
    ObservationStep out;out.waypoint=index_;
    if(!cfg_.enabled) {out.reason="disabled";return out;}
    if(!std::isfinite(now)||!pose.valid||!pose.continuous||!pose.session||
       !std::isfinite(pose.stamp)||!std::isfinite(pose.x)||!std::isfinite(pose.y)||
       !std::isfinite(pose.yaw)||now<pose.stamp||now-pose.stamp>cfg_.pose_timeout_sec){
      out.fault=true;out.reason="search localization invalid or stale";return out;}
    if(session_ && session_!=pose.session){out.fault=true;out.reason="search origin session changed";return out;}
    session_=pose.session;
    if(std::hypot(pose.x,pose.y)>cfg_.maximum_radius_m){out.fault=true;out.reason="search envelope exceeded";return out;}
    if(!entered_) entered_=now;
    if(hold && !hold_){anchor_={pose.x,pose.y};hold_=true;}
    if(!hold_ && index_>=cfg_.waypoints.size()){out.exhausted=true;out.reason="search exhausted without tag";return out;}
    if(!hold_ && now-entered_>cfg_.waypoint_timeout_sec){out.fault=true;out.reason="search waypoint timeout";return out;}
    const auto target=hold_?anchor_:cfg_.waypoints[index_];
    const double dx=target[0]-pose.x,dy=target[1]-pose.y;
    const double distance=std::hypot(dx,dy);
    if(distance<=cfg_.tolerance_m){
      if(!stable_since_)stable_since_=now;
      if(!hold_ && now-stable_since_>=cfg_.stable_sec){++index_;entered_=now;stable_since_=0;}
      out.reason=hold_?"holding map observation position":"waypoint settling";return out;
    }
    stable_since_=0;
    const double speed=std::min(cfg_.maximum_speed,cfg_.gain*distance);
    const double vx=dx/distance*speed,vy=dy/distance*speed;
    out.surge=std::cos(pose.yaw)*vx+std::sin(pose.yaw)*vy;
    out.sway=-std::sin(pose.yaw)*vx+std::cos(pose.yaw)*vy;
    out.reason=hold_?"holding map observation position":"searching";return out;
  }
 private:
  ObservationSearchConfig cfg_;std::uint64_t session_{0};std::size_t index_{0};
  double entered_{0},stable_since_{0};bool hold_{false};std::array<double,2> anchor_{};
};
}
