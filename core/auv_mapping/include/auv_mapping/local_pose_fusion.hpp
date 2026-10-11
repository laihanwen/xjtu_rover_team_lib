#pragma once
#include "auv_mapping/metric_grid_pose.hpp"
#include "auv_control/observation_search.hpp"

namespace auv_mapping {
// Short bounded propagation only. A stage boundary discards the absolute
// anchor; it cannot resurrect underwater coordinates during surface reacquisition.
class LocalPoseFusion {
 public:
  void reset(std::uint64_t epoch,double boundary) {
    epoch_=epoch;boundary_=boundary;anchored_=false;anchor_={};odom_={};last_={};
  }
  MetricGridPose snapshot(double now)const {auto p=last_;p.valid=p.valid&&std::isfinite(now)&&now>=p.stamp&&now-p.stamp<=.3;return p;}
  MetricGridPose update(const MetricGridPose& absolute,const auv_control::ObservationPose& odom,
      double now,std::uint64_t epoch,double pitch,double maximum_speed,double budget=1.0) {
    MetricGridPose invalid;invalid.reason="local fusion requires new-stage landmark";
    if(epoch!=epoch_||!std::isfinite(now)||!std::isfinite(pitch)||pitch<=0||
       !std::isfinite(maximum_speed)||maximum_speed<=0||!std::isfinite(budget)||budget<=0||budget>2) {
      anchored_=false;return invalid;
    }
    const auto fresh=[&](double stamp){return std::isfinite(stamp)&&stamp>=boundary_&&stamp<=now&&now-stamp<=.3;};
    const bool usable=odom.valid&&odom.continuous&&odom.session&&fresh(odom.stamp)&&
      std::isfinite(odom.x)&&std::isfinite(odom.y)&&std::isfinite(odom.yaw);
    if(absolute.valid&&fresh(absolute.stamp)&&std::isfinite(absolute.row)&&std::isfinite(absolute.col)) {
      if(last_.valid) {
        const double dt=absolute.stamp-last_.stamp;
        if(dt<0||std::hypot(absolute.row-last_.row,absolute.col-last_.col)*pitch>maximum_speed*dt+.05) {
          anchored_=false;last_={};invalid.reason="landmark correction discontinuity";return invalid;
        }
      }
      if(last_.valid&&absolute.stamp==last_.stamp)return last_;
      anchor_=absolute;anchor_.sequence=++sequence_;last_=anchor_;
      odom_=odom;anchored_=usable&&std::abs(odom.stamp-absolute.stamp)<=.1;
      return anchor_;
    }
    if(!anchored_||!usable||odom.session!=odom_.session||odom.stamp<odom_.stamp||
       now-anchor_.stamp>budget||odom.stamp<anchor_.stamp) {
      anchored_=false;invalid.reason="local odometry gap, epoch or drift budget";return invalid;
    }
    const double x=odom.x-odom_.x,y=odom.y-odom_.y;
    const double cs=std::cos(odom_.yaw),sn=std::sin(odom_.yaw);
    const double bx=cs*x+sn*y,by=-sn*x+cs*y;
    const auto& r=anchor_.body_from_grid;
    auto out=anchor_;
    out.col+=(r[0]*bx+r[3]*by)/pitch;out.row+=(r[1]*bx+r[4]*by)/pitch;
    const double dt=odom.stamp-last_.stamp;
    if(!std::isfinite(out.row)||!std::isfinite(out.col)||out.row<0||out.row>=3||out.col<0||out.col>=3||
       dt<0||std::hypot(out.row-last_.row,out.col-last_.col)*pitch>maximum_speed*dt+.01) {
      anchored_=false;invalid.reason="local propagated pose inconsistent";return invalid;
    }
    // Express the fixed grid axes in the CURRENT body frame, not the anchor
    // body's frame, so traversal directions follow the measured yaw.
    const double angle=odom.yaw-odom_.yaw,ca=std::cos(angle),sa=std::sin(angle);
    for(int i=0;i<3;++i){out.body_from_grid[i]=ca*r[i]+sa*r[3+i];out.body_from_grid[3+i]=-sa*r[i]+ca*r[3+i];}
    out.stamp=odom.stamp;out.sequence=odom.stamp>last_.stamp?++sequence_:last_.sequence;out.landmark_observation=false;out.semantic_match=false;
    out.reason="bounded down odometry from unique local landmark";last_=out;return out;
  }
 private:
  std::uint64_t epoch_{0},sequence_{0};double boundary_{0};bool anchored_{false};
  MetricGridPose anchor_,last_;auv_control::ObservationPose odom_;
};
}
