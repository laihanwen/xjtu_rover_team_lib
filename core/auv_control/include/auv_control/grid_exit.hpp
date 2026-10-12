#pragma once
#include "auv_mapping/metric_grid_pose.hpp"
#include <queue>
#include <algorithm>
#include <limits>

namespace auv_control {
struct GridExitConfig {
  bool enabled{false},verified{false};unsigned allowed_sides{0}; // N/E/S/W bits
  double cell_size_m{0},body_radius_m{0},clearance_m{0},speed{.1},gain{.4};
  double pose_timeout{.3},timeout{60},stable_sec{.5},corridor_cells{.15};
  double reacquire_timeout{3};
  double outside_margin()const{return (body_radius_m+clearance_m)/cell_size_m+.15;}
  void validate()const {
    if(!enabled)return;
    for(double v:{cell_size_m,body_radius_m,clearance_m,speed,gain,pose_timeout,timeout,stable_sec,corridor_cells,reacquire_timeout})
      if(!std::isfinite(v)||v<=0)throw std::invalid_argument("exit requires finite measured positive parameters");
    if(!verified||!allowed_sides||allowed_sides>15||outside_margin()>.75||speed>.2||pose_timeout>.5||
       body_radius_m>=cell_size_m*.4||corridor_cells>=.5-body_radius_m/cell_size_m)
      throw std::invalid_argument("exit clearance, sides or navigation unverified");
  }
};
struct GridExitStep {bool fault{false},complete{false};double surge{0},sway{0};std::string reason;};
class GridExit {
 public:
  explicit GridExit(GridExitConfig c):c_(c){c_.validate();}
  bool begin_transit(const auv_mapping::MetricGridPose& pose,const std::array<bool,9>& empty,int goal,double now) {
    reset();if(!c_.enabled||!pose.valid||!pose.sequence||goal<0||goal>=9||!empty[goal]||
      !std::isfinite(now)||!std::isfinite(pose.stamp)||now<pose.stamp||now-pose.stamp>c_.pose_timeout||
      !std::isfinite(pose.row)||!std::isfinite(pose.col)||pose.row<0||pose.row>=3||pose.col<0||pose.col>=3)return false;
    const int start=int(pose.row)*3+int(pose.col);
    std::array<int,9> parent;parent.fill(-2);parent[start]=-1;std::queue<int> todo;todo.push(start);
    while(!todo.empty()&&parent[goal]==-2) {
      const int n=todo.front();todo.pop();
      for(int next:{n-3,n+1,n+3,n-1})if(next>=0&&next<9&&empty[next]&&parent[next]==-2&&
         std::abs(next/3-n/3)+std::abs(next%3-n%3)==1){parent[next]=n;todo.push(next);}
    }
    if(parent[goal]==-2)return false;
    std::vector<int> path;for(int n=goal;n>=0;n=parent[n])path.push_back(n);std::reverse(path.begin(),path.end());
    for(int n:path)points_.emplace_back(n%3+.5,n/3+.5);
    initial_={pose.col,pose.row};transit_=active_=true;last_cell_=start;boundary_=now;return true;
  }
  bool begin(const auv_mapping::MetricGridPose& pose,const std::array<bool,9>& empty,unsigned visited,double now) {
    reset();if(!c_.enabled||!pose.valid||!pose.sequence||!pose.surface_frame||!std::isfinite(pose.stamp)||!std::isfinite(now)||now<pose.stamp||now-pose.stamp>c_.pose_timeout||
      !std::isfinite(pose.row)||!std::isfinite(pose.col)||pose.row<0||pose.row>=3||pose.col<0||pose.col>=3)return false;
    const int start=int(pose.row)*3+int(pose.col);visited_=visited;
    unsigned count=0;for(int i=0;i<9;++i)count+=(visited>>i)&1U;
    if(count!=4||visited>511||!(visited&(1U<<start)))return false;
    std::array<int,9> parent;parent.fill(-2);parent[start]=-1;std::queue<int> todo;todo.push(start);
    int goal=-1,side=-1;
    while(!todo.empty()&&goal<0) {
      const int node=todo.front();todo.pop();const int r=node/3,col=node%3;
      for(int s=0;s<4;++s)if((c_.allowed_sides&(1U<<s))&&
        ((s==0&&r==0)||(s==1&&col==2)||(s==2&&r==2)||(s==3&&col==0))){goal=node;side=s;break;}
      for(int next:{node-3,node+1,node+3,node-1}) {
        if(next<0||next>=9||std::abs(next/3-r)+std::abs(next%3-col)!=1||parent[next]!=-2||
           !empty[next]||(visited&(1U<<next)))continue;
        parent[next]=node;todo.push(next);
      }
    }
    if(goal<0)return false;
    std::vector<int> path;for(int n=goal;n>=0;n=parent[n])path.push_back(n);std::reverse(path.begin(),path.end());
    for(int n:path)points_.emplace_back(n%3+.5,n/3+.5);
    auto outside=points_.back();const double d=(c_.body_radius_m+c_.clearance_m)/c_.cell_size_m+.05;
    if(side==0)outside.y=-d;if(side==1)outside.x=3+d;if(side==2)outside.y=3+d;if(side==3)outside.x=-d;
    points_.push_back(outside);initial_={pose.col,pose.row};side_=side;last_cell_=start;boundary_=now;active_=true;return true;
  }
  GridExitStep step(const auv_mapping::MetricGridPose& p,double now) {
    GridExitStep out;auto fail=[&](const char* s){failed_=true;out.fault=true;out.reason=s;return out;};
    if(failed_)return fail("exit fault latched");
    if(!active_||!p.valid||!p.sequence||(!transit_&&!p.surface_frame)||!std::isfinite(now)||!std::isfinite(p.stamp)||
       !std::isfinite(p.row)||!std::isfinite(p.col)||now<p.stamp||now-p.stamp>c_.pose_timeout||now-boundary_>c_.timeout)
      return fail("fresh bounded exit pose required");
    if(p.stamp<=boundary_){out.reason="waiting for new exit-stage observation";return out;}
    const cv::Matx33d rotation(p.body_from_grid.data());
    for(double v:p.body_from_grid)if(!std::isfinite(v))return fail("exit rotation nonfinite");
    if(cv::norm(cv::Mat(rotation*rotation.t()-cv::Matx33d::eye()))>1e-3||std::abs(cv::determinant(rotation)-1)>1e-3)
      return fail("exit rotation invalid");
    if(sequence_&&p.sequence<sequence_)return fail("exit sequence regressed");
    const bool fresh=p.sequence!=sequence_;
    if(fresh&&sequence_&&(p.stamp<=last_stamp_||std::hypot(p.row-last_row_,p.col-last_col_)*c_.cell_size_m>
         c_.speed*(p.stamp-last_stamp_)+.05))return fail("exit pose jump");
    if(fresh){sequence_=p.sequence;last_stamp_=p.stamp;last_row_=p.row;last_col_=p.col;}
    if(p.row<-c_.outside_margin()||p.row>3+c_.outside_margin()||p.col<-c_.outside_margin()||p.col>3+c_.outside_margin())
      return fail("exit outside supported metric envelope");
    if(p.row>=0&&p.row<3&&p.col>=0&&p.col<3) {
      const int cell=int(p.row)*3+int(p.col);
      if(cell!=last_cell_) {
        if((visited_&(1U<<cell))||std::abs(cell/3-last_cell_/3)+std::abs(cell%3-last_cell_%3)!=1)
          return fail("exit reentered cone or skipped a cell");
        last_cell_=cell;
      }
    }
    const cv::Point2d point(p.col,p.row),target=points_[index_];
    const auto origin=index_?points_[index_-1]:initial_;const auto delta=target-origin;
    const double length=delta.dot(delta),t=length>0?std::clamp((point-origin).dot(delta)/length,0.,1.):0;
    if(cv::norm(point-(origin+delta*t))>c_.corridor_cells)return fail("exit corridor violation");
    if(cv::norm(target-point)<.08&&index_+1<points_.size()){if(fresh)++index_;out.reason="exit waypoint reached";return out;}
    if(transit_&&index_+1==points_.size()&&cv::norm(target-point)<.08) {
      if(!p.landmark_observation){stable_=-1;votes_=0;}
      else if(fresh){if(stable_<0)stable_=now;++votes_;out.complete=now-stable_>=c_.stable_sec&&votes_>=3;}
      out.reason="confirmed measured transit destination";return out;
    }
    const double clear=(c_.body_radius_m+c_.clearance_m)/c_.cell_size_m;
    const bool departed=!transit_&&(side_==0?p.row<=-clear:side_==1?p.col>=3+clear:side_==2?p.row>=3+clear:p.col<=-clear);
    if(index_+1==points_.size()&&departed&&p.landmark_observation&&fresh) {
      if(stable_<0)stable_=now;++votes_;
      if(now-stable_>=c_.stable_sec&&votes_>=3){out.complete=true;out.reason="whole-body exit confirmed by fresh landmarks";return out;}
      out.reason="confirming whole-body exit";return out;
    }
    if(!departed||!p.landmark_observation){stable_=-1;votes_=0;}
    const auto body=rotation*cv::Vec3d((target.x-point.x)*c_.cell_size_m,(target.y-point.y)*c_.cell_size_m,0);
    out.surge=std::clamp(c_.gain*body[0],-c_.speed,c_.speed);out.sway=std::clamp(c_.gain*body[1],-c_.speed,c_.speed);
    out.reason="following verified exit corridor";return out;
  }
  void reset(){active_=failed_=transit_=false;visited_=0;points_.clear();index_=0;sequence_=0;stable_=-1;votes_=0;}
 private:GridExitConfig c_;bool active_{false},failed_{false},transit_{false};unsigned visited_{0};int side_{0},last_cell_{0},votes_{0};
  cv::Point2d initial_;std::vector<cv::Point2d> points_;std::size_t index_{0};std::uint64_t sequence_{0};
  double boundary_{0},last_stamp_{0},last_row_{0},last_col_{0},stable_{-1};
};
}
