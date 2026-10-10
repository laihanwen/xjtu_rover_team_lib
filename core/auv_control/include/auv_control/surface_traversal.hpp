#pragma once
#include "auv_mapping/metric_grid_pose.hpp"
#include "auv_planning/grid_planner.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

namespace auv_control {
struct SurfaceTraversalConfig {
  bool enabled{false},ascent_verified{false},surface_verified{false};
  bool center_approach_verified{false};
  double center_approach_radius_cells{0},center_stable_sec{0.3},center_timeout_sec{10};
  int ascent_row{-1},ascent_col{-1};
  double ascent_tolerance_cells{0.15},surface_depth_m{0},depth_tolerance_m{0.05};
  double ascent_rate_mps{0.03},surface_stable_sec{1},reacquire_timeout_sec{15};
  int reacquire_frames{5};
  int surface_depth_samples{3};double depth_sample_timeout_sec{2.5};
  double pose_timeout_sec{0.3},maximum_speed{0.1},gain{0.4},arrival_tolerance_cells{0.12};
  double arrival_stable_sec{0.3},entry_margin_cells{0.05},boundary_clearance_m{0};
  double corridor_tolerance_cells{0.2},pose_jump_tolerance_m{0.05},waypoint_timeout_sec{20};
  auv_mapping::MetricGridCalibration underwater,surface;
  // Front camera can carry grid mapping + metric localization instead of the
  // down camera. camera_to_body is the fixed tilt angle rotation, left as a
  // configurable parameter until measured; verified gates keep it inert.
  bool metric_camera_front{false};
  auv_mapping::MetricGridCalibration front_underwater,front_surface;
  const auv_mapping::MetricGridCalibration& water_model() const { return metric_camera_front?front_underwater:underwater; }
  const auv_mapping::MetricGridCalibration& surface_model() const { return metric_camera_front?front_surface:surface; }
  void validate() const {
    for(double x:{ascent_tolerance_cells,surface_depth_m,depth_tolerance_m,ascent_rate_mps,surface_stable_sec,
      reacquire_timeout_sec,pose_timeout_sec,maximum_speed,gain,arrival_tolerance_cells,arrival_stable_sec,
      entry_margin_cells,boundary_clearance_m,corridor_tolerance_cells,pose_jump_tolerance_m,waypoint_timeout_sec,depth_sample_timeout_sec,
      center_approach_radius_cells,center_stable_sec,center_timeout_sec})
      if(!std::isfinite(x))throw std::invalid_argument("nonfinite A2 setting");
    if(!enabled)return;
    const auto& water=water_model();const auto& air=surface_model();
    water.validate();air.validate();
    if(!ascent_verified||!surface_verified||!center_approach_verified||!water.verified||!air.verified||
      center_approach_radius_cells<=0||center_approach_radius_cells>0.4||
      center_approach_radius_cells>0.5-boundary_clearance_m/air.cell_size_m+1e-6||
      center_stable_sec<=0||center_timeout_sec<=0||ascent_row<0||ascent_row>2||
      ascent_col<0||ascent_col>2||ascent_tolerance_cells<=0||ascent_tolerance_cells>=0.4||
      surface_depth_m<0||surface_depth_m>=air.pool_depth_m||depth_tolerance_m<=0||
      ascent_rate_mps<=0||ascent_rate_mps>0.1||surface_stable_sec<=0||reacquire_timeout_sec<=0||reacquire_frames<2||
      surface_depth_samples<2||depth_sample_timeout_sec<=0||depth_sample_timeout_sec>3||
      pose_timeout_sec<=0||maximum_speed<=0||maximum_speed>0.2||gain<=0||arrival_tolerance_cells<=0||
      arrival_tolerance_cells>=0.3||arrival_stable_sec<=0||entry_margin_cells<=0||entry_margin_cells>=0.2||
      boundary_clearance_m<=0||boundary_clearance_m>=air.cell_size_m*0.4||
      corridor_tolerance_cells<=0||corridor_tolerance_cells>=0.4||
      pose_jump_tolerance_m<=0||pose_jump_tolerance_m>=air.cell_size_m*0.25||waypoint_timeout_sec<=0||
      std::abs(water.cell_size_m-air.cell_size_m)>1e-6||
      std::abs(water.pool_depth_m-air.pool_depth_m)>1e-6)
      throw std::invalid_argument("A2 requires measured ascent clearance, independent surface calibration and safe limits");
  }
};

struct CenterApproachStep {bool fault{false},complete{false};double surge{0},sway{0};std::string detail;};
class CenterApproach {
 public:
  explicit CenterApproach(SurfaceTraversalConfig c):c_(std::move(c)){c_.validate();}
  CenterApproachStep step(const auv_mapping::MetricGridPose& p,double now) {
    CenterApproachStep out;
    const double dr=c_.ascent_row+0.5-p.row,dc=c_.ascent_col+0.5-p.col;
    if(!c_.enabled||!p.valid||!p.sequence||p.surface_frame||!std::isfinite(now)||!std::isfinite(p.stamp)||
      now<p.stamp||now-p.stamp>c_.pose_timeout_sec||!std::isfinite(dr)||!std::isfinite(dc)||
      std::hypot(dr,dc)>c_.center_approach_radius_cells) {
      out.fault=true;out.detail="outside measured underwater center approach region";return out;
    }
    if(start_<0)start_=now;
    if(now-start_>c_.center_timeout_sec){out.fault=true;out.detail="center alignment timeout";return out;}
    if(p.sequence!=sequence_) {
      if(sequence_&&(p.sequence<sequence_||p.stamp<=last_stamp_)) {
        out.fault=true;out.detail="center observation sequence discontinuity";return out;
      }
      sequence_=p.sequence;last_stamp_=p.stamp;
      if(std::abs(dr)<=c_.ascent_tolerance_cells&&std::abs(dc)<=c_.ascent_tolerance_cells) {
        if(stable_<0)stable_=p.stamp;
        out.complete=p.stamp-stable_>=c_.center_stable_sec;
      }else stable_=-1;
    }
    const double x=dc*c_.water_model().cell_size_m*c_.gain,y=dr*c_.water_model().cell_size_m*c_.gain;
    out.surge=p.body_from_grid[0]*x+p.body_from_grid[1]*y;
    out.sway=p.body_from_grid[3]*x+p.body_from_grid[4]*y;
    const double speed=std::hypot(out.surge,out.sway);
    if(!std::isfinite(speed)){out.fault=true;out.detail="invalid center control rotation";return out;}
    if(speed>c_.maximum_speed){out.surge*=c_.maximum_speed/speed;out.sway*=c_.maximum_speed/speed;}
    if(out.complete)out.surge=out.sway=0;
    out.detail="aligning within verified central cell before ascent";return out;
  }
  void reset(){start_=stable_=-1;sequence_=0;}
 private:
  SurfaceTraversalConfig c_;double start_{-1},stable_{-1},last_stamp_{0};std::uint64_t sequence_{0};
};

struct AscentStep {bool fault{false},complete{false};double target_depth{0};std::string detail;};
class ControlledAscent {
 public:
  explicit ControlledAscent(SurfaceTraversalConfig c):c_(std::move(c)){c_.validate();}
  bool begin(const auv_mapping::MetricGridPose& pose,double depth,double now) {
    if(!c_.enabled||!pose.valid||pose.surface_frame||!std::isfinite(now)||!std::isfinite(pose.stamp)||
      !std::isfinite(pose.row)||!std::isfinite(pose.col)||now<pose.stamp||now-pose.stamp>c_.pose_timeout_sec||
      !std::isfinite(depth)||depth<c_.surface_depth_m||
      std::abs(pose.row-c_.ascent_row-0.5)>c_.ascent_tolerance_cells||
      std::abs(pose.col-c_.ascent_col-0.5)>c_.ascent_tolerance_cells)return false;
    start_depth_=target_=depth;last_=now;stable_=-1;samples_=0;sample_sequence_=0;active_=true;return true;
  }
  AscentStep step(double depth,bool safe,double now,std::uint32_t sensor_sequence) {
    AscentStep out;out.target_depth=target_;
    if(!active_||!safe||!sensor_sequence||!std::isfinite(depth)||!std::isfinite(now)||now<last_||
      depth>start_depth_+c_.depth_tolerance_m||depth<0) {
      out.fault=true;out.detail="ascent unavailable or depth outside envelope";return out;
    }
    // Rate-limited absolute setpoint. Never command lateral thrust during ascent.
    target_=std::max(c_.surface_depth_m,target_-c_.ascent_rate_mps*std::min(now-last_,0.1));last_=now;
    out.target_depth=target_;out.detail="controlled ascent in verified clear column";
    if(target_<=c_.surface_depth_m+1e-6 && std::abs(depth-c_.surface_depth_m)<=c_.depth_tolerance_m) {
      if(stable_<0)stable_=now;
      if(sensor_sequence!=sample_sequence_){++samples_;sample_sequence_=sensor_sequence;}
      out.complete=now-stable_>=c_.surface_stable_sec && samples_>=c_.surface_depth_samples;
    }else {stable_=-1;samples_=0;sample_sequence_=sensor_sequence;}
    return out;
  }
  void reset(){active_=false;stable_=-1;}
 private:
  SurfaceTraversalConfig c_;bool active_{false};double start_depth_{0},target_{0},last_{0},stable_{-1};
  int samples_{0};std::uint32_t sample_sequence_{0};
};

struct SurfaceRouteStep {
  bool fault{false},complete{false},new_sample{false};std::string detail;
  double surge{0},sway{0};std::size_t waypoint{0};
  std::optional<auv_planning::GridCell> entered,visited;
};
class SurfaceRouteExecutor {
 public:
  explicit SurfaceRouteExecutor(SurfaceTraversalConfig c):c_(std::move(c)){c_.validate();}
  bool set_route(const auv_planning::PlanResult& route) {
    reset();route_=route;
    if(!route.valid||route.path.empty()||route.targets.size()!=4)return false;
    unsigned expected=0;
    for(const auto& t:route.targets) {
      if(t.row<0||t.row>2||t.col<0||t.col>2||(expected&(1U<<(t.row*3+t.col))))return false;
      expected|=1U<<(t.row*3+t.col);
    }
    unsigned seen=0;
    for(std::size_t i=0;i<route.path.size();++i) {
      const auto& cell=route.path[i];if(cell.row<0||cell.row>2||cell.col<0||cell.col>2)return false;
      if(i&&std::abs(cell.row-route.path[i-1].row)+std::abs(cell.col-route.path[i-1].col)!=1)return false;
      if(is_target(cell.row*3+cell.col)) {
        const unsigned bit=1U<<(cell.row*3+cell.col);if(seen&bit)return false;seen|=bit;
      }
    }
    if(seen!=expected)return false;ready_=true;return true;
  }
  SurfaceRouteStep step(const auv_mapping::MetricGridPose& pose,double now) {
    SurfaceRouteStep out;out.waypoint=index_;
    auto fail=[&](const std::string& s){out.fault=true;out.detail=s;return out;};
    if(failed_)return fail("surface route fault latched");
    if(!ready_||!pose.valid||!std::isfinite(now)||now<pose.stamp||now-pose.stamp>c_.pose_timeout_sec||
      !std::isfinite(pose.row)||!std::isfinite(pose.col)) {failed_=true;return fail("fresh absolute surface grid pose required");}
    for(double v:pose.body_from_grid)if(!std::isfinite(v)){failed_=true;return fail("nonfinite grid rotation");}
    const cv::Matx33d rotation(pose.body_from_grid.data());
    if(cv::norm(cv::Mat(rotation*rotation.t()-cv::Matx33d::eye()))>1e-3||std::abs(cv::determinant(rotation)-1)>1e-3) {
      failed_=true;return fail("invalid grid rotation");
    }
    const double margin=c_.boundary_clearance_m/c_.surface_model().cell_size_m;
    if(pose.row<margin||pose.row>3-margin||pose.col<margin||pose.col>3-margin) {
      failed_=true;return fail("measured body exceeds field clearance");
    }
    if(sequence_ && pose.sequence<sequence_){failed_=true;return fail("surface frame sequence regressed");}
    out.new_sample=pose.sequence!=sequence_;
    if(out.new_sample) {
      if(sequence_) {
        const cv::Matx33d previous(last_rotation_.data());
        const auto delta=rotation*previous.t();
        const double angle=std::acos(std::clamp((delta(0,0)+delta(1,1)+delta(2,2)-1)/2,-1.0,1.0));
        // Constant-yaw traversal: reject large reference flips without needing
        // an underwater odometry frame to define the surface map.
        if(angle>0.1+0.5*(pose.stamp-last_stamp_)) {
          failed_=true;return fail("surface orientation discontinuity");
        }
      }
      if(sequence_ && (pose.stamp<=last_stamp_ ||
        std::hypot(pose.row-last_row_,pose.col-last_col_)*c_.surface_model().cell_size_m>
        c_.maximum_speed*(pose.stamp-last_stamp_)+c_.pose_jump_tolerance_m)) {
        failed_=true;return fail("surface pose discontinuity or implausible speed");
      }
      sequence_=pose.sequence;last_stamp_=pose.stamp;last_row_=pose.row;last_col_=pose.col;
      last_rotation_=pose.body_from_grid;
      const int r=static_cast<int>(pose.row),col=static_cast<int>(pose.col),id=r*3+col;
      const double m=c_.entry_margin_cells;
      const bool interior=pose.row-r>=m&&pose.row-r<=1-m&&pose.col-col>=m&&pose.col-col<=1-m;
      if(interior && id!=cell_) {
        if(cell_>=0 && std::abs(r-cell_/3)+std::abs(col-cell_%3)!=1) {
          failed_=true;return fail("unobserved intermediate cell or diagonal crossing");
        }
        if(cell_>=0 && is_target(cell_) && !(confirmed_&(1U<<cell_))) {
          failed_=true;return fail("left cone cell before observation confirmation");
        }
        cell_=id;entry_stamp_=pose.stamp;out.entered=auv_planning::GridCell{static_cast<std::int8_t>(r),static_cast<std::int8_t>(col),"observed"};
        if(is_target(id)) {
          if(entered_&(1U<<id)){failed_=true;return fail("actual cone reentry detected");}
          entered_|=1U<<id;
        }
      }
      if(interior && is_target(id) && !(confirmed_&(1U<<id)) && pose.stamp-entry_stamp_>=c_.arrival_stable_sec) {
        confirmed_|=1U<<id;out.visited=out.entered.value_or(auv_planning::GridCell{
          static_cast<std::int8_t>(r),static_cast<std::int8_t>(col),"observed"});
      }
    }
    if(index_>=route_.path.size()) {
      out.complete=std::all_of(route_.targets.begin(),route_.targets.end(),[&](const auto& t){return confirmed_&(1U<<(t.row*3+t.col));});
      if(!out.complete){failed_=true;return fail("route ended without four observed cone entries");}
      out.detail="four cones confirmed from measured surface trajectory";return out;
    }
    if(waypoint_start_<0)waypoint_start_=now;
    if(now-waypoint_start_>c_.waypoint_timeout_sec){failed_=true;return fail("surface waypoint timeout");}
    const auto& goal=route_.path[index_];const double dr=goal.row+0.5-pose.row,dc=goal.col+0.5-pose.col;
    // Stay in the corridor between adjacent cell centers; first center allows
    // the measured initial point anywhere inside that same cell's clearance.
    if(index_>0) {
      const auto& prev=route_.path[index_-1];
      const double rlo=std::min(prev.row,goal.row)+0.5,rhi=std::max(prev.row,goal.row)+0.5;
      const double clo=std::min(prev.col,goal.col)+0.5,chi=std::max(prev.col,goal.col)+0.5;
      if(std::hypot(pose.row-std::clamp(pose.row,rlo,rhi),pose.col-std::clamp(pose.col,clo,chi))>c_.corridor_tolerance_cells) {
        failed_=true;return fail("actual pose left planned cell corridor");
      }
    }else if(static_cast<int>(pose.row)!=goal.row||static_cast<int>(pose.col)!=goal.col) {
      failed_=true;return fail("surface route start mismatch");
    }
    if(out.new_sample) {
      if(std::abs(dr)<=c_.arrival_tolerance_cells&&std::abs(dc)<=c_.arrival_tolerance_cells) {
        if(arrival_<0)arrival_=pose.stamp;
        if(pose.stamp-arrival_>=c_.arrival_stable_sec) {
          ++index_;arrival_=-1;waypoint_start_=now;out.waypoint=index_;out.detail="measured waypoint reached";return out;
        }
      }else arrival_=-1;
    }
    // Remain motionless inside the arrival band while collecting independent
    // pose evidence. Continuing tiny corrections here can drive the vehicle
    // out of the band before its observation dwell completes.
    if(std::abs(dr)<=c_.arrival_tolerance_cells&&std::abs(dc)<=c_.arrival_tolerance_cells) {
      out.detail="holding waypoint arrival band for fresh observation confirmation";
      return out;
    }
    const double ex=dc*c_.surface_model().cell_size_m*c_.gain,ey=dr*c_.surface_model().cell_size_m*c_.gain;
    out.surge=pose.body_from_grid[0]*ex+pose.body_from_grid[1]*ey;
    out.sway=pose.body_from_grid[3]*ex+pose.body_from_grid[4]*ey;
    const double norm=std::hypot(out.surge,out.sway);
    if(!std::isfinite(norm)||norm>10){failed_=true;return fail("invalid metric control transform");}
    if(norm>c_.maximum_speed){out.surge*=c_.maximum_speed/norm;out.sway*=c_.maximum_speed/norm;}
    out.detail="tracking absolute surface grid waypoint";return out;
  }
  void reset(){ready_=failed_=false;index_=0;sequence_=0;cell_=-1;entered_=confirmed_=0;arrival_=waypoint_start_=-1;}
 private:
  bool is_target(int id)const{return std::any_of(route_.targets.begin(),route_.targets.end(),[&](const auto& t){return t.row*3+t.col==id;});}
  SurfaceTraversalConfig c_;auv_planning::PlanResult route_;bool ready_{false},failed_{false};
  std::size_t index_{0};std::uint64_t sequence_{0};int cell_{-1};unsigned entered_{0},confirmed_{0};
  double entry_stamp_{0},last_stamp_{0},last_row_{0},last_col_{0},arrival_{-1},waypoint_start_{-1};
  std::array<double,9> last_rotation_{};
};
}
