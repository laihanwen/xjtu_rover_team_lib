#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

// A single straight outbound/reverse trial. Surge is a normalized MCU target,
// not metres per second. The time-based fallback is explicitly open loop.
struct BootTagRouteConfig {
  bool explicit_motion_authorization{};
  double stable_sec{10}, stable_span_m{.03}, min_start_depth_m{.35};
  double min_operating_depth_m{.1}, max_depth_m{1.2};
  double ascend_m{.2}, depth_tolerance_m{.04}, ascend_timeout_sec{30};
  double forward_target{.04}, forward_sec{15}, max_displacement_m{.5};
  double return_timeout_sec{25}, arrival_tolerance_m{.12}, hold_sec{2}, hold_timeout_sec{12};
  double yaw_limit_rad{.35}, arm_timeout_sec{5};
  void validate() const {
    for(double v:{stable_sec,stable_span_m,min_start_depth_m,min_operating_depth_m,max_depth_m,ascend_m,
      depth_tolerance_m,ascend_timeout_sec,forward_target,forward_sec,max_displacement_m,
      return_timeout_sec,arrival_tolerance_m,hold_sec,hold_timeout_sec,yaw_limit_rad,arm_timeout_sec})
      if(!std::isfinite(v))throw std::runtime_error("nonfinite boot route limit");
    if(stable_sec<10||stable_span_m<=0||stable_span_m>.05||min_start_depth_m<.3||
       min_operating_depth_m<.1||max_depth_m>1.2||min_start_depth_m>=max_depth_m||ascend_m!=.2||
       min_start_depth_m-ascend_m<min_operating_depth_m+depth_tolerance_m||
       depth_tolerance_m<=0||depth_tolerance_m>.05||
       ascend_timeout_sec<=0||forward_target<=0||forward_target>.08||
       forward_sec<=0||forward_sec>15||max_displacement_m<=0||max_displacement_m>.5||
       return_timeout_sec<=forward_sec||return_timeout_sec>45||
       arrival_tolerance_m<=0||arrival_tolerance_m>=max_displacement_m||
       hold_sec<=0||hold_timeout_sec<=hold_sec||hold_timeout_sec>30||
       yaw_limit_rad<=0||yaw_limit_rad>.35||arm_timeout_sec<=0)
      throw std::runtime_error("invalid boot route limits");
  }
};

class BootTagRoute {
 public:
  enum class Phase {WaitDepth,WaitArm,Ascend,Forward,Return,Hold,Done,Fault};
  struct Input {
    double now{},depth{},yaw{},x{},y{};
    std::uint64_t depth_sequence{};
    bool depth_fresh{},status_safe{},cameras_fresh{},recording_ready{},armed{},pose_valid{};
    bool front_tag{},down_tag{};
  };
  struct Output {
    double surge{},target_depth{},target_yaw{};
    bool start{},arm{},disarm{},transition{};
    std::string event,detail;
  };
  explicit BootTagRoute(BootTagRouteConfig cfg):cfg_(cfg){cfg_.validate();}
  const char* name() const {
    switch(phase){case Phase::WaitDepth:return "DEPTH_STABLE_WAIT";
      case Phase::WaitArm:return "WAIT_ARM";case Phase::Ascend:return "ASCEND_0.2M";
      case Phase::Forward:return "FORWARD";case Phase::Return:return "RETURN";
      case Phase::Hold:return "ARRIVED_HOLD";case Phase::Done:return "DONE";
      default:return "FAULT";}
  }
  bool active()const{return phase!=Phase::WaitDepth&&phase!=Phase::Done&&phase!=Phase::Fault;}
  Phase phase{Phase::WaitDepth};
  std::string fault_reason;
  double stable_depth{},forward_duration{},forward_distance{};
  bool open_loop_return{};
  Output step(const Input& in) {
    Output out;out.target_depth=target_depth_;out.target_yaw=yaw_target_;
    if(phase==Phase::Done||phase==Phase::Fault)return out;
    const bool depth_outside=std::isfinite(in.depth)&&
      (in.depth<0||in.depth>cfg_.max_depth_m||
       (phase!=Phase::WaitDepth&&in.depth<cfg_.min_operating_depth_m));
    if(!std::isfinite(in.now)||!std::isfinite(in.depth)||!std::isfinite(in.yaw)||
       (in.pose_valid&&(!std::isfinite(in.x)||!std::isfinite(in.y)))||
       !in.depth_fresh||!in.status_safe||!in.cameras_fresh||!in.recording_ready||
       depth_outside){
      if(phase!=Phase::WaitDepth)return fail(depth_outside?"depth outside operating envelope":
        "sensor, status, camera, recording or pose invalid");
      reset_stability();return out;
    }
    if(phase==Phase::WaitDepth){
      if(in.depth<cfg_.min_start_depth_m){reset_stability();return out;}
      if(in.depth_sequence!=last_depth_sequence_){
        last_depth_sequence_=in.depth_sequence;
        if(!stable_since_){stable_since_=in.now;depth_min_=depth_max_=in.depth;}
        depth_min_=std::min(depth_min_,in.depth);depth_max_=std::max(depth_max_,in.depth);
        if(depth_max_-depth_min_>cfg_.stable_span_m){stable_since_=in.now;depth_min_=depth_max_=in.depth;stable_samples_=0;}
        ++stable_samples_;
      }
      if(stable_samples_>=5&&stable_since_&&in.now-stable_since_>=cfg_.stable_sec){
        stable_depth=in.depth;target_depth_=stable_depth-cfg_.ascend_m;
        if(target_depth_<cfg_.min_operating_depth_m+cfg_.depth_tolerance_m)
          return fail("ascent target shallower than envelope");
        yaw_target_=in.yaw;start_x_=in.x;start_y_=in.y;pose_at_start_=in.pose_valid;
        phase=Phase::WaitArm;entered_=in.now;
        out.start=out.arm=true;out.transition=true;out.event="DEPTH_STABLE";
        out.detail="independent pressure samples; positive underwater depth; explicit trial authorization";
      }
    }else{
      if(std::abs(std::remainder(in.yaw-yaw_target_,2*3.141592653589793))>cfg_.yaw_limit_rad)
        return fail("yaw deviation limit exceeded");
      if(phase==Phase::WaitArm){
        if(in.armed){phase=Phase::Ascend;entered_=in.now;out.transition=true;out.event="ASCEND_START";}
        else if(in.now-entered_>cfg_.arm_timeout_sec)return fail("ARM acknowledgement timeout");
      }else if(!in.armed)return fail("unexpected DISARM");
      else if(phase==Phase::Ascend){
        if(in.now-entered_>cfg_.ascend_timeout_sec)return fail("ascent timeout");
        if(in.depth<=target_depth_+cfg_.depth_tolerance_m){
          phase=Phase::Forward;entered_=in.now;forward_start_x_=in.x;forward_start_y_=in.y;
          yaw_target_=in.yaw; // Lock the forward heading once; reverse without turning.
          pose_at_forward_=in.pose_valid;out.transition=true;out.event="ASCEND_DONE";
          out.detail="depth target reached; forward started";
        }
      }else if(phase==Phase::Forward){
        const double elapsed=in.now-entered_;
        const double displacement=pose_at_forward_&&in.pose_valid?
          std::hypot(in.x-forward_start_x_,in.y-forward_start_y_):-1;
        if(displacement>=cfg_.max_displacement_m&&displacement>=0)
          return fail("forward displacement limit exceeded");
        if(in.front_tag||in.down_tag||elapsed>=cfg_.forward_sec){
          forward_duration=std::max(0.,elapsed);forward_distance=displacement;
          open_loop_return=displacement<0||!pose_at_start_;
          phase=Phase::Return;entered_=in.now;out.transition=true;out.event="FORWARD_END";
          out.detail=in.front_tag?"front tag":in.down_tag?"down tag":"forward duration reached without tag";
        }else out.surge=cfg_.forward_target;
      }else if(phase==Phase::Return){
        const double elapsed=in.now-entered_;
        if(elapsed>cfg_.return_timeout_sec)return fail("return timeout");
        if(!open_loop_return&&!in.pose_valid){open_loop_return=true;out.transition=true;
          out.event="RETURN_OPEN_LOOP";out.detail="localization lost; reverse duration estimate";}
        const double distance=std::hypot(in.x-start_x_,in.y-start_y_);
        if((!open_loop_return&&distance<=cfg_.arrival_tolerance_m)||
           (open_loop_return&&elapsed>=forward_duration)){
          phase=Phase::Hold;entered_=in.now;hold_since_=0;out.transition=true;
          out.event="RETURN_END";out.detail=open_loop_return?"open-loop reverse duration":"measured origin tolerance";
        }else out.surge=-cfg_.forward_target;
      }else if(phase==Phase::Hold){
        if(in.now-entered_>cfg_.hold_timeout_sec)return fail("final depth hold timeout");
        if(in.depth_sequence!=last_depth_sequence_){
          last_depth_sequence_=in.depth_sequence;
          if(std::abs(in.depth-target_depth_)<=cfg_.depth_tolerance_m){
            if(!hold_since_)hold_since_=in.now;
            ++hold_samples_;
          }else{hold_since_=0;hold_samples_=0;}
        }
        if(hold_samples_>=3&&hold_since_&&in.now-hold_since_>=cfg_.hold_sec){
          phase=Phase::Done;out.disarm=true;out.transition=true;out.event="HOLD_STABLE";
          out.detail="translation neutral; depth stable; DISARM requested";
        }
      }
    }
    out.target_depth=target_depth_;out.target_yaw=yaw_target_;
    return out;
  }
  Output fail(const std::string& why){phase=Phase::Fault;fault_reason=why;
    Output out;out.disarm=true;out.transition=true;out.event="FAULT";out.detail=why;return out;}
 private:
  void reset_stability(){stable_since_=0;stable_samples_=0;depth_min_=depth_max_=0;}
  BootTagRouteConfig cfg_;
  double stable_since_{},depth_min_{},depth_max_{},entered_{},target_depth_{},yaw_target_{};
  double start_x_{},start_y_{},forward_start_x_{},forward_start_y_{},hold_since_{};
  std::uint64_t last_depth_sequence_{};
  int stable_samples_{},hold_samples_{};
  bool pose_at_start_{},pose_at_forward_{};
};
