#pragma once
#include "camera_rectifier.hpp"
#include "auv_mapping/plane_odometry.hpp"
#include "auv_control/observation_search.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <deque>
#include <utility>
#include <cstdint>
#include <chrono>
#include <memory>

// Adapter owns image tracking and status. Geometry core has no ROS, UART or HTTP dependencies.
class Localization {
 public:
 struct Settings {
  bool enabled=false;int width=0,height=0;double hz=10;
  std::string input="/run/auv-rov/localization-input.json";
  std::vector<double> intrinsics,distortion,rotation,camera_offset,depth_offset,imu_signs,imu_offsets;
  auv_mapping::PlaneCalibration plane;
 };
 static Settings config(const YAML::Node& n){
  Settings c;if(!n)return c;c.enabled=n["enabled"].as<bool>(false);
  c.width=n["width"].as<int>(0);c.height=n["height"].as<int>(0);c.hz=n["hz"].as<double>(10);
  if(!std::isfinite(c.hz)||c.hz<1||c.hz>20)throw std::runtime_error("localization hz must be 1..20");
  c.input=n["telemetry_file"].as<std::string>(c.input);
  for(auto item:{std::make_pair("camera_matrix",&c.intrinsics),{"distortion",&c.distortion},{"camera_to_body",&c.rotation},{"camera_offset_m",&c.camera_offset},{"depth_offset_m",&c.depth_offset},{"imu_signs",&c.imu_signs},{"imu_offsets_deg",&c.imu_offsets}})
   if(n[item.first]) *item.second=n[item.first].as<std::vector<double>>();
  c.plane.pool_depth=n["pool_depth_m"].as<double>(0);c.plane.depth_zero=n["depth_zero_m"].as<double>(0);c.plane.verified=n["calibration_verified"].as<bool>(false);
  if(c.rotation.size()==9)std::copy(c.rotation.begin(),c.rotation.end(),c.plane.camera_to_body.begin());
  if(c.camera_offset.size()==3)std::copy(c.camera_offset.begin(),c.camera_offset.end(),c.plane.camera_offset.begin());
  if(c.depth_offset.size()==3)std::copy(c.depth_offset.begin(),c.depth_offset.end(),c.plane.depth_offset.begin());
  return c;
 }
 explicit Localization(Settings cfg):cfg_(std::move(cfg)){}
 double period()const{return 1/cfg_.hz;}bool enabled()const{return cfg_.enabled;}
 auv_control::ObservationPose snapshot(double now)const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {valid_ && now>=stamp_ && now-stamp_<=.3,continuous_,session_,stamp_,x_,y_,yaw_};
 }
 bool has_origin()const {std::lock_guard<std::mutex> lock(mutex_);return session_!=0;}
 void unavailable(const std::string& reason){std::lock_guard<std::mutex> lock(mutex_);fail(reason);}
 // Called by a dedicated worker with shared capture image (never opens another camera).
 void process(const cv::Mat& image,double stamp,double now,const YAML::Node& supplied=YAML::Node(),bool image_rectified=false){
  std::lock_guard<std::mutex> lock(mutex_);
  if(!std::isfinite(stamp)||!std::isfinite(now)){stamp_=0;fail("invalid_timestamp");return;}stamp_=stamp;
  if(!cfg_.enabled){fail("disabled");return;}
  if(!ready_config()){fail("calibration_required");return;}
  if(image.cols!=cfg_.width||image.rows!=cfg_.height){fail("calibration_resolution_mismatch");return;}
  if(now-stamp<0||now-stamp>.2){fail("image_stale");return;}
  auv_mapping::PlaneSample sample;sample.stamp=stamp;double input_stamp=0;
  try{
   const auto t=supplied.IsMap()?supplied:YAML::LoadFile(cfg_.input);const double ts=t["stamp"].as<double>();input_stamp=ts;
   if(!std::isfinite(ts)||now-ts<0||now-ts>.3||std::abs(ts-stamp)>.15||!t["valid"].as<bool>()){fail("telemetry_stale_or_unsynchronized");return;}
   armed_=t["armed"].as<bool>(true);
   const double rad=std::acos(-1)/180;
   sample.roll=(cfg_.imu_signs[0]*t["roll_deg"].as<double>()+cfg_.imu_offsets[0])*rad;
   sample.pitch=(cfg_.imu_signs[1]*t["pitch_deg"].as<double>()+cfg_.imu_offsets[1])*rad;
   sample.yaw=(cfg_.imu_signs[2]*t["yaw_deg"].as<double>()+cfg_.imu_offsets[2])*rad;
   sample.depth=t["depth_m"].as<double>();
  }catch(...){fail("telemetry_unavailable");return;}
  auv_mapping::Vec3 check;if(!auv_mapping::floor_point(cfg_.plane,sample,0,0,check)){fail("height_or_attitude_invalid");return;}
  {
    const double deg=180/std::acos(-1);std::ostringstream input;input<<std::setprecision(15);
    input<<"{\"stamp\":"<<input_stamp<<",\"roll_deg\":"<<(sample.roll*deg-cfg_.imu_offsets[0])/cfg_.imu_signs[0]
      <<",\"pitch_deg\":"<<(sample.pitch*deg-cfg_.imu_offsets[1])/cfg_.imu_signs[1]
      <<",\"yaw_deg\":"<<(sample.yaw*deg-cfg_.imu_offsets[2])/cfg_.imu_signs[2]
      <<",\"depth_m\":"<<sample.depth<<",\"armed\":"<<(armed_?"true":"false")<<",\"valid\":true}";
    input_json_=input.str();
  }
  if(std::abs(sample.roll)>.6||std::abs(sample.pitch)>.6){fail("tilt_out_of_range");return;}
  cv::Mat corrected=image;
  if(!rectifier_)rectifier_=std::make_unique<CameraRectifier>(true,cfg_.width,cfg_.height,cfg_.intrinsics,cfg_.distortion);
  if(!image_rectified) {
   corrected=rectifier_->apply(image);
  }
  cv::Mat gray;if(corrected.channels()==1)gray=corrected;else cv::cvtColor(corrected,gray,cv::COLOR_BGR2GRAY);
  std::vector<cv::Point2f> points;cv::goodFeaturesToTrack(gray,points,200,.02,7,rectifier_->valid_mask());
  if(points.size()<30){fail("insufficient_texture");return;}
  ready_=true;ready_time_=now;
  history_.push_back(sample);while(!history_.empty()&&stamp-history_.front().stamp>2.5)history_.pop_front();
  if(gray_.empty()){reason_="ready_set_origin";valid_=false;previous_=sample;gray_=gray.clone();points_=points;return;}
  if(stamp-previous_.stamp<=0||stamp-previous_.stamp>.3){fail("frame_time_gap");return;}
  std::vector<cv::Point2f> next,back;std::vector<unsigned char> status,back_status;std::vector<float> error,back_error;
  cv::calcOpticalFlowPyrLK(gray_,gray,points_,next,status,error,cv::Size(21,21),3);
  cv::calcOpticalFlowPyrLK(gray,gray_,next,back,back_status,back_error,cv::Size(21,21),3);
  std::vector<cv::Point2f> a,b;
  for(std::size_t i=0;i<points_.size();++i)if(status[i]&&back_status[i]&&cv::norm(points_[i]-back[i])<1&&next[i].x>=0&&next[i].y>=0&&next[i].x<gray.cols&&next[i].y<gray.rows&&rectifier_->valid_mask().at<unsigned char>(static_cast<int>(next[i].y),static_cast<int>(next[i].x))){a.push_back(points_[i]);b.push_back(next[i]);}
  if(a.size()<30){fail("tracking_lost");return;}
  cv::Mat mask;auto homography=cv::findHomography(a,b,cv::RANSAC,2,mask);if(homography.empty()){fail("plane_fit_failed");return;}
  cv::Mat k(3,3,CV_64F,cfg_.intrinsics.data());std::vector<cv::Point2f> na,nb;
  // Tracking coordinates are already corrected with the same K. Normalize only.
  cv::undistortPoints(a,na,k,cv::noArray());cv::undistortPoints(b,nb,k,cv::noArray());
  std::vector<std::array<double,4>> matches;
  for(std::size_t i=0;i<a.size();++i)if(mask.at<unsigned char>(static_cast<int>(i)))matches.push_back({na[i].x,na[i].y,nb[i].x,nb[i].y});
  const auto estimate=auv_mapping::estimate_translation(cfg_.plane,previous_,sample,matches);
  if(!estimate.valid){fail(estimate.reason);return;}
  if(std::hypot(estimate.dx,estimate.dy)/(sample.stamp-previous_.stamp)>.02)stationary_since_=0;
  else if(!stationary_since_)stationary_since_=now;
  const double cs=std::cos(origin_yaw_),sn=std::sin(origin_yaw_);
  if(session_&&continuous_){x_+=cs*estimate.dx+sn*estimate.dy;y_+=-sn*estimate.dx+cs*estimate.dy;}
  yaw_=std::atan2(std::sin(sample.yaw-origin_yaw_),std::cos(sample.yaw-origin_yaw_));
  inliers_=estimate.inliers;residual_=estimate.residual;valid_=session_&&continuous_;reason_=valid_?"tracking":session_?"continuity_lost_reset_required":"ready_set_origin";
  previous_=sample;gray_=gray.clone();points_=std::move(points);
 }
 bool reset(double now){
  std::lock_guard<std::mutex> lock(mutex_);
  if(!ready_||armed_||!stationary_since_||now-stationary_since_<2||now-ready_time_>.3||history_.size()<10||history_.back().stamp-history_.front().stamp<2)return false;
  const auto& last=history_.back();for(const auto& s:history_)if(std::abs(s.roll-last.roll)>.01||std::abs(s.pitch-last.pitch)>.01||std::abs(std::remainder(s.yaw-last.yaw,2*std::acos(-1)))>.01||std::abs(s.depth-last.depth)>.03)return false;
  origin_yaw_=last.yaw;previous_=last;x_=y_=yaw_=0;++session_;continuous_=true;valid_=false;reason_="origin_set_wait_next_frame";return true;
 }
 std::string json(double now)const{
  std::lock_guard<std::mutex> lock(mutex_);std::ostringstream s;s<<std::setprecision(15);
  const bool fresh=now>=stamp_&&now-stamp_<=.5;
  s<<"{\"input\":"<<input_json_<<",\"image_space\":\"rectified\",\"frame_id\":\"odom\",\"source\":\"down_plane_imu\",\"session\":"<<session_<<",\"session_id\":\""<<boot_<<"-"<<session_<<"\""
   <<",\"stamp\":"<<stamp_<<",\"age_s\":"<<(stamp_?now-stamp_:-1)
   <<",\"valid\":"<<(valid_&&fresh?"true":"false")<<",\"continuous\":"<<(continuous_?"true":"false")
   <<",\"ready\":"<<(ready_&&now-ready_time_<=.3?"true":"false")<<",\"armed\":"<<(armed_?"true":"false")
   <<",\"x_m\":"<<x_<<",\"y_m\":"<<y_<<",\"yaw_rad\":"<<yaw_<<",\"inliers\":"<<inliers_<<",\"residual_m\":"<<residual_
   <<",\"uncertainty_m\":null,\"reason\":\""<<(stamp_&&!fresh?"image_stale":reason_)<<"\"}";return s.str();
 }
 private:
 bool ready_config()const{
  if(cfg_.width<=0||cfg_.height<=0||cfg_.intrinsics.size()!=9||cfg_.camera_offset.size()!=3||cfg_.depth_offset.size()!=3||cfg_.imu_signs.size()!=3||cfg_.imu_offsets.size()!=3||cfg_.distortion.empty()||!auv_mapping::calibration_valid(cfg_.plane))return false;
  if(cfg_.intrinsics[0]<=0||cfg_.intrinsics[4]<=0)return false;
  for(const auto* v:{&cfg_.intrinsics,&cfg_.distortion,&cfg_.imu_signs,&cfg_.imu_offsets})for(double x:*v)if(!std::isfinite(x))return false;
  for(double x:cfg_.imu_signs)if(std::abs(x)!=1)return false;
  return cfg_.distortion.size()==4||cfg_.distortion.size()==5||cfg_.distortion.size()==8;
 }
 void fail(const std::string& why){valid_=ready_=false;input_json_="null";stationary_since_=0;reason_=why;history_.clear();gray_.release();points_.clear();if(session_)continuous_=false;}
 Settings cfg_;mutable std::mutex mutex_;cv::Mat gray_;std::vector<cv::Point2f> points_;
 std::unique_ptr<CameraRectifier> rectifier_;
 auv_mapping::PlaneSample previous_;std::deque<auv_mapping::PlaneSample> history_;
 bool valid_=false,ready_=false,continuous_=false,armed_=true;double stationary_since_=0,stamp_=0,ready_time_=0,x_=0,y_=0,yaw_=0,origin_yaw_=0,residual_=0;
 std::size_t inliers_=0;const std::uint64_t boot_=static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
 std::uint64_t session_=0;std::string input_json_="null",reason_="disabled";
};
