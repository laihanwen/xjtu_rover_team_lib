#include "../src/localization.hpp"
#include <stdexcept>
static void require(bool x){if(!x)throw std::runtime_error("localization origin regression");}
int main(){
  const auto config=YAML::Load(R"(
enabled: true
width: 320
height: 240
camera_matrix: [250, 0, 160, 0, 250, 120, 0, 0, 1]
distortion: [0, 0, 0, 0, 0]
camera_to_body: [0, -1, 0, -1, 0, 0, 0, 0, -1]
camera_offset_m: [0, 0, 0]
depth_offset_m: [0, 0, 0]
imu_signs: [1, 1, 1]
imu_offsets_deg: [0, 0, 0]
pool_depth_m: 2
depth_zero_m: 0
calibration_verified: true
)");
  Localization l(Localization::config(config));
  YAML::Node input;input["valid"]=true;input["armed"]=false;
  input["roll_deg"]=0;input["pitch_deg"]=0;input["yaw_deg"]=0;input["depth_m"]=.3;
  cv::Mat image(240,320,CV_8UC3);cv::RNG rng(7);rng.fill(image,cv::RNG::UNIFORM,0,255);
  require(!l.reset(10));
  for(int i=0;i<28;++i){const double now=10+i*.1;input["stamp"]=now;l.process(image,now,now,input);}
  require(l.reset(12.7));require(l.has_origin());
  input["stamp"]=12.8;l.process(image,12.8,12.8,input);
  const auto pose=l.snapshot(12.8);require(pose.valid && pose.continuous && pose.session==1);
  require(!l.snapshot(13.2).valid);
  input["armed"]=true;input["stamp"]=12.9;l.process(image,12.9,12.9,input);require(!l.reset(12.9));
  l.unavailable("tracking_lost");require(!l.snapshot(12.9).valid && !l.snapshot(12.9).continuous);
  require(l.has_origin()); // Automatic startup must not silently reset a lost session.
  // Raw replay and shared rectified capture must produce identical odometry.
  auto curved=YAML::Clone(config);curved["distortion"]=YAML::Load("[0.15,-0.04,0.001,0.001,0.01]");
  const auto settings=Localization::config(curved);
  Localization raw_flow(settings),corrected_flow(settings);
  CameraRectifier correction(true,320,240,settings.intrinsics,settings.distortion);
  input["armed"]=false;
  const auto rectified=correction.apply(image);
  for(int i=0;i<28;++i){const double now=20+i*.1;input["stamp"]=now;
    raw_flow.process(image,now,now,input);corrected_flow.process(rectified,now,now,input,true);}
  require(raw_flow.reset(22.7)&&corrected_flow.reset(22.7));
  cv::Mat moved;const cv::Mat shift=(cv::Mat_<double>(2,3)<<1,0,.5,0,1,0);
  cv::warpAffine(image,moved,shift,image.size());input["stamp"]=22.8;
  raw_flow.process(moved,22.8,22.8,input);
  corrected_flow.process(correction.apply(moved),22.8,22.8,input,true);
  const auto a=raw_flow.snapshot(22.8),b=corrected_flow.snapshot(22.8);
  require(a.valid&&b.valid&&std::hypot(a.x,a.y)>0.001);
  require(std::abs(a.x-b.x)<1e-9 && std::abs(a.y-b.y)<1e-9);
  auto disabled=config;disabled["calibration_verified"]=false;
  Localization uncalibrated(Localization::config(disabled));uncalibrated.process(image,13,13,input);
  require(!uncalibrated.reset(13) && !uncalibrated.snapshot(13).valid);
}
