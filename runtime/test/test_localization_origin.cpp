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
  auto disabled=config;disabled["calibration_verified"]=false;
  Localization uncalibrated(Localization::config(disabled));uncalibrated.process(image,13,13,input);
  require(!uncalibrated.reset(13) && !uncalibrated.snapshot(13).valid);
}
