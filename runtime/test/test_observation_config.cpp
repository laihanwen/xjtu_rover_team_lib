#include "../src/runtime_config.hpp"
#include <filesystem>
#include <fstream>
#include <chrono>
static void require(bool x){if(!x)throw std::runtime_error("A0 configuration regression");}
int main(int argc,char** argv){
  require(argc==2);
  const auto base=std::filesystem::path(argv[1])/"runtime/config/pi-auv-observation.yaml";
  auto c=load_config(base.string());require(!c.motion_enabled && !c.auto_arm && c.recording_required &&
    c.grid.single_yellow_edge && c.mission.stop_after_map && c.auto_origin && !c.web_enabled);
  const auto temp=std::filesystem::temp_directory_path()/
    ("auv-config-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".yaml");
  auto y=YAML::LoadFile(base.string());
  auto write=[&](){std::ofstream out(temp);out<<y;};
  auto rejects=[&](){write();try{load_config(temp.string());return false;}catch(const std::exception&){return true;}};
  y["observation_search"]["enabled"]=true;require(rejects());
  y=YAML::LoadFile(base.string());y["operation"]["auto_arm"]=true;require(rejects());
  y=YAML::LoadFile(base.string());y["recording"]["enabled"]=false;require(rejects());
  y=YAML::LoadFile(base.string());y["safety"]["frame_timeout_sec"]=".nan";require(rejects());
  y=YAML::LoadFile(base.string());
  y["observation_search"]["enabled"]=true;y["observation_search"]["corridor_calibrated"]=true;
  y["observation_search"]["maximum_radius_m"]=2;y["observation_search"]["depth_target_m"]=.3;
  y["observation_search"]["waypoints_m"]=YAML::Load("[[1,0]]");
  y["motion"]["motion_commands_enabled"]=true;y["motion"]["directions_calibrated"]=true;y["motion"]["limits_calibrated"]=true;
  y["serial"]["device"]="/dev/DO_NOT_OPEN_SYNTHETIC_TEST";
  y["camera"]["camera_matrix"]=YAML::Load("[250,0,320,0,250,240,0,0,1]");
  y["camera"]["distortion_coefficients"]=YAML::Load("[0,0,0,0,0]");write();
  c=load_config(temp.string());require(c.search.enabled); // No unrelated grid-route gains required in A1.
  std::filesystem::remove(temp);
}
