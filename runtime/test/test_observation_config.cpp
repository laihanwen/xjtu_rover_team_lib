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
  const auto a2base=std::filesystem::path(argv[1])/"runtime/config/pi-auv-task-one.yaml";
  c=load_config(a2base.string());require(c.mission.surface_before_visit&&!c.traversal.enabled&&!c.motion_enabled);
  y["mission"]["profile"]="a2_task_one";require(rejects()); // A1 motion cannot bypass surface commissioning.
  auto l=y["localization"];
  l["camera_matrix"]=YAML::Load("[250,0,320,0,250,240,0,0,1]");
  l["distortion"]=YAML::Load("[0,0,0,0,0]");l["camera_to_body"]=YAML::Load("[0,1,0,1,0,0,0,0,-1]");
  l["camera_offset_m"]=YAML::Load("[0,0,0]");l["depth_offset_m"]=YAML::Load("[0,0,0]");
  l["imu_signs"]=YAML::Load("[1,1,1]");l["imu_offsets_deg"]=YAML::Load("[0,0,0]");
  l["pool_depth_m"]=2;l["calibration_verified"]=true;
  y["camera_front"]["enabled"]=true;
  auto t=YAML::LoadFile(a2base.string())["surface_traversal"];y["surface_traversal"]=t;
  t["enabled"]=true;t["ascent_clearance_verified"]=true;t["surface_localization_verified"]=true;
  t["center_approach_verified"]=true;t["center_approach_radius_cells"]=.3;
  t["ascent_cell"]=YAML::Load("[1,1]");t["cell_size_m"]=.5;t["boundary_clearance_m"]=.1;
  t["camera_matrix"]=YAML::Load("[260,0,320,0,260,240,0,0,1]");t["distortion_coefficients"]=YAML::Load("[0,0,0,0,0]");
  write();c=load_config(temp.string());require(c.traversal.enabled&&c.planner.forbid_target_reentry);
  t["surface_localization_verified"]=false;require(rejects());t["surface_localization_verified"]=true;
  t["boundary_clearance_m"]=0;require(rejects());t["boundary_clearance_m"]=.1;
  t["camera_matrix"]=YAML::Load("[260,0,320,0,260,240,0,0,.nan]");require(rejects());
  std::filesystem::remove(temp);
}
