#define main front_surface_replay_main
#include "../src/surface_replay.cpp"
#undef main
#include "../src/mission_recorder.hpp"
#include <opencv2/imgproc.hpp>
#include <chrono>

static void require(bool yes){if(!yes)throw std::runtime_error("front surface replay regression");}
int main(int argc,char** argv) {
  try {
  require(argc==2);
  const auto dir=std::filesystem::temp_directory_path()/
    ("auv-front-surface-replay-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(dir);
  auto config=YAML::LoadFile((std::filesystem::path(argv[1])/"runtime/config/pi-auv-task-one.yaml").string());
  config["vision"]["white_grid_edges"]=false; // This fixture deliberately uses black grid edges.
  auto l=config["localization"];l["pool_depth_m"]=3.0;
  auto t=config["surface_traversal"];t["cell_size_m"]=.5;
  // Route absolute localization through the front camera: metric_camera_front
  // selects front_metric.surface with a front-only 320x240 K/D. The down surface
  // calibration stays unverified, so selecting the wrong camera fails the run.
  auto f=config["front_metric"];f["enabled"]=true;
  f["surface"]["verified"]=true;
  f["surface"]["camera_matrix"]=YAML::Load("[400,0,160,0,400,120,0,0,1]");
  f["surface"]["distortion_coefficients"]=YAML::Load("[0,0,0,0,0]");
  f["camera_to_body"]=YAML::Load("[0,1,0,1,0,0,0,0,-1]");
  f["camera_offset_m"]=YAML::Load("[0,0,0]");f["depth_offset_m"]=YAML::Load("[0,0,0]");
  {std::ofstream out(dir/"config.yaml");out<<config;}
  cv::Mat image(240,320,CV_8UC3,cv::Scalar(210,210,210));
  cv::rectangle(image,{60,20},{260,220},cv::Scalar(0,0,0),5);
  for(int line:{87,153})cv::line(image,{60,line},{260,line},cv::Scalar(0,0,0),5);
  for(int line:{127,193})cv::line(image,{line,20},{line,220},cv::Scalar(0,0,0),5);
  cv::line(image,{60,220},{260,220},cv::Scalar(0,255,255),9);
  std::vector<std::uint8_t> jpeg;require(cv::imencode(".jpg",image,jpeg));
  {MissionRecorder recorder(dir.string(),60,1024);
    for(int i=1;i<=12;++i) {
      const double stamp=10+i*.1;
      const std::string telemetry="{\"phase\":\"VISIT_CONES\",\"valid\":true,\"received_sec\":"+std::to_string(stamp)+",\"depth_m\":0}";
      recorder.append("front",jpeg,i,stamp,"\"synthetic\"",telemetry);
    }
  }
  std::vector<std::string> args{"auv_surface_replay",(dir/"config.yaml").string(),(dir/"front-1.mjpg").string(),
    (dir/"front-1.frames.jsonl").string(),(dir/"poses.jsonl").string()};
  std::vector<char*> pointers;for(auto& a:args)pointers.push_back(a.data());
  require(front_surface_replay_main(5,pointers.data())==0);
  std::ifstream results(dir/"poses.jsonl");std::string line;int valid=0,count=0;
  while(std::getline(results,line)) {
    const auto point=YAML::Load(line);++count;
    if(point["valid"].as<bool>()){
      // Thick yellow/black contour edges bias the outer corners by a fixed pixel
      // amount. At 320x240 the front grid is ~200px, so that raster bias is larger
      // in cells than the 640x480 down fixture; routing and resolution correctness
      // are still enforced by the unverified down surface and the 320 vs 640 size
      // gate. This only sanity-checks that the pose lands near the grid center.
      ++valid;require(std::abs(point["row"].as<double>()-1.5)<.2&&std::abs(point["col"].as<double>()-1.5)<.2);
    }
  }
  results.close();require(count==12&&valid>=8);
  require(front_surface_replay_main(5,pointers.data())==1); // Never overwrite evidence.
  std::filesystem::remove_all(dir);
  return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
