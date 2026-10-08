#define main surface_replay_main
#include "../src/surface_replay.cpp"
#undef main
#include "../src/mission_recorder.hpp"
#include <opencv2/imgproc.hpp>
#include <chrono>

static void require(bool yes){if(!yes)throw std::runtime_error("surface replay regression");}
int main(int argc,char** argv) {
  try {
  require(argc==2);
  const auto dir=std::filesystem::temp_directory_path()/
    ("auv-surface-replay-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(dir);
  auto config=YAML::LoadFile((std::filesystem::path(argv[1])/"runtime/config/pi-auv-task-one.yaml").string());
  auto l=config["localization"];
  l["camera_to_body"]=YAML::Load("[0,1,0,1,0,0,0,0,-1]");
  l["camera_offset_m"]=YAML::Load("[0,0,0]");l["depth_offset_m"]=YAML::Load("[0,0,0]");l["pool_depth_m"]=1.875;
  auto t=config["surface_traversal"];t["surface_localization_verified"]=true;t["cell_size_m"]=.5;
  t["camera_matrix"]=YAML::Load("[400,0,320,0,400,240,0,0,1]");t["distortion_coefficients"]=YAML::Load("[0,0,0,0,0]");
  {std::ofstream out(dir/"config.yaml");out<<config;}
  cv::Mat image(480,640,CV_8UC3,cv::Scalar(210,210,210));
  cv::rectangle(image,{160,80},{480,400},cv::Scalar(0,0,0),5);
  for(int line:{187,293})cv::line(image,{160,line},{480,line},cv::Scalar(0,0,0),5);
  for(int line:{267,373})cv::line(image,{line,80},{line,400},cv::Scalar(0,0,0),5);
  cv::line(image,{160,400},{480,400},cv::Scalar(0,255,255),9);
  std::vector<std::uint8_t> jpeg;require(cv::imencode(".jpg",image,jpeg));
  {MissionRecorder recorder(dir.string(),60,1024);
    for(int i=1;i<=12;++i) {
      const double stamp=10+i*.1;
      const std::string telemetry="{\"phase\":\"VISIT_CONES\",\"valid\":true,\"received_sec\":"+std::to_string(stamp)+",\"depth_m\":0}";
      recorder.append("down",jpeg,i,stamp,"\"synthetic\"",telemetry);
    }
  }
  std::vector<std::string> args{"auv_surface_replay",(dir/"config.yaml").string(),(dir/"down-1.mjpg").string(),
    (dir/"down-1.frames.jsonl").string(),(dir/"poses.jsonl").string()};
  std::vector<char*> pointers;for(auto& a:args)pointers.push_back(a.data());
  require(surface_replay_main(5,pointers.data())==0);
  std::ifstream results(dir/"poses.jsonl");std::string line;int valid=0,count=0;
  while(std::getline(results,line)) {
    const auto point=YAML::Load(line);++count;
    if(point["valid"].as<bool>()){
      // Thick yellow/black contour edges have a bounded subcell bias in this
      // raster fixture; exact projected corners are tested separately.
      ++valid;require(std::abs(point["row"].as<double>()-1.5)<.06&&std::abs(point["col"].as<double>()-1.5)<.06);
    }
  }
  results.close();require(count==12&&valid>=8);
  require(surface_replay_main(5,pointers.data())==1); // Never overwrite evidence.
  std::filesystem::remove_all(dir);
  return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
