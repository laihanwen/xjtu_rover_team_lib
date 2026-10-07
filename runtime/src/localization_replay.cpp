#include "localization.hpp"
#include <opencv2/videoio.hpp>
#include <fstream>
#include <iostream>
// One JSONL sensor record per video frame: stamp, roll_deg, pitch_deg, yaw_deg,
// depth_m, armed, valid; frame_stamp is separately the image capture timestamp.
int main(int argc,char** argv){
 if(argc!=5){std::cerr<<"Usage: auv_localization_replay config.yaml down.avi input.jsonl output.jsonl\n";return 2;}
 try{
  auto config=Localization::config(YAML::LoadFile(argv[1])["localization"]);config.enabled=true;
  Localization local(config);cv::VideoCapture video(argv[2]);std::ifstream input(argv[3]);std::ofstream output(argv[4]);
  if(!video.isOpened()||!input||!output)throw std::runtime_error("cannot open replay input/output");
  std::string line;cv::Mat image;bool initialized=false;std::size_t count=0;
  while(video.read(image)){
   if(!std::getline(input,line))throw std::runtime_error("missing per-frame telemetry");
   const auto record=YAML::Load(line);const auto sensor=record["input"]?record["input"]:record;
   const double stamp=record["frame_stamp"].as<double>();
   local.process(image,stamp,stamp,sensor);
   // Offline-only origin initialization; runtime requires operator reset.
   if(!initialized)initialized=local.reset(stamp);
   output<<local.json(stamp)<<'\n';++count;
  }
  if(std::getline(input,line))throw std::runtime_error("extra telemetry records");
  std::cout<<"Replayed "<<count<<" frames; initialized="<<initialized<<"\n";return initialized?0:3;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
