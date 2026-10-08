#include "../src/mission_recorder.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <chrono>
#include <limits>
#include <iostream>
static void require(bool x){if(!x)throw std::runtime_error("recorder regression");}
int main() try {
  const auto dir=std::filesystem::temp_directory_path()/
    ("auv-recorder-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::vector<std::uint8_t> jpg;
  cv::imencode(".jpg",cv::Mat(32,32,CV_8UC3,cv::Scalar(12,34,56)),jpg);
  {
    MissionRecorder r(dir.string(),1,1024);
    r.append("down",jpg,1,10,"\"camera-down\"","null");
    r.append("down",jpg,1,10,"\"camera-down\"","null"); // duplicate ignored
    r.append("down",jpg,3,10.5,"\"camera-down\"","{\"valid\":false}");
    r.append("front",jpg,1,10.5,"\"camera-front\"","null");
    r.append("down",jpg,4,11.5,"\"camera-down\"","null");
    bool refused=false;try{r.append("down",{1,2,3,4},5,12,"null","null");}catch(const std::runtime_error&){refused=true;}
    require(refused);
  }
  require(std::filesystem::file_size(dir/"down-1.mjpg")==2*jpg.size());
  require(std::filesystem::file_size(dir/"front-1.mjpg")==jpg.size());
  require(std::filesystem::file_size(dir/"down-2.mjpg")==jpg.size());
  std::ifstream index(dir/"down-1.frames.jsonl");std::string first,second,third;
  std::getline(index,first);std::getline(index,second);std::getline(index,third);
  require(first.find("\"offset\":0")!=std::string::npos);
  require(second.find("\"dropped_since_previous\":1")!=std::string::npos && third.empty());
  index.close();
  bool refused=false;try{MissionRecorder r(dir.string(),1,std::numeric_limits<std::uintmax_t>::max());}catch(const std::runtime_error&){refused=true;}
  require(refused);std::filesystem::remove_all(dir);
}
catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
