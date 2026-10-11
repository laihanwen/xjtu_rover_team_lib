// Offline same-library replay; accepts already rectified display frames only.
#include "auv_mapping/grid_mapper.hpp"
#include "auv_vision/cone_detector.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cstdlib>

int main(int argc,char** argv) {
  if(argc!=4 && argc!=5){std::cerr<<"Usage: auv_task_video_replay rectified_video output_dir step_frames [--independent]\n";return 2;}
  try {
    const int step=std::stoi(argv[3]);if(step<1)throw std::runtime_error("step must be positive");
    const bool independent=argc==5;
    if(independent && std::string(argv[4])!="--independent")throw std::runtime_error("unknown option");
    cv::VideoCapture capture(argv[1]);if(!capture.isOpened())throw std::runtime_error("video open failed");
    std::filesystem::create_directories(argv[2]);
    std::ofstream log(std::filesystem::path(argv[2])/"frames.csv");
    log<<"frame,geometry_valid,stable,orientation_valid,yellow_edge,confidence,cone_count,reason\n";
    auv_mapping::GridMapperConfig config;config.single_yellow_edge=true;config.white_grid_edges=true;
    auv_mapping::GridMapper mapper(config);auv_vision::ConeDetector detector;
    cv::Mat image;int index=0;
    while(capture.read(image)) {
      if(index%step){++index;continue;}
      // Sparse samples are independent observations, not continuous stability votes.
      if(step>1 || independent)mapper.reset();
      const auto grid=mapper.process(image);
      std::size_t cones=0;
      if(grid.geometry_valid) {
        const auto result=detector.process(grid.rectified);cones=result.observations.size();
        std::ofstream evidence(std::filesystem::path(argv[2])/(std::to_string(index)+"-cones.csv"));
        evidence<<"row,col,shape,confidence,contour_x,contour_y\n";
        for(const auto& cone:result.observations)for(const auto& point:cone.contour)
          evidence<<cone.row<<','<<cone.col<<','<<auv_vision::cone_shape_name(cone.shape)<<','<<cone.confidence<<','<<point.x<<','<<point.y<<'\n';
        if(!cv::imwrite((std::filesystem::path(argv[2])/(std::to_string(index)+"-cones.jpg")).string(),result.debug_image))throw std::runtime_error("image write failed");
      }
      log<<index<<','<<grid.geometry_valid<<','<<grid.stable<<','<<grid.orientation_valid<<','<<grid.yellow_edge<<','<<grid.confidence<<','<<cones<<','<<grid.reason<<'\n';
      if(!log)throw std::runtime_error("log write failed");
      if(index%(step*10)==0 && !cv::imwrite((std::filesystem::path(argv[2])/(std::to_string(index)+"-grid.jpg")).string(),grid.debug_image))throw std::runtime_error("debug write failed");
      ++index;
    }
    std::ofstream metadata(std::filesystem::path(argv[2])/"image-space.txt");
    metadata<<"already_rectified_display_video; no undistortion; no metric pose; white-edge diagnostic defaults\n";
    return 0;
  } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
