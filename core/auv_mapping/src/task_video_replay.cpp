// Offline same-library replay; accepts already rectified display frames only.
#include "auv_mapping/grid_mapper.hpp"
#include "auv_vision/cone_detector.hpp"
#include "auv_vision/apriltag_detector.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
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
    log<<"frame,geometry_valid,stable,orientation_valid,yellow_edge,confidence,cone_count,tag18,reason\n";
    std::ofstream tag_log(std::filesystem::path(argv[2])/"tags.csv");
    tag_log<<"frame,id,corner,x,y\n";
    auv_vision::AprilTagDetector tags("tag16h5");
    auv_mapping::GridMapperConfig config;config.single_yellow_edge=true;config.white_grid_edges=true;
    auv_mapping::GridMapper mapper(config);auv_vision::ConeDetector detector;
    std::ofstream candidates(std::filesystem::path(argv[2])/"local-candidates.csv");
    candidates<<"frame,kind,candidate,point,x,y,classification,association\n";
    cv::Mat image;int index=0;
    while(capture.read(image)) {
      if(index%step){++index;continue;}
      // Sparse samples are independent observations, not continuous stability votes.
      if(step>1 || independent)mapper.reset();
      bool tag18=false;
      for(const auto& tag:tags.detect(image)) {
        tag18|=tag.id==18;
        for(std::size_t corner=0;corner<tag.corners.size();++corner)
          tag_log<<index<<','<<tag.id<<','<<corner<<','<<tag.corners[corner].x<<','<<tag.corners[corner].y<<'\n';
      }
      // Image-space diagnostics only: never interpret an arbitrary camera
      // image as nine cells or classify a red stripe as a whole cone.
      const auto colour=detector.process(image);
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(colour.mask.clone(),contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
      cv::Mat local_debug=image.clone();int candidate=0;
      for(const auto& contour:contours) {
        if(std::abs(cv::contourArea(contour))<20)continue;
        for(std::size_t point=0;point<contour.size();++point)
          candidates<<index<<",red_component,"<<candidate<<','<<point<<','<<contour[point].x<<','<<contour[point].y<<",unknown,unverified\n";
        cv::drawContours(local_debug,std::vector<std::vector<cv::Point>>{contour},-1,{0,0,255},1);++candidate;
      }
      cv::Mat hsv,white,yellow,pipe,edges;
      cv::cvtColor(image,hsv,cv::COLOR_BGR2HSV);
      cv::inRange(hsv,cv::Scalar(0,0,config.white_v_min),cv::Scalar(179,config.white_s_max,255),white);
      cv::inRange(hsv,config.hsv_lower,config.hsv_upper,yellow);cv::bitwise_or(white,yellow,pipe);
      cv::Canny(pipe,edges,60,160);std::vector<cv::Vec4i> lines;
      cv::HoughLinesP(edges,lines,1,CV_PI/180,40,40,8);
      if(lines.size()<=80)for(std::size_t line=0;line<lines.size();++line) {
        const auto p=lines[line];
        candidates<<index<<",pipe_segment,"<<line<<",0,"<<p[0]<<','<<p[1]<<",unknown,unverified\n";
        candidates<<index<<",pipe_segment,"<<line<<",1,"<<p[2]<<','<<p[3]<<",unknown,unverified\n";
        cv::line(local_debug,{p[0],p[1]},{p[2],p[3]},{255,0,255},1);
      }
      if(index%(step*10)==0&&!cv::imwrite((std::filesystem::path(argv[2])/(std::to_string(index)+"-local.jpg")).string(),local_debug))
        throw std::runtime_error("local diagnostic image write failed");
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
      std::string reason;for(char ch:grid.reason){reason+=ch;if(ch=='"')reason+='"';}
      log<<index<<','<<grid.geometry_valid<<','<<grid.stable<<','<<grid.orientation_valid<<','<<grid.yellow_edge<<','<<grid.confidence<<','<<cones<<','<<tag18<<",\""<<reason<<"\"\n";
      if(!log||!tag_log||!candidates)throw std::runtime_error("log write failed");
      if(index%(step*10)==0 && !cv::imwrite((std::filesystem::path(argv[2])/(std::to_string(index)+"-grid.jpg")).string(),grid.debug_image))throw std::runtime_error("debug write failed");
      ++index;
    }
    std::ofstream metadata(std::filesystem::path(argv[2])/"image-space.txt");
    metadata<<"already_rectified_display_video; no undistortion; no metric pose; white-edge diagnostic defaults\n";
    return 0;
  } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
