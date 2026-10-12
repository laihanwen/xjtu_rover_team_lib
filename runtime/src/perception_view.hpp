#pragma once
#include "auv_vision/apriltag_detector.hpp"
#include "auv_mapping/local_map_accumulator.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <iomanip>

namespace auv_runtime {
inline std::string view_quote(const std::string& value) {
  std::ostringstream out;out<<'"';
  for(unsigned char c:value) {
    if(c=='"'||c=='\\')out<<'\\'<<c;
    else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
    else out<<c;
  }
  out<<'"';return out.str();
}
inline std::string view_number(double v){return std::isfinite(v)?std::to_string(v):"null";}
inline const char* view_class(auv_mapping::LocalCellClass c) {
  return c==auv_mapping::LocalCellClass::circle?"circle":c==auv_mapping::LocalCellClass::square?"square":
         c==auv_mapping::LocalCellClass::empty?"empty":"unknown";
}
inline std::string view_base64(const std::vector<std::uint8_t>& bytes) {
  static const char* chars="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;out.reserve((bytes.size()+2)/3*4);
  for(std::size_t i=0;i<bytes.size();i+=3) {
    unsigned v=unsigned(bytes[i])<<16;
    if(i+1<bytes.size())v|=unsigned(bytes[i+1])<<8;
    if(i+2<bytes.size())v|=bytes[i+2];
    out+=chars[(v>>18)&63];out+=chars[(v>>12)&63];
    out+=i+1<bytes.size()?chars[(v>>6)&63]:'=';out+=i+2<bytes.size()?chars[v&63]:'=';
  }return out;
}
// Read-only evidence; none of these fields can authorize motion or update the map.
struct PerceptionView {
  std::uint64_t sequence{0},epoch{0},tag_sequence{0},pose_sequence{0};double stamp{0},tag_stamp{0},pose_stamp{0};
  std::string pose_model,pose_source;
  std::string camera,tag_camera,model,phase,reason{"waiting for perception frame"},pose_reason,cone_reason,cone_mode;
  int width{0},height{0},nodes{0},hypotheses{0},red_components{0},colour_cones{-1},shape_cones{-1};
  bool local_enabled{false},sensor_ready{false},geometry_valid{false},association_unique{false},pose_valid{false},map_complete{false};
  double row{NAN},col{NAN},reprojection{NAN};
  std::vector<auv_vision::AprilTagObservation> tags;
  std::vector<int> target_ids{18};bool tag_confirmed{false};
  std::array<auv_mapping::AccumulatedCell,9> cells{};std::array<bool,9> visible{};
  std::vector<std::uint8_t> jpeg;
  void attach_image(const cv::Mat& image,const cv::Mat& debug,bool tags_same_frame) {
    try {
      cv::Mat overlay=(debug.empty()?image:debug).clone();cv::Mat hsv,a,b,mask;
      cv::cvtColor(image,hsv,cv::COLOR_BGR2HSV);
      cv::inRange(hsv,cv::Scalar(0,80,50),cv::Scalar(20,255,255),a);
      cv::inRange(hsv,cv::Scalar(165,80,50),cv::Scalar(179,255,255),b);cv::bitwise_or(a,b,mask);
      std::vector<std::vector<cv::Point>> regions;cv::findContours(mask,regions,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
      for(const auto& region:regions)if(std::abs(cv::contourArea(region))>=40&&red_components<64) {
        cv::polylines(overlay,region,true,{0,180,255},1);++red_components;
      }
      if(tags_same_frame)for(const auto& tag:tags) {
        std::vector<cv::Point> points;for(auto p:tag.corners)points.push_back(p);
        cv::polylines(overlay,points,true,{70,230,140},2);
        cv::putText(overlay,"ID "+std::to_string(tag.id),points[0],cv::FONT_HERSHEY_SIMPLEX,.5,{70,230,140},1);
      }
      cv::putText(overlay,"DIAGNOSTIC / "+camera+" / frame "+std::to_string(sequence),{8,20},cv::FONT_HERSHEY_SIMPLEX,.4,{255,255,255},1);
      if(overlay.cols>800)cv::resize(overlay,overlay,{800,int(800.*overlay.rows/overlay.cols)});
      cv::imencode(".jpg",overlay,jpeg,{cv::IMWRITE_JPEG_QUALITY,65});
      if(jpeg.size()>256*1024)jpeg.clear();
    }catch(const cv::Exception&){jpeg.clear();reason+="; diagnostic image unavailable";}
  }
  std::string json(double now,double pose_timeout,double tag_timeout)const {
    auto fresh=[now](double t,double limit){return t>0&&std::isfinite(t)&&std::isfinite(now)&&now>=t&&now-t<=limit;};
    const bool frame=fresh(stamp,1.5)&&sequence,tag_fresh=fresh(tag_stamp,tag_timeout)&&tag_sequence;
    const bool pose=frame&&pose_valid&&std::isfinite(row)&&std::isfinite(col)&&fresh(pose_stamp,pose_timeout);
    std::ostringstream out;out.precision(15);
    out<<"{\"schema\":\"auv.perception.v1\",\"diagnostic_only\":true,\"sequence\":"<<sequence
      <<",\"epoch\":"<<epoch<<",\"stamp\":"<<view_number(stamp)<<",\"age_sec\":"<<(stamp>0?view_number(now-stamp):"null")
      <<",\"frame_fresh\":"<<(frame?"true":"false")<<",\"camera\":"<<view_quote(camera)<<",\"model\":"<<view_quote(model)
      <<",\"phase\":"<<view_quote(phase)<<",\"width\":"<<width<<",\"height\":"<<height
      <<",\"local_enabled\":"<<(local_enabled?"true":"false")<<",\"sensor_ready\":"<<(frame&&sensor_ready?"true":"false")
      <<",\"geometry_valid\":"<<(frame&&geometry_valid?"true":"false")<<",\"association_unique\":"<<(frame&&association_unique?"true":"false")
      <<",\"nodes\":"<<nodes<<",\"hypotheses\":"<<hypotheses<<",\"reason\":"<<view_quote(reason)
      <<",\"red_components\":"<<red_components<<",\"colour_cones\":"<<colour_cones<<",\"shape_cones\":"<<shape_cones
      <<",\"cone_mode\":"<<view_quote(cone_mode)<<",\"cone_reason\":"<<view_quote(cone_reason)
      <<",\"tag_camera\":"<<view_quote(tag_camera)<<",\"tag_sequence\":"<<tag_sequence<<",\"tag_fresh\":"<<(tag_fresh?"true":"false")<<",\"tags\":[";
    if(tag_fresh)for(std::size_t i=0;i<tags.size();++i) {
      if(i)out<<',';out<<"{\"id\":"<<tags[i].id<<",\"corners\":[";
      for(int j=0;j<4;++j){if(j)out<<',';out<<'['<<view_number(tags[i].corners[j].x)<<','<<view_number(tags[i].corners[j].y)<<']';}out<<"]}";
    }
    out<<"],\"target_ids\":[";
    for(std::size_t i=0;i<target_ids.size();++i){if(i)out<<',';out<<target_ids[i];}
    out<<"],\"tag_confirmed\":"<<(tag_confirmed?"true":"false")<<",\"pose_valid\":"<<(pose?"true":"false")<<",\"pose_age_sec\":"<<(pose_stamp>0?view_number(now-pose_stamp):"null")
      <<",\"pose_reason\":"<<view_quote(pose_reason)<<",\"pose_sequence\":"<<pose_sequence<<",\"pose_model\":"<<view_quote(pose_model)<<",\"pose_source\":"<<view_quote(pose_source)
      <<",\"row\":"<<(pose?view_number(row):"null")<<",\"col\":"<<(pose?view_number(col):"null")
      <<",\"reprojection_px\":"<<view_number(reprojection)<<",\"map_complete\":"<<(map_complete?"true":"false")<<",\"cells\":[";
    for(std::size_t i=0;i<cells.size();++i) {
      if(i)out<<',';const auto& c=cells[i];
      out<<"{\"row\":"<<i/3<<",\"col\":"<<i%3<<",\"type\":"<<view_quote(view_class(c.type))
        <<",\"observed\":"<<(c.observed?"true":"false")<<",\"visible_now\":"<<(frame&&visible[i]?"true":"false")
        <<",\"conflict\":"<<(c.conflict?"true":"false")<<",\"votes\":["<<c.votes[0]<<','<<c.votes[1]<<','<<c.votes[2]<<"]}";
    }
    out<<"],\"image_url\":"<<view_quote(frame&&!jpeg.empty()?"data:image/jpeg;base64,"+view_base64(jpeg):"")<<'}';return out.str();
  }
};
}
