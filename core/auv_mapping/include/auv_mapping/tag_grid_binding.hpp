#pragma once
#include "auv_mapping/metric_grid_pose.hpp"
#include "auv_vision/apriltag_detector.hpp"

namespace auv_mapping {
struct TagGridBinding {
  bool ready{false};double x{0},y{0},yaw{0},stamp{0};std::uint64_t sequence{0};int votes{0};
  std::string reason{"no paired tag/grid observation"};
  cv::Point2d grid_to_tag(cv::Point2d p)const {
    const double dx=p.x-x,dy=p.y-y;return {std::cos(yaw)*dx+std::sin(yaw)*dy,-std::sin(yaw)*dx+std::cos(yaw)*dy};
  }
};
class TagGridBinder {
 public:
  TagGridBinding update(const auv_vision::AprilTagObservation& tag,const MetricGridPose& grid,
      const MetricGridCalibration& c,double size_m,double depth,double stamp) {
    auto reject=[&](const char* reason){value_={};value_.reason=reason;return value_;};
    if(tag.id!=18||!grid.valid||!grid.landmark_observation||!c.verified||!std::isfinite(stamp)||
       !std::isfinite(size_m)||size_m<=0||!std::isfinite(depth)||depth<0||depth>=c.pool_depth_m||
       std::abs(stamp-grid.stamp)>.001)return reject("synchronized tag/grid landmarks required");
    if(!std::isfinite(grid.row)||!std::isfinite(grid.col)||!std::all_of(grid.body_from_grid.begin(),grid.body_from_grid.end(),
        [](double x){return std::isfinite(x);}))return reject("nonfinite grid reference");
    if(value_.sequence==grid.sequence)return value_;
    c.validate();std::vector<cv::Point2f> pixels(tag.corners.begin(),tag.corners.end());
    for(int i=0;i<4;++i)if(!std::isfinite(pixels[i].x)||!std::isfinite(pixels[i].y)||
       cv::norm(pixels[i]-pixels[(i+1)%4])<12)return reject("tag border too small or invalid");
    const double half=size_m/2;std::vector<cv::Point3d> square{{-half,half,0},{half,half,0},{half,-half,0},{-half,-half,0}};
    cv::Vec3d rotation,translation;
    if(!cv::solvePnP(square,pixels,cv::Mat(3,3,CV_64F,const_cast<double*>(c.camera_matrix.data())),cv::noArray(),
                    rotation,translation,false,cv::SOLVEPNP_IPPE_SQUARE)||translation[2]<=0)return reject("tag PnP rejected");
    std::vector<cv::Point2d> projected;cv::projectPoints(square,rotation,translation,
      cv::Mat(3,3,CV_64F,const_cast<double*>(c.camera_matrix.data())),cv::noArray(),projected);
    double error=0;for(int i=0;i<4;++i)error+=std::pow(cv::norm(projected[i]-cv::Point2d(pixels[i])),2);
    if(!std::isfinite(error)||std::sqrt(error/4)>c.max_reprojection_px)return reject("tag reprojection inconsistent");
    cv::Mat rc;cv::Rodrigues(rotation,rc);const cv::Matx33d body_camera(c.camera_to_body.data()),body_grid(grid.body_from_grid.data());
    const cv::Matx33d camera_tag(rc);const auto body_tag=body_camera*camera_tag;
    const auto body_position=body_camera*translation+cv::Vec3d(c.camera_offset_m.data());
    const cv::Vec3d normal(body_grid(0,2),body_grid(1,2),body_grid(2,2));
    const cv::Vec3d tag_normal(body_tag(0,2),body_tag(1,2),body_tag(2,2));
    const auto offset=body_grid.t()*body_position;
    if(std::abs(normal.dot(tag_normal))<.9||std::abs(std::abs(offset[2])-(c.pool_depth_m-depth+
       cv::Vec3d(c.depth_offset_m.data()).dot(normal[2]>0?-normal:normal)))>c.height_tolerance_m)
      return reject("tag/grid floor planes inconsistent");
    const auto axis=body_grid.t()*cv::Vec3d(body_tag(0,0),body_tag(1,0),body_tag(2,0));
    const double x=grid.col*c.cell_size_m+offset[0],y=grid.row*c.cell_size_m+offset[1],yaw=std::atan2(axis[1],axis[0]);
    if(value_.votes&&(stamp<=value_.stamp||stamp-value_.stamp>.5||std::hypot(x-value_.x,y-value_.y)>.05||
       std::abs(std::remainder(yaw-value_.yaw,2*CV_PI))>.1))return reject("tag/grid binding discontinuity");
    value_.x=x;value_.y=y;value_.yaw=yaw;value_.stamp=stamp;value_.sequence=grid.sequence;
    value_.ready=++value_.votes>=5;value_.reason="observed tag-frame transform; competition heading not inferred";return value_;
  }
  void reset(){value_={};}
 private:TagGridBinding value_;
};
}
