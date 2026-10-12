#pragma once
#include "auv_mapping/metric_grid_pose.hpp"
#include "auv_mapping/plane_odometry.hpp"

namespace auv_mapping {
// Local geometry already assumes a measured horizontal floor. Preserve that
// constraint when estimating pose instead of fitting a free planar PnP tilt.
inline MetricGridPose local_plane_pose(const std::vector<cv::Point3d>& world,
    const std::vector<cv::Point2f>& pixels,const MetricGridCalibration& c,
    const cv::Size& size,const PlaneSample& sample,std::uint64_t sequence) {
  MetricGridPose out;out.stamp=sample.stamp;out.sequence=sequence;
  if(!c.verified){out.reason="metric calibration unverified";return out;}
  c.validate();
  if(size!=cv::Size(c.width,c.height)||world.size()<4||world.size()!=pixels.size()||!sequence||
      !std::isfinite(sample.depth)||sample.depth<0||sample.depth>=c.pool_depth_m) {
    out.reason="local plane landmarks invalid";return out;
  }
  PlaneCalibration plane;plane.verified=true;plane.pool_depth=c.pool_depth_m;
  std::copy(c.camera_to_body.begin(),c.camera_to_body.end(),plane.camera_to_body.begin());
  std::copy(c.camera_offset_m.begin(),c.camera_offset_m.end(),plane.camera_offset.begin());
  std::copy(c.depth_offset_m.begin(),c.depth_offset_m.end(),plane.depth_offset.begin());
  std::vector<cv::Point2d> measured;std::vector<cv::Point2f> hull_points;
  cv::Point2d centre_world{},centre_measured{};double height=0;
  for(std::size_t i=0;i<world.size();++i) {
    const auto& w=world[i];Vec3 q;
    if(!std::isfinite(w.x)||!std::isfinite(w.y)||w.z!=0||
        !floor_point(plane,sample,(pixels[i].x-c.camera_matrix[2])/c.camera_matrix[0],
          (pixels[i].y-c.camera_matrix[5])/c.camera_matrix[4],q)) {
      out.reason="local plane projection invalid";return out;
    }
    measured.emplace_back(q[0],q[1]);hull_points.emplace_back(w.x,w.y);
    centre_world+=cv::Point2d(w.x,w.y);centre_measured+=measured.back();height=-q[2];
  }
  std::vector<cv::Point2f> hull;cv::convexHull(hull_points,hull);
  if(hull.size()<3||cv::contourArea(hull)<.1*c.cell_size_m*c.cell_size_m) {
    out.reason="collinear or insufficient landmark baseline";return out;
  }
  centre_world*=1./world.size();centre_measured*=1./world.size();
  double dot=0,cross=0;
  for(std::size_t i=0;i<world.size();++i) {
    const auto w=cv::Point2d(world[i].x,world[i].y)-centre_world;
    const auto q=measured[i]-centre_measured;
    dot+=w.dot(q);cross+=w.x*q.y-w.y*q.x;
  }
  if(std::hypot(dot,cross)<1e-10){out.reason="local plane orientation degenerate";return out;}
  const double a=std::atan2(cross,dot),cs=std::cos(a),sn=std::sin(a);
  const cv::Matx33d level_from_grid(cs,-sn,0,sn,cs,0,0,0,1);
  const cv::Vec3d translation= cv::Vec3d(centre_measured.x,centre_measured.y,-height)-
    level_from_grid*cv::Vec3d(centre_world.x,centre_world.y,0);
  const auto attitude_values=attitude(sample.roll,sample.pitch,sample.yaw);
  const cv::Matx33d level_from_body(attitude_values.data()),rcb(c.camera_to_body.data());
  const cv::Matx33d body_from_grid=level_from_body.t()*level_from_grid;
  if(std::abs(body_from_grid(2,2))<.8) {
    out.reason="local plane tilt outside supported range";return out;
  }
  const cv::Matx33d camera_from_grid=rcb.t()*body_from_grid;
  const cv::Vec3d camera_translation=rcb.t()*(level_from_body.t()*translation-cv::Vec3d(c.camera_offset_m.data()));
  cv::Mat rvec;cv::Rodrigues(camera_from_grid,rvec);
  std::vector<cv::Point2d> projected;
  cv::projectPoints(world,rvec,camera_translation,cv::Matx33d(c.camera_matrix.data()),cv::noArray(),projected);
  double error=0;
  for(std::size_t i=0;i<world.size();++i) {
    if((camera_from_grid*cv::Vec3d(world[i].x,world[i].y,0)+camera_translation)[2]<=0) {
      out.reason="grid behind camera";return out;
    }
    error+=std::pow(cv::norm(projected[i]-cv::Point2d(pixels[i])),2);
  }
  out.reprojection_px=std::sqrt(error/world.size());
  if(!std::isfinite(out.reprojection_px)||out.reprojection_px>c.max_reprojection_px) {
    out.reason="local measured plane residual inconsistent";return out;
  }
  const auto position=-level_from_grid.t()*translation;
  out.col=position[0]/c.cell_size_m;out.row=position[1]/c.cell_size_m;
  const double margin=c.outside_margin_cells;
  if(!std::isfinite(out.row)||!std::isfinite(out.col)||out.row<-margin||out.row>=3+margin||
      out.col<-margin||out.col>=3+margin) {out.reason="body outside grid";return out;}
  std::copy(body_from_grid.val,body_from_grid.val+9,out.body_from_grid.begin());
  out.valid=true;out.reason="fresh measured-plane local grid pose";return out;
}
}
