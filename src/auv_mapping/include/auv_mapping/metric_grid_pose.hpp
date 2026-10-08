#pragma once
#include "auv_mapping/grid_mapper.hpp"
#include <opencv2/calib3d.hpp>
#include <array>
#include <cmath>
#include <vector>
#include <string>
#include <stdexcept>

namespace auv_mapping {
struct MetricGridCalibration {
  bool verified{false};
  double cell_size_m{0}, pool_depth_m{0}, height_tolerance_m{0.15}, max_reprojection_px{3};
  double minimum_confidence{0.8};
  int width{0},height{0};
  std::vector<double> camera_matrix,distortion,camera_to_body,camera_offset_m,depth_offset_m;
  void validate() const {
    if(!verified)return;
    if(!std::isfinite(cell_size_m)||cell_size_m<=0||!std::isfinite(pool_depth_m)||pool_depth_m<=0||
      !std::isfinite(height_tolerance_m)||height_tolerance_m<=0||
      !std::isfinite(max_reprojection_px)||max_reprojection_px<=0||
      !std::isfinite(minimum_confidence)||minimum_confidence<=0||minimum_confidence>1||width<16||height<16||
      camera_matrix.size()!=9||camera_to_body.size()!=9||camera_offset_m.size()!=3||depth_offset_m.size()!=3||
      (distortion.size()!=4&&distortion.size()!=5&&distortion.size()!=8&&distortion.size()!=12&&distortion.size()!=14))
      throw std::invalid_argument("metric grid calibration incomplete");
    for(const auto* v:{&camera_matrix,&distortion,&camera_to_body,&camera_offset_m,&depth_offset_m})
      for(double x:*v)if(!std::isfinite(x))throw std::invalid_argument("nonfinite metric grid calibration");
    const cv::Matx33d r(camera_to_body.data());
    if(cv::norm(cv::Mat(r*r.t()-cv::Matx33d::eye()))>1e-3||std::abs(cv::determinant(r)-1)>1e-3||
      camera_matrix[0]<=0||camera_matrix[4]<=0||std::abs(camera_matrix[8]-1)>1e-6)
      throw std::invalid_argument("invalid metric camera rotation or intrinsics");
  }
};
struct MetricGridPose {
  bool valid{false},surface_frame{false},semantic_match{false};std::string reason{"unavailable"};
  std::uint64_t sequence{0};double stamp{0},row{0},col{0},reprojection_px{0};
  std::array<double,9> body_from_grid{}; // columns: column, row, plane normal
};
// Corners must come from a single-yellow-edge mapper, in canonical TL/TR/BR/BL
// order. Input image is already undistorted using this calibration's intrinsics.
inline MetricGridPose metric_grid_pose(const GridResult& grid,const MetricGridCalibration& c,
  const cv::Size& size,double corrected_sensor_depth,double stamp,std::uint64_t sequence) {
  MetricGridPose out;out.stamp=stamp;out.sequence=sequence;
  if(!c.verified){out.reason="metric calibration unverified";return out;}
  c.validate();
  if(size.width!=c.width||size.height!=c.height||!grid.geometry_valid||!grid.stable||!grid.orientation_valid||
    grid.yellow_edge<0||grid.confidence<c.minimum_confidence||!std::isfinite(corrected_sensor_depth)||
    corrected_sensor_depth<0||corrected_sensor_depth>=c.pool_depth_m||!std::isfinite(stamp)||!sequence) {
    out.reason="grid, resolution or depth invalid";return out;
  }
  const double extent=3*c.cell_size_m;
  const std::vector<cv::Point3d> world{{0,0,0},{extent,0,0},{extent,extent,0},{0,extent,0}};
  const std::vector<cv::Point2f> pixels(grid.corners.begin(),grid.corners.end());
  const cv::Matx33d k(c.camera_matrix.data()), rcb(c.camera_to_body.data());
  cv::Mat rvec,tvec;
  if(!cv::solvePnP(world,pixels,k,cv::noArray(),rvec,tvec,false,cv::SOLVEPNP_ITERATIVE)) {
    out.reason="grid PnP failed";return out;
  }
  cv::Mat rotation;cv::Rodrigues(rvec,rotation);cv::Matx33d rgc;
  for(int i=0;i<9;++i)rgc.val[i]=rotation.at<double>(i/3,i%3);
  const cv::Vec3d t(tvec.at<double>(0),tvec.at<double>(1),tvec.at<double>(2));
  for(const auto& p:world)if((rgc*cv::Vec3d(p.x,p.y,p.z)+t)[2]<=0) {
    out.reason="grid behind camera";return out;
  }
  std::vector<cv::Point2d> projected;cv::projectPoints(world,rvec,tvec,k,cv::noArray(),projected);
  double error=0;for(std::size_t i=0;i<4;++i)error+=std::pow(cv::norm(projected[i]-cv::Point2d(pixels[i])),2);
  out.reprojection_px=std::sqrt(error/4);
  const cv::Vec3d offset(c.camera_offset_m.data());
  const cv::Vec3d body_in_grid=-rgc.t()*(t+rcb.t()*offset);
  const cv::Matx33d rbg=rcb*rgc;
  cv::Vec3d down(rbg(0,2),rbg(1,2),rbg(2,2));if(down[2]>0)down=-down;
  const cv::Vec3d sensor(c.depth_offset_m.data());
  const double expected_height=c.pool_depth_m-corrected_sensor_depth+down.dot(sensor);
  if(!std::isfinite(out.reprojection_px)||out.reprojection_px>c.max_reprojection_px||
    std::abs(down[2])<0.8||std::abs(std::abs(body_in_grid[2])-expected_height)>c.height_tolerance_m) {
    out.reason="grid projection, tilt or metric height inconsistent";return out;
  }
  out.row=body_in_grid[1]/c.cell_size_m;out.col=body_in_grid[0]/c.cell_size_m;
  if(!std::isfinite(out.row)||!std::isfinite(out.col)||out.row<0||out.row>=3||out.col<0||out.col>=3) {
    out.reason="body outside grid";return out;
  }
  std::copy(rbg.val,rbg.val+9,out.body_from_grid.begin());
  out.valid=true;out.reason="fresh absolute yellow-bottom grid pose";return out;
}
}
