#include "auv_mapping/tag_grid_binding.hpp"
#include <stdexcept>
static void check(bool v,const char* s){if(!v)throw std::runtime_error(s);}
int main(){
  auv_mapping::MetricGridCalibration c;c.verified=true;c.cell_size_m=.5;c.pool_depth_m=1.4;c.width=320;c.height=240;
  c.camera_matrix={150,0,160,0,150,120,0,0,1};c.distortion={0,0,0,0,0};
  c.camera_to_body={1,0,0,0,-1,0,0,0,-1};c.camera_offset_m=c.depth_offset_m={0,0,0};
  auv_mapping::MetricGridPose p;p.valid=p.landmark_observation=true;p.row=p.col=1.5;
  p.body_from_grid={1,0,0,0,1,0,0,0,1};
  auv_vision::AprilTagObservation tag;tag.id=18;
  std::vector<cv::Point3d> square{{-.069,.069,0},{.069,.069,0},{.069,-.069,0},{-.069,-.069,0}};
  std::vector<cv::Point2d> pixels;cv::projectPoints(square,cv::Vec3d(CV_PI,0,0),cv::Vec3d(-.55,.45,1),
    cv::Mat(3,3,CV_64F,c.camera_matrix.data()),cv::noArray(),pixels);
  for(int i=0;i<4;++i)tag.corners[i]=pixels[i];
  auv_mapping::TagGridBinder binder;auv_mapping::TagGridBinding binding;
  for(int i=0;i<6;++i){p.stamp=1+.05*i;p.sequence=i+1;binding=binder.update(tag,p,c,.138,.4,p.stamp);}
  check(binding.ready,"paired tag/grid observations become stable");
  check(std::hypot(binding.x-.2,binding.y-.3)<.01,"relative tag translation recovered");
  auto point=binding.grid_to_tag({.7,.8});check(cv::norm(point-cv::Point2d(.5,.5))<.01,"tag frame transform");
  check(!binder.update(tag,p,c,.138,.4,p.stamp+.1).ready,"different frame times rejected");
}
