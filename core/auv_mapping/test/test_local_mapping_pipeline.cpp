#include "auv_mapping/local_mapping_pipeline.hpp"
#include "auv_mapping/local_pose_fusion.hpp"
#include <stdexcept>
#include <limits>
using namespace auv_mapping;
static void check(bool v,const char* reason){if(!v)throw std::runtime_error(reason);}
static void measured_plane_regression() {
  MetricGridCalibration c;c.verified=true;c.cell_size_m=.5;c.pool_depth_m=1.4;
  c.width=320;c.height=240;c.camera_matrix={250,0,160,0,250,120,0,0,1};
  c.distortion={0,0,0,0,0};c.camera_to_body={0,1,0,1,0,0,0,0,-1};
  c.camera_offset_m={.2,.03,.04};c.depth_offset_m={.01,-.02,.03};
  const std::vector<cv::Point3d> world{{.5,.5,0},{1,.5,0},{.5,1,0},{1,1,0}};
  for(double tilt:{0.,.12}) {
    PlaneSample sample;sample.stamp=1;sample.depth=.4;sample.roll=tilt;sample.pitch=-tilt/2;sample.yaw=.2;
    const auto values=attitude(sample.roll,sample.pitch,sample.yaw);
    const cv::Matx33d level_from_body(values.data()),rcb(c.camera_to_body.data());
    const cv::Matx33d level_from_grid(0,1,0,-1,0,0,0,0,1);
    const double height=1.4-sample.depth-(level_from_body*cv::Vec3d(c.depth_offset_m.data()))[2];
    const cv::Vec3d translation=-level_from_grid*cv::Vec3d(.884,.844,height);
    const auto camera_from_grid=rcb.t()*level_from_body.t()*level_from_grid;
    const auto t=rcb.t()*(level_from_body.t()*translation-cv::Vec3d(c.camera_offset_m.data()));
    cv::Mat rvec;cv::Rodrigues(camera_from_grid,rvec);
    std::vector<cv::Point2d> projected;
    cv::projectPoints(world,rvec,t,cv::Matx33d(c.camera_matrix.data()),cv::noArray(),projected);
    std::vector<cv::Point2f> pixels(projected.begin(),projected.end());
    auto exact=local_plane_pose(world,pixels,c,{320,240},sample,1);
    check(exact.valid&&std::hypot(exact.col*.5-.884,exact.row*.5-.844)<1e-6,
      "measured plane honors attitude and camera/depth offsets");
    pixels[0].x+=1;pixels[3].x-=1;
    auto noisy=local_plane_pose(world,pixels,c,{320,240},sample,2);
    check(noisy.valid&&std::hypot(noisy.col*.5-.884,noisy.row*.5-.844)<.01,
      "small local pixel errors cannot create free-PnP tilt drift");
    pixels[0].x+=20;
    check(!local_plane_pose(world,pixels,c,{320,240},sample,3).valid,
      "measured plane rejects inconsistent landmarks without relaxing pixel gate");
  }
}
int main() {
  measured_plane_regression();
  MetricGridCalibration c;c.verified=true;c.cell_size_m=.5;c.pool_depth_m=1.4;
  c.width=320;c.height=240;c.camera_matrix={150,0,160,0,150,120,0,0,1};
  c.distortion={0,0,0,0,0};c.camera_to_body={1,0,0,0,-1,0,0,0,-1};
  c.camera_offset_m=c.depth_offset_m={0,0,0};
  PlaneSample sensor;sensor.stamp=1;sensor.depth=.4;
  cv::Mat image(240,320,CV_8UC3,cv::Scalar(70,70,70));
  auto pixel=[](double x,double y){return cv::Point(int(std::lround(160+150*(x-.75))),int(std::lround(120+150*(1.2-y))));};
  for(int i=0;i<4;++i) {
    cv::line(image,pixel(i*.5,0),pixel(i*.5,1.5),{255,255,255},5);
    cv::line(image,pixel(0,i*.5),pixel(1.5,i*.5),i==3?cv::Scalar(0,255,255):cv::Scalar(255,255,255),5);
  }
  // Top row is outside the image: no complete outer rectangle is available.
  LocalMappingPipeline pipeline({});
  auto found=pipeline.process(image,c,sensor,1,1,0);
  check(found.geometry.valid,"cropped yellow boundary topology must resolve");
  check(found.pose.valid,"local landmark PnP must resolve");
  check(std::abs(found.pose.row-2.4)<.08&&std::abs(found.pose.col-1.5)<.08,"metric local body position");
  auto stray_pipe=image.clone();
  cv::line(stray_pipe,pixel(.75,0),pixel(.75,1.5),{255,255,255},5);
  const auto filtered=pipeline.process(stray_pipe,c,sensor,2,1,0);
  check(filtered.geometry.valid&&filtered.pose.valid,
    "one off-lattice pipe fragment cannot veto the measured grid");
  auto cone_image=image.clone();cv::circle(cone_image,pixel(1.25,1.25),15,{0,80,255},-1);
  cv::Mat partial(image.size(),CV_8U,cv::Scalar(255));partial(cv::Rect(209,84,5,5))=0;
  const auto partial_cone=pipeline.process(cone_image,c,sensor,2,1,0,partial);
  check(partial_cone.pose.valid,"partial-cell cone retains associated geometry");
  const auto& classified=partial_cone.observation.cells[8];
  check(!classified.full_cell_visible&&classified.type==LocalCellClass::circle&&!classified.contour.empty(),
    "complete cone silhouette can classify a partly observed cell");
  partial(cv::Rect(229,95,10,40))=0;
  const auto clipped_cone=pipeline.process(cone_image,c,sensor,3,1,0,partial);
  check(clipped_cone.pose.valid&&clipped_cone.observation.cells[8].type==LocalCellClass::unknown,
    "invisible silhouette pixels cannot authorize cone class");
  LocalFloorModel empty_floor;empty_floor.verified=true;
  empty_floor.lower={0,0,50};empty_floor.upper={179,15,110};
  LocalMappingPipeline partial_floor({}, {},empty_floor);
  LocalMapAccumulator partial_map;partial_map.reset(1,"yellow_bottom_pipe_centres");
  cv::Mat left(image.size(),CV_8U,cv::Scalar(255)),right=left.clone();
  left(cv::Rect(175,155,19,66))=0;right(cv::Rect(126,155,19,66))=0;
  std::string coverage_reason;
  for(int frame=1;frame<=6;++frame) {
    auto sample=sensor;sample.stamp=frame*.1;
    auto observation=partial_floor.process(image,c,sample,frame,1,0,frame<=3?left:right);
    check(observation.pose.valid&&!observation.observation.cells[4].full_cell_visible,
      "each partial floor frame stays associated without a whole visible cell");
    check(partial_map.update(observation.observation,sample.stamp,coverage_reason),"partial floor evidence accepted");
    if(frame<=3)check(partial_map.cells()[4].type==LocalCellClass::unknown,"one cropped view cannot certify empty");
  }
  check(partial_map.cells()[4].type==LocalCellClass::empty,"image-derived complementary views certify empty interior");
  GridMapperConfig ridge;ridge.local_pipe_centres=true;
  check(LocalMappingPipeline({},ridge).process(image,c,sensor,2,1,0).pose.valid,"experimental ridge resolves controlled fixture");
  auv_vision::ConeDetectorConfig alternative;alternative.mode="agreement";
  LocalFloorModel floor;floor.verified=true;floor.lower={0,0,0};floor.upper={179,255,255};
  auto no_model=LocalMappingPipeline(alternative,{},floor).process(image,c,sensor,2,1,0);
  check(no_model.pose.valid,"alternative backend preserves measured geometry");
  check(std::all_of(no_model.observation.cells.begin(),no_model.observation.cells.end(),
      [](const auto& cell){return cell.type==LocalCellClass::unknown;}),"unavailable model cannot authorize empty cells");
  c.verified=false;
  check(!pipeline.process(image,c,sensor,2,1,0).pose.valid,"unverified model cannot navigate");
  c.verified=true;
  cv::Mat missing(image.size(),CV_8U,cv::Scalar(0));
  check(!pipeline.process(image,c,sensor,3,1,0,missing).geometry.valid,"invisible remap cannot produce landmarks");
  LocalPoseFusion fusion;fusion.reset(1,0);
  auv_control::ObservationPose odom{true,true,1,1,0,0,0};
  auto p=found.pose;
  auto anchored=fusion.update(p,odom,1,1,.5,.2);
  check(anchored.valid&&anchored.sequence,"absolute anchor");
  odom.stamp=1.1;odom.x=.01;
  auto propagated=fusion.update({},odom,1.1,1,.5,.2);
  check(propagated.valid&&!propagated.landmark_observation&&!propagated.semantic_match,"odometry cannot reacquire by itself");
  check(propagated.sequence>anchored.sequence,"fusion sequence is independent of camera sequence");
  check(fusion.update({},odom,1.1,1,.5,.2).sequence==propagated.sequence,"cached odometry is not a new observation");
  odom.stamp=2.1;
  check(!fusion.update({},odom,2.1,1,.5,.2).valid,"absolute drift budget expires");
  fusion.reset(2,3);
  check(!fusion.update(p,odom,3,2,.5,.2).valid,"old-stage absolute pose cannot anchor");
  p.stamp=3.1;odom.stamp=3.1;
  check(fusion.update(p,odom,3.1,2,.5,.2).valid,"new-stage anchor allowed");
  auto conflict=p;conflict.stamp=3.2;conflict.row+=1;
  check(!fusion.update(conflict,odom,3.2,2,.5,.2).valid,"landmark jump rejected");
  check(!fusion.update({},odom,3.2,2,.5,.2).valid,"rejected correction cannot revive old anchor");
  fusion.reset(3,4);p.stamp=4;odom={true,true,2,4,0,0,0};
  check(fusion.update(p,odom,4,3,.5,.2).valid,"stationary landmark anchor");
  fusion.reject_landmark();
  check(!fusion.snapshot(4).valid&&!fusion.update({},odom,4,3,.5,.2).valid,
    "contradictory topology stops all control propagation");
  odom.stamp=7;
  check(!fusion.update({},odom,7,3,.5,.2).valid,"long gap cannot propagate odometry");
  conflict=p;conflict.stamp=7;conflict.col+=1;
  check(!fusion.update(conflict,odom,7,3,.5,.2).valid,"long gap cannot authorize adjacent-column reacquisition");
  p.stamp=7.1;odom.stamp=7.1;
  check(fusion.update(p,odom,7.1,3,.5,.2).valid,"fresh matching landmark can reacquire after gap");
}
