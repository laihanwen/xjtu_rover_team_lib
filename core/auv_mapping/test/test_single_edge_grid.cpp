#include "auv_mapping/grid_mapper.hpp"
#include "auv_vision/cone_detector.hpp"
#include "auv_core/semantic_map.hpp"
#include <opencv2/imgproc.hpp>
#include <stdexcept>
static void check(bool x,const std::string& why){if(!x)throw std::runtime_error(why);}
static cv::Mat scene(bool yellow=true,bool lines=true){
  cv::Mat image(640,640,CV_8UC3,cv::Scalar(215,215,215));
  cv::rectangle(image,{80,80},{560,560},{20,20,20},12);
  if(lines)for(int x:{240,400}){
    cv::line(image,{x,80},{x,560},{20,20,20},8);
    cv::line(image,{80,x},{560,x},{20,20,20},8);
  }
  if(yellow)cv::line(image,{80,560},{560,560},{0,255,255},14);
  cv::circle(image,{160,160},35,{0,80,255},-1);
  cv::circle(image,{480,160},35,{0,80,255},-1);
  cv::rectangle(image,{125,445},{195,515},{0,80,255},-1);
  cv::rectangle(image,{445,445},{515,515},{0,80,255},-1);
  return image;
}
void test_single_edge_grid(){
  auv_mapping::GridMapperConfig cfg;cfg.single_yellow_edge=true;cfg.stable_frames=1;cfg.yellow_oriented_frames=1;
  for(int rotation=0;rotation<4;++rotation){
    cv::Mat image=scene();for(int i=0;i<rotation;++i)cv::rotate(image,image,cv::ROTATE_90_CLOCKWISE);
    auv_mapping::GridMapper mapper(cfg);auto grid=mapper.process(image);
    check(grid.stable && grid.orientation_valid,"single yellow edge: "+grid.reason);
    auto detections=auv_vision::ConeDetector().process(grid.rectified);
    auto map=auv_core::fuse_semantic_map(grid,detections.observations,true,{},4);
    check(map.complete,"four cones must be classified");
    check(map.grid.cells[0].cell.object_type=="circle_cone" &&
      map.grid.cells[6].cell.object_type=="square_cone","yellow edge must orient map bottom");
    for(const auto& cone:detections.observations)check(!cone.contour.empty(),"observed silhouette missing");
    auto excess=detections.observations;auto c=excess[0];c.row=1;c.col=1;excess.push_back(c);
    check(!auv_core::fuse_semantic_map(grid,excess,true,{},4).complete,"extra cones cannot complete map");
  }
  check(!auv_mapping::GridMapper(cfg).process(scene(false)).geometry_valid,"missing yellow edge accepted");
  cv::Mat ambiguous=scene();cv::line(ambiguous,{80,80},{560,80},{0,255,255},14);
  check(!auv_mapping::GridMapper(cfg).process(ambiguous).geometry_valid,"ambiguous orientation accepted");
  check(!auv_mapping::GridMapper(cfg).process(scene(true,false)).geometry_valid,"missing grid dividers accepted");
}
