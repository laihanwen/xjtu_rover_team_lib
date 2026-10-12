#pragma once
#include "auv_mapping/local_grid_detector.hpp"
#include "auv_mapping/local_map_accumulator.hpp"
#include "auv_vision/cone_detector.hpp"

namespace auv_mapping {
struct LocalMappingResult {
  std::size_t colour_cones{0},shape_cones{0};std::string cone_reason;
  LocalGridDetection geometry;
  MetricGridPose pose;
  LocalMapObservation observation;
};
struct LocalFloorModel {
  bool verified{false};cv::Scalar lower{},upper{};double minimum_support{.95};
  void validate()const {
    if(!verified)return;
    if(!std::isfinite(minimum_support)||minimum_support<.9||minimum_support>1)
      throw std::invalid_argument("invalid local empty-floor support");
    for(int i=0;i<3;++i)if(!std::isfinite(lower[i])||!std::isfinite(upper[i])||
        lower[i]<0||lower[i]>upper[i]||upper[i]>(i==0?179:255))
      throw std::invalid_argument("invalid local empty-floor HSV model");
  }
};
class LocalMappingPipeline {
 public:
  explicit LocalMappingPipeline(auv_vision::ConeDetectorConfig c,GridMapperConfig grid={},LocalFloorModel floor={}):grid_(grid),cones_(c),floor_(floor),cone_mode_(c.mode){floor_.validate();}
  LocalMappingResult process(const cv::Mat& image,const MetricGridCalibration& calibration,
      const PlaneSample& sensor,std::uint64_t sequence,std::uint64_t epoch,int camera,const cv::Mat& valid_pixels=cv::Mat(),const MetricGridPose& prior=MetricGridPose())const {
    LocalMappingResult result;result.geometry=grid_.process(image,calibration,sensor,valid_pixels);
    result.pose.stamp=sensor.stamp;result.pose.sequence=sequence;
    result.pose.reason=result.geometry.reason;
    if(!result.geometry.valid&&prior.valid&&std::isfinite(prior.stamp)&&
        std::all_of(prior.body_from_grid.begin(),prior.body_from_grid.end(),[](double x){return std::isfinite(x);})&&
        sensor.stamp>=prior.stamp&&sensor.stamp-prior.stamp<=.3&&
        result.geometry.association.hypotheses.size()>1&&result.geometry.nodes.size()>=4) {
      LocalGridDetection selected;int matches=0;
      const auto candidates=result.geometry.association.hypotheses.size();
      for(std::size_t i=0;i<candidates;++i) {
        auto candidate=grid_.resolve(result.geometry,image,valid_pixels,i);
        if(!candidate.valid)continue;
        std::vector<cv::Point3d> points;
        for(const auto& p:candidate.association.hypotheses[0].nodes)
          points.emplace_back(p[1]*calibration.cell_size_m,p[0]*calibration.cell_size_m,0);
        const auto pose=metric_landmark_pose(points,candidate.pixels,calibration,image.size(),sensor.depth,sensor.stamp,sequence);
        if(!pose.valid||std::hypot(pose.row-prior.row,pose.col-prior.col)*calibration.cell_size_m>.12)continue;
        double rotation_error=0;
        for(int j=0;j<9;++j)rotation_error+=std::pow(pose.body_from_grid[j]-prior.body_from_grid[j],2);
        if(rotation_error>.12)continue;
        ++matches;selected=std::move(candidate);
      }
      if(matches==1){result.geometry=std::move(selected);result.geometry.reason="unique temporal metric association";}
      else result.geometry.reason="temporal association ambiguous or discontinuous";
    }
    if(!result.geometry.valid){result.pose.reason=result.geometry.reason;return result;}
    std::vector<cv::Point3d> world;
    for(const auto& p:result.geometry.association.hypotheses[0].nodes)
      world.emplace_back(p[1]*calibration.cell_size_m,p[0]*calibration.cell_size_m,0);
    result.pose=metric_landmark_pose(world,result.geometry.pixels,calibration,image.size(),sensor.depth,sensor.stamp,sequence);
    if(!result.pose.valid)return result;
    auto& observation=result.observation;
    observation.epoch=epoch;observation.sequence=sequence;observation.camera=camera;observation.stamp=sensor.stamp;
    observation.grid_frame="yellow_bottom_pipe_centres";observation.association_unique=true;
    const auto found=cones_.process(result.geometry.rectified);
    result.colour_cones=found.colour_observations.size();result.shape_cones=found.shape_observations.size();
    result.cone_reason=found.decision_reason;
    cv::Mat hsv,red1,red2,red;
    cv::cvtColor(result.geometry.rectified,hsv,cv::COLOR_BGR2HSV);
    cv::inRange(hsv,cv::Scalar(0,80,50),cv::Scalar(20,255,255),red1);
    cv::inRange(hsv,cv::Scalar(165,80,50),cv::Scalar(179,255,255),red2);cv::bitwise_or(red1,red2,red);
    for(int row=0;row<3;++row)for(int col=0;col<3;++col) {
      LocalCellEvidence cell;cell.row=row;cell.col=col;
      const cv::Rect roi(col*200+12,row*200+12,176,176);
      // Any unseen cell interior remains unknown, including cropped cones.
      cell.full_cell_visible=cv::countNonZero(result.geometry.visible(cv::Rect(col*200,row*200,200,200)))==40000;
      if(!cell.full_cell_visible||!found.classifier_ready){observation.cells.push_back(cell);continue;}
      const auto cone=std::find_if(found.observations.begin(),found.observations.end(),
        [&](const auto& c){return c.row==row&&c.col==col;});
      if(cone!=found.observations.end()&&cone->confidence>=.8&&
          (cone->shape==auv_vision::ConeShape::kCircle||cone->shape==auv_vision::ConeShape::kSquare)) {
        // Split striped components are insufficient shape evidence. Keep
        // unknown until a single measured silhouette supports the class.
        cv::Mat labels,stats,centroids;const int parts=cv::connectedComponentsWithStats(red(roi),labels,stats,centroids);
        int significant=0;for(int i=1;i<parts;++i)if(stats.at<int>(i,cv::CC_STAT_AREA)>roi.area()*.01)++significant;
        if(significant==1||cone_mode_!="colour") {
          cell.type=cone->shape==auv_vision::ConeShape::kCircle?LocalCellClass::circle:LocalCellClass::square;
          cell.confidence=cone->confidence;cell.contour_frame="yellow_bottom_map_pixels_600";
          for(auto p:cone->contour)cell.contour.push_back({double(p.x),double(p.y)});
        }
      } else if(floor_.verified&&cv::countNonZero(red(roi))<roi.area()*.005&&
          (cone_mode_=="colour"||(!found.shape_background.empty()&&
            cv::countNonZero(found.shape_background(roi))>=roi.area()*.995))) {
        cv::Mat floor_mask;cv::inRange(hsv(roi),floor_.lower,floor_.upper,floor_mask);
        if(cv::countNonZero(floor_mask)<roi.area()*floor_.minimum_support) {
          observation.cells.push_back(cell);continue;
        }
        cell.type=LocalCellClass::empty;cell.confidence=.9;
      }
      observation.cells.push_back(cell);
    }
    return result;
  }
 private:
  LocalGridDetector grid_;auv_vision::ConeDetector cones_;LocalFloorModel floor_;std::string cone_mode_;
};
}
