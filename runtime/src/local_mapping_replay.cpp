#include "runtime_config.hpp"
#include "auv_mapping/local_mapping_pipeline.hpp"
#include "auv_mapping/local_pose_fusion.hpp"
#include "auv_vision/apriltag_detector.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <map>
#include <iostream>

using namespace auv_mapping;

// Native, uncut camera files with explicit per-frame telemetry. Never opens
// UART, requests ARM or invents telemetry from frame rate / commanded motion.
int main(int argc,char** argv) {
  if(argc!=5){std::cerr<<"Usage: auv_local_mapping_replay runtime.yaml video frame-telemetry.yaml output_dir\n";return 2;}
  try {
    const auto cfg=load_config(argv[1]);
    if(cfg.motion_enabled||cfg.auto_start||cfg.auto_arm)throw std::runtime_error("offline config must disable motion and automatic startup");
    const auto input=YAML::LoadFile(argv[3]);
    const std::string camera=input["camera"].as<std::string>();
    const std::string space=input["image_space"].as<std::string>();
    if((camera!="front"&&camera!="down")||(space!="raw"&&space!="rectified"))
      throw std::runtime_error("explicit camera and native image space required; display crops unsupported");
    std::map<int,YAML::Node> frames;
    for(auto item:input["frames"]) {
      const int index=item["frame"].as<int>();
      if(index<0||!frames.emplace(index,item).second)throw std::runtime_error("invalid or duplicate telemetry frame index");
    }
    cv::VideoCapture video(argv[2]);if(!video.isOpened())throw std::runtime_error("video open failed");
    std::filesystem::create_directories(argv[4]);
    const std::filesystem::path folder(argv[4]);
    std::ofstream report(folder/"frames.csv");report.precision(15);report<<"frame,stamp,phase,geometry,pose,nodes,hypotheses,tag18,map_complete,reason\n";
    LocalMappingPipeline pipeline(cfg.cone,cfg.grid,cfg.local_floor);
    LocalMapAccumulator map;std::uint64_t epoch=1;map.reset(epoch,"yellow_bottom_pipe_centres");
    auv_vision::AprilTagDetector tags(cfg.apriltag_family);
    std::unique_ptr<CameraRectifier> rectifier;
    cv::Mat image;int index=0;std::string phase;double last_stamp=-1;
    MetricGridPose prior;
    while(video.read(image)) {
      const int frame=index++;auto sample=frames.find(frame);if(sample==frames.end())continue;
      const auto item=sample->second;
      const double stamp=item["stamp"].as<double>(),telemetry=item["telemetry_stamp"].as<double>();
      const auto next_phase=item["phase"].as<std::string>();
      if(!std::isfinite(stamp)||stamp<0||stamp<=last_stamp||!std::isfinite(telemetry)||
          std::abs(stamp-telemetry)>.15||!item["valid"].as<bool>()||
          (next_phase!="underwater"&&next_phase!="surface"))
        throw std::runtime_error("invalid, reordered or unsynchronized frame telemetry");
      last_stamp=stamp;
      const auto& c=camera=="front"?(next_phase=="surface"?cfg.traversal.front_surface:cfg.traversal.front_underwater):
        (next_phase=="surface"?cfg.traversal.surface:cfg.traversal.underwater);
      if(next_phase!=phase) {
        if(!phase.empty()&&!map.begin_stage(++epoch,stamp,"yellow_bottom_pipe_centres"))
          throw std::runtime_error("map phase transition rejected");
        phase=next_phase;rectifier.reset();prior=MetricGridPose();
      }
      if(cfg.localization.imu_signs.size()!=3||cfg.localization.imu_offsets.size()!=3)
        throw std::runtime_error("recorded IMU conversion required; defaults cannot replace evidence");
      PlaneSample sensor;sensor.stamp=stamp;sensor.depth=item["depth_m"].as<double>();
      sensor.roll=(cfg.localization.imu_signs[0]*item["roll_deg"].as<double>()+cfg.localization.imu_offsets[0])*CV_PI/180;
      sensor.pitch=(cfg.localization.imu_signs[1]*item["pitch_deg"].as<double>()+cfg.localization.imu_offsets[1])*CV_PI/180;
      sensor.yaw=(cfg.localization.imu_signs[2]*item["yaw_deg"].as<double>()+cfg.localization.imu_offsets[2])*CV_PI/180;
      cv::Mat corrected=image;
      if(c.verified) {
        if(!rectifier)rectifier=std::make_unique<CameraRectifier>(true,c.width,c.height,c.camera_matrix,c.distortion);
        if(space=="raw")corrected=rectifier->apply(image);
      }
      const auto result=pipeline.process(corrected,c,sensor,std::uint64_t(frame)+1,epoch,camera=="front"?1:0,
        rectifier?rectifier->valid_mask():cv::Mat(),prior);
      // Use only previous measured landmark poses. The pipeline enforces the
      // 0.3 s age/jump gates; never carry an anchor across an imaging phase.
      if(result.pose.valid)prior=result.pose;
      std::string reason=result.pose.reason;
      if(result.pose.valid&&!map.update(result.observation,stamp,reason))reason="map reject: "+reason;
      bool tag18=false;for(const auto& tag:tags.detect(corrected))tag18|=tag.id==18;
      report<<frame<<','<<stamp<<','<<phase<<','<<result.geometry.valid<<','<<result.pose.valid<<','
        <<result.geometry.nodes.size()<<','<<result.geometry.association.hypotheses.size()<<','<<tag18<<','<<map.complete()<<",\""<<reason<<"\"\n";
      if(!result.geometry.debug.empty()&&!cv::imwrite((folder/(std::to_string(frame)+"-nodes.jpg")).string(),result.geometry.debug))
        throw std::runtime_error("node evidence write failed");
      if(result.geometry.valid) {
        YAML::Emitter evidence;evidence<<YAML::BeginMap<<YAML::Key<<"frame"<<YAML::Value<<frame
          <<YAML::Key<<"stamp"<<YAML::Value<<stamp<<YAML::Key<<"epoch"<<YAML::Value<<epoch
          <<YAML::Key<<"cells"<<YAML::Value<<YAML::BeginSeq;
        for(const auto& cell:result.observation.cells) {
          evidence<<YAML::BeginMap<<YAML::Key<<"row"<<YAML::Value<<cell.row<<YAML::Key<<"col"<<YAML::Value<<cell.col
            <<YAML::Key<<"type"<<YAML::Value<<int(cell.type)<<YAML::Key<<"full_cell_visible"<<YAML::Value<<cell.full_cell_visible
            <<YAML::Key<<"contour_frame"<<YAML::Value<<cell.contour_frame<<YAML::Key<<"contour"<<YAML::Value<<YAML::BeginSeq;
          for(auto p:cell.contour)evidence<<YAML::Flow<<YAML::BeginSeq<<p[0]<<p[1]<<YAML::EndSeq;
          evidence<<YAML::EndSeq<<YAML::EndMap;
        }
        evidence<<YAML::EndSeq<<YAML::EndMap;
        std::ofstream out(folder/(std::to_string(frame)+"-evidence.yaml"));out<<evidence.c_str();if(!out)throw std::runtime_error("cell evidence write failed");
      }
    }
    if(!report||frames.empty()||index==0||last_stamp<0)throw std::runtime_error("empty replay or evidence write failure");
    for(const auto& p:frames)if(p.first>=index)throw std::runtime_error("telemetry refers to a frame outside video");
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
