#include "runtime_config.hpp"
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>

// Offline only: indexed JPEG decoding and absolute grid localization. This
// executable has no serial port, sockets, ARM commands or mission actuation.
int main(int argc,char** argv) {
  try {
    if(argc!=5){std::cerr<<"usage: auv_surface_replay CONFIG SEGMENT.mjpg SEGMENT.frames.jsonl OUTPUT.jsonl\n";return 2;}
    auto cfg=load_config(argv[1]);const auto& c=cfg.traversal.surface;
    if(!c.verified)throw std::runtime_error("independent surface calibration required");c.validate();
    for(int i:{1,2,3})if(std::filesystem::weakly_canonical(argv[i])==std::filesystem::weakly_canonical(argv[4]))
      throw std::runtime_error("output must not overwrite replay input");
    if(std::filesystem::exists(argv[4]))throw std::runtime_error("output already exists");
    std::ifstream video(argv[2],std::ios::binary),index(argv[3]);std::ofstream out(argv[4]);
    if(!video||!index||!out)throw std::runtime_error("cannot open replay paths");
    auv_mapping::GridMapper mapper(cfg.grid);
    cv::Mat intrinsics(3,3,CV_64F,const_cast<double*>(c.camera_matrix.data()));
    cv::Mat distortion(c.distortion);std::string line;std::size_t total=0,valid=0;
    std::uint64_t previous_sequence=0;double previous_stamp=-1;
    while(std::getline(index,line)) {
      const auto item=YAML::Load(line);const auto seq=item["sequence"].as<std::uint64_t>();
      const double stamp=item["steady_sec"].as<double>();const auto offset=item["offset"].as<std::uint64_t>();
      const auto size=item["bytes"].as<std::size_t>();
      if(!std::isfinite(stamp)||seq<=previous_sequence||stamp<=previous_stamp||size<4||size>16*1024*1024)
        throw std::runtime_error("invalid frame index sequence, time or size");
      previous_sequence=seq;previous_stamp=stamp;
      video.seekg(static_cast<std::streamoff>(offset));std::vector<unsigned char> bytes(size);
      video.read(reinterpret_cast<char*>(bytes.data()),static_cast<std::streamsize>(size));
      if(!video)throw std::runtime_error("truncated indexed JPEG");
      cv::Mat image=cv::imdecode(bytes,cv::IMREAD_COLOR),corrected;
      if(image.empty())throw std::runtime_error("JPEG decode failed");
      cv::undistort(image,corrected,intrinsics,distortion);
      const auto telemetry=item["telemetry"];double depth=-1;
      const std::string phase=telemetry["phase"].as<std::string>("");
      if(telemetry["valid"].as<bool>(false)&&
        (phase=="RELOCALIZE_SURFACE"||phase=="PLAN_CONES"||phase=="VISIT_CONES")&&
        std::abs(stamp-telemetry["received_sec"].as<double>(-100))<=cfg.status_timeout)
        depth=telemetry["depth_m"].as<double>(-1);
      const auto pose=auv_mapping::metric_grid_pose(mapper.process(corrected),c,image.size(),depth,stamp,seq);
      ++total;if(pose.valid)++valid;
      out.precision(15);out<<"{\"sequence\":"<<seq<<",\"steady_sec\":"<<stamp<<",\"valid\":"<<(pose.valid?"true":"false")
        <<",\"row\":"<<(std::isfinite(pose.row)?std::to_string(pose.row):"null")
        <<",\"col\":"<<(std::isfinite(pose.col)?std::to_string(pose.col):"null")
        <<",\"reprojection_px\":"<<(std::isfinite(pose.reprojection_px)?std::to_string(pose.reprojection_px):"null")
        <<",\"reason\":\""<<pose.reason<<"\"}\n";
      if(!out)throw std::runtime_error("replay output write failed");
    }
    out.flush();if(!out)throw std::runtime_error("replay flush failed");
    std::cout<<"frames="<<total<<" valid_absolute_surface_poses="<<valid<<"; localization analysis only\n";
    return total&&valid?0:1;
  }catch(const std::exception& e){std::cerr<<"auv_surface_replay: "<<e.what()<<'\n';return 1;}
}
