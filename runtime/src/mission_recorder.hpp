#pragma once
#include <filesystem>
#include <fstream>
#include <cstdint>
#include <string>
#include <stdexcept>
#include <cmath>
#include <vector>
#include <utility>

// Used only by the recording worker. MJPEG preserves received frames without
// encoding them again; the JSONL index supplies variable-rate capture timing.
class MissionRecorder {
 public:
  MissionRecorder(std::string directory, double segment_sec, std::uintmax_t reserve)
    :directory_(std::move(directory)),segment_sec_(segment_sec),reserve_(reserve){
    if(!std::isfinite(segment_sec_)||segment_sec_<=0||reserve_==0)
      throw std::invalid_argument("invalid recorder limits");
    std::filesystem::create_directories(directory_);
    require_space();
  }
  void append(const std::string& role,const std::vector<std::uint8_t>& jpeg,
              std::uint64_t sequence,double stamp,const std::string& source_json,
              const std::string& telemetry_json) {
    if(role!="down" && role!="front")throw std::invalid_argument("unknown camera role");
    auto& s=role=="down"?down_:front_;
    if(jpeg.empty()||sequence<=s.sequence||!std::isfinite(stamp)||stamp<=s.stamp)return;
    if(jpeg.size()<4||jpeg[0]!=0xff||jpeg[1]!=0xd8||jpeg[jpeg.size()-2]!=0xff||jpeg.back()!=0xd9)
      throw std::runtime_error("recorder rejected incomplete JPEG");
    if(!s.video.is_open()||stamp-s.started>=segment_sec_){
      require_space();s.video.close();s.index.close();
      s.filename=role+"-"+std::to_string(++s.segment)+".mjpg";
      s.video.open(directory_/s.filename,std::ios::binary|std::ios::trunc);
      s.index.open(directory_/(role+"-"+std::to_string(s.segment)+".frames.jsonl"),std::ios::trunc);
      s.started=stamp;
      if(!s.video||!s.index)throw std::runtime_error("cannot open onboard recording segment");
    }
    if(stamp-last_space_check_>=1){require_space();last_space_check_=stamp;}
    const auto offset=s.video.tellp();
    s.video.write(reinterpret_cast<const char*>(jpeg.data()),static_cast<std::streamsize>(jpeg.size()));
    s.video.flush();
    if(!s.video)throw std::runtime_error("onboard video write failed");
    s.index.precision(15);
    s.index<<"{\"sequence\":"<<sequence<<",\"steady_sec\":"<<stamp
      <<",\"source\":"<<source_json<<",\"offset\":"<<offset<<",\"bytes\":"<<jpeg.size()
      <<",\"dropped_since_previous\":"<<(s.sequence?sequence-s.sequence-1:0)
      <<",\"telemetry\":"<<telemetry_json<<"}\n";
    s.index.flush();
    if(!s.index)throw std::runtime_error("onboard frame index write failed");
    s.sequence=sequence;s.stamp=stamp;
  }
 private:
  void require_space(){if(std::filesystem::space(directory_).available<reserve_)
    throw std::runtime_error("onboard recorder free-space reserve reached");}
  struct Stream {
    std::ofstream video,index;std::uint64_t sequence{0},segment{0};
    double stamp{0},started{0};std::string filename;
  };
  std::filesystem::path directory_;double segment_sec_,last_space_check_{0};
  std::uintmax_t reserve_;Stream down_,front_;
};
