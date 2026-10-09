#include "csi_capture.hpp"
#include "camera_rectifier.hpp"
#include "localization.hpp"
#include "mission_recorder.hpp"
#include "runtime_config.hpp"
#include "auv_core/semantic_map.hpp"
#include "auv_core/status_decoder.hpp"
#include "auv_control/route_executor.hpp"
#include "auv_mapping/grid_mapper.hpp"
#include "auv_mission/mission_fsm.hpp"
#include "auv_planning/grid_planner.hpp"
#include "auv_stm32_bridge/motion_target.hpp"
#include "auv_stm32_bridge/depth_sample.hpp"
#include "auv_stm32_bridge/gripper_protocol.hpp"
#include "auv_stm32_bridge/protocol.h"
#include "auv_stm32_bridge/serial_port.hpp"
#include "auv_stm32_bridge/stream_parser.hpp"
#include "auv_vision/apriltag_detector.hpp"
#include "auv_vision/camera_source.hpp"
#include "auv_vision/cone_detector.hpp"
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <algorithm>
#include <condition_variable>
#include <deque>
#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <memory>
#include <sstream>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/wait.h>
#ifdef AUV_HAVE_HTTPLIB
#include <httplib.h>
#endif

using Clock = std::chrono::steady_clock;
static double seconds() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
static std::atomic<bool> running{true};
static void stop_signal(int) { running = false; }
static std::string json_escape(const std::string& s) {
  std::string out;
  for (char c : s) { if (c == '"' || c == '\\') out += '\\'; if (c >= 32) out += c; }
  return out;
}
static std::string json_number(double value) {return std::isfinite(value)?std::to_string(value):"null";}
static bool surface_grid_phase(auv_mission::MissionPhase phase) {
  return phase==auv_mission::MissionPhase::kRelocalizeSurface ||
      phase==auv_mission::MissionPhase::kPlanCones || phase==auv_mission::MissionPhase::kVisitCones;
}
static bool propulsion_phase(auv_mission::MissionPhase phase,bool surface_traversal=false) {
  if(surface_traversal && (phase==auv_mission::MissionPhase::kSurfaceForCones ||
    phase==auv_mission::MissionPhase::kRelocalizeSurface || phase==auv_mission::MissionPhase::kPlanCones))return true;
  switch (phase) {
    case auv_mission::MissionPhase::kSearchAprilTag:
    case auv_mission::MissionPhase::kBuildMap:
    case auv_mission::MissionPhase::kVisitCones:
    case auv_mission::MissionPhase::kSearchCucumber:
    case auv_mission::MissionPhase::kAlignCucumber:
    case auv_mission::MissionPhase::kGrab:
    case auv_mission::MissionPhase::kTransport:
    case auv_mission::MissionPhase::kRelease:
    case auv_mission::MissionPhase::kSearchValve:
    case auv_mission::MissionPhase::kAlignValve:
    case auv_mission::MissionPhase::kRotateValve:
    case auv_mission::MissionPhase::kReturnHome:
    case auv_mission::MissionPhase::kSurface:
      return true;
    default:
      return false;
  }
}
class Runtime {
 public:
  explicit Runtime(Config cfg) : cfg_(std::move(cfg)), mission_(cfg_.mission),
    route_(cfg_.route), planner_(cfg_.planner), search_(cfg_.search),
    ascent_(cfg_.traversal),surface_route_(cfg_.traversal),center_approach_(cfg_.traversal),docking_(cfg_.docking) {
    (void)auv_vision::AprilTagDetector(cfg_.apriltag_family);
    (void)auv_mapping::GridMapper(cfg_.grid);
    (void)auv_vision::ConeDetector(cfg_.cone);
    (void)auv_vision::ConeTracker(cfg_.tracker);
    localization_=std::make_unique<Localization>(cfg_.localization);
    run_id_=std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count())+"-"+
      std::to_string(Clock::now().time_since_epoch().count());
    if(cfg_.recording_enabled) {
      run_dir_=cfg_.recording_dir+"/"+run_id_;
      if(!std::filesystem::create_directories(run_dir_))throw std::runtime_error("recording run directory already exists");
      recorder_=std::make_unique<MissionRecorder>(run_dir_,cfg_.recording_segment,cfg_.recording_reserve);
      cfg_.log=run_dir_+"/events.ndjson";cfg_.debug_dir=run_dir_+"/evidence";
      const auto source=YAML::LoadFile(cfg_.config_source);
      std::ofstream config_copy(run_dir_+"/config.yaml");config_copy<<source;
      YAML::Node manifest;
      manifest["run_id"]=run_id_;manifest["mode"]=cfg_.operation_mode;
      manifest["profile"]=cfg_.mission_profile;manifest["time_basis"]="Pi receive steady seconds; not exposure timestamps";
      manifest["video_format"]="received JPEG frames in MJPEG segments with JSONL offsets/timestamps";
      manifest["release"]=source["release"] ? source["release"] : YAML::Node("unverified");
      manifest["localization"]=source["localization"];
      std::ofstream out(run_dir_+"/manifest.yaml");out<<manifest;
      if(!out||!config_copy)throw std::runtime_error("cannot save run provenance");
    }
    std::filesystem::create_directories(std::filesystem::path(cfg_.log).parent_path());
    log_.open(cfg_.log, std::ios::app);
    if (!log_) throw std::runtime_error("cannot open event log");
  }
  void run() {
    std::signal(SIGINT, stop_signal); std::signal(SIGTERM, stop_signal); std::signal(SIGPIPE,SIG_IGN);
    event("BOOT", "mode=" + cfg_.operation_mode + " DISARM");
    std::thread logger(&Runtime::logger_loop, this);
    std::thread capture(&Runtime::capture_loop, this);
    std::thread front(&Runtime::front_loop, this);
    std::thread vision(&Runtime::vision_loop, this);
    std::thread localization(&Runtime::localization_loop,this);
    std::thread recorder(&Runtime::recording_loop,this);
    std::thread serial(&Runtime::serial_loop, this);
    std::thread control([this] {
      try { control_loop(); }
      catch (const std::exception& e) { fault(std::string("control: ")+e.what()); running=false; }
    });
    std::thread video([this] {
      try { video_loop(); }
      catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        video_detail_=e.what();
      }
    });
#ifdef AUV_HAVE_HTTPLIB
    std::thread web([this] {
      try { web_loop(); }
      catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        web_detail_=e.what();
      }
      web_finished_=true;
    });
#else
    { std::lock_guard<std::mutex> lock(state_mutex_); web_detail_ = "cpp-httplib unavailable"; }
#endif
    try { socket_loop(); }
    catch (const std::exception& e) { fault(std::string("control socket: ")+e.what()); }
    running = false;
#ifdef AUV_HAVE_HTTPLIB
    while (!web_finished_ && !web_server_) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (auto* server=web_server_.load()) server->stop();
    web.join();
#endif
    capture.join(); front.join(); vision.join(); localization.join(); control.join(); serial.join(); video.join(); recorder.join();
    event("SHUTDOWN", "DISARM requested");
    log_stop_=true;
    log_cv_.notify_all(); logger.join();
  }
 private:
  void event(const std::string& name, const std::string& detail) {
    try {
    std::ostringstream line;
    const auto unix_ms=std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    line << "{\"run_id\":\""<<run_id_<<"\",\"timestamp_unix_ms\":" << unix_ms << ",\"steady_sec\":" << seconds()
         << ",\"event\":\"" << json_escape(name)
         << "\",\"detail\":\"" << json_escape(detail) << "\"}\n";
    { std::lock_guard<std::mutex> lock(log_mutex_);
      if (log_queue_.size() >= 256) { log_queue_.pop_front(); log_degraded_=true; }
      log_queue_.push_back(line.str()); }
    log_cv_.notify_one();
    } catch (...) { log_degraded_=true; }
  }
  void logger_loop() {
    while (true) {
      std::string line;
      { std::unique_lock<std::mutex> lock(log_mutex_);
        log_cv_.wait_for(lock,std::chrono::milliseconds(100),[this]{return log_stop_ || !log_queue_.empty();});
        if (log_queue_.empty() && log_stop_) break;
        if (!log_queue_.empty()) { line=std::move(log_queue_.front()); log_queue_.pop_front(); } }
      if (line.empty()) continue;
      try {
        if (log_.tellp() >= static_cast<std::streampos>(cfg_.max_event_bytes) ||
            seconds()-log_started_ > 86400) {
          log_.close();
          std::filesystem::rename(cfg_.log,cfg_.log+"."+std::to_string(++log_segment_));
          log_.open(cfg_.log,std::ios::trunc);
          log_started_=seconds();
        }
        log_ << line;
        log_.flush();
        if (!log_) log_degraded_=true;
      } catch (...) { log_degraded_=true; }
    }
  }
  void fault(const std::string& why) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    set_fault_locked(why);
    armed_requested_ = false;
    disarm_pending_ = true;
    motion_ = {};
  }
  void set_fault_locked(const std::string& why) {
    if (fault_.empty()) { fault_ = why; event("FAULT",why); }
  }
  bool status_fresh(double now) const { return status_time_ > 0 && now - status_time_ <= cfg_.status_timeout; }
  bool safe_status(double now) const {
    return serial_connected_ && status_fresh(now) &&
      status_.error_flags == 0 && status_.telemetry_valid &&
      (!status_.dual_mode || status_.autonomous_mode);
  }
  bool gripper_status_fresh(double now) const {
    return gripper_time_ > 0 && now-gripper_time_ <= cfg_.actuator_status_timeout;
  }
  bool recording_ready(double now) const {
    return !cfg_.recording_required || (recording_detail_.empty() && !log_degraded_ &&
      recording_down_time_>0 && now-recording_down_time_.load()<=cfg_.frame_timeout+1 &&
      (!cfg_.front_enabled || (recording_front_time_>0 && now-recording_front_time_.load()<=cfg_.frame_timeout+1)));
  }
  bool arm_gate_ready(double now) const {
    if(cfg_.mission_profile=="tag_docking")return docking_.phase==TagDockTask::Phase::WaitArm&&
      cfg_.motion_enabled&&cfg_.directions_calibrated&&cfg_.limits_calibrated&&!cfg_.serial.empty()&&
      fault_.empty()&&safe_status(now)&&!status_.armed&&dock_ready(now);
    const auto phase=mission_.snapshot().phase;
    const bool observation_ready=cfg_.search.enabled &&
      (phase==auv_mission::MissionPhase::kSearchAprilTag || phase==auv_mission::MissionPhase::kBuildMap) &&
      localization_->snapshot(now).valid;
    const bool route_ready=!cfg_.mission.surface_before_visit && pose_valid_ && now-pose_time_<=cfg_.pose_timeout && map_.complete && route_ready_ &&
      phase==auv_mission::MissionPhase::kVisitCones;
    return cfg_.motion_enabled && cfg_.directions_calibrated && cfg_.limits_calibrated &&
      !cfg_.serial.empty() && fault_.empty() && safe_status(now) && !status_.armed &&
      recording_ready(now) && frame_time_>0 && now-frame_time_<=cfg_.frame_timeout &&
      (!cfg_.traversal.enabled||depth_sample_fresh(now)) && (observation_ready||route_ready);
  }
  bool depth_sample_fresh(double now) const {
    return depth_sample_.valid && depth_sample_received_>0 && now>=depth_sample_received_ &&
      now-depth_sample_received_+depth_sample_.age_sec<=(cfg_.mission_profile=="tag_docking" ?
        cfg_.docking.frame_timeout : cfg_.traversal.depth_sample_timeout_sec);
  }
  bool surface_grid_fresh(double now) const {
    return metric_pose_.valid && metric_pose_.surface_frame && metric_pose_.stamp>surface_reacquire_start_ && now>=metric_pose_.stamp &&
      now-metric_pose_.stamp<=cfg_.traversal.pose_timeout_sec;
  }
  void queue_trajectory_locked(double now,const std::string& source) {
    if(!cfg_.mission.surface_before_visit)return;
    if(trajectory_queue_.size()>=256){set_fault_locked("trajectory recording queue overflow");return;}
    const auto local=localization_->snapshot(now);
    std::ostringstream out;out.precision(15);
    out<<"{\"run_id\":\""<<run_id_<<"\",\"control_steady_sec\":"<<now
      <<",\"phase\":\""<<auv_mission::mission_phase_name(mission_.snapshot().phase)<<"\",\"source\":\""<<source
      <<"\",\"frame_sequence\":"<<metric_pose_.sequence<<",\"frame_steady_sec\":"<<metric_pose_.stamp
      <<",\"grid_valid\":"<<(metric_pose_.valid?"true":"false")<<",\"grid_row\":"<<json_number(metric_pose_.row)
      <<",\"grid_col\":"<<json_number(metric_pose_.col)<<",\"reprojection_px\":"<<json_number(metric_pose_.reprojection_px)
      <<",\"grid_reason\":\""<<json_escape(metric_pose_.reason)<<"\",\"underwater_valid\":"<<(local.valid?"true":"false")
      <<",\"underwater_session\":"<<local.session<<",\"underwater_steady_sec\":"<<local.stamp
      <<",\"underwater_x_m\":"<<json_number(local.x)<<",\"underwater_y_m\":"<<json_number(local.y)
      <<",\"depth_m\":"<<json_number(status_.depth)<<",\"depth_sensor_sequence\":"<<depth_sample_.sensor_sequence
      <<",\"depth_sample_age_sec\":"<<depth_sample_.age_sec+std::max(0.0,now-depth_sample_received_)
      <<",\"yaw_rad\":"<<json_number(status_.yaw)<<",\"waypoint\":"<<waypoint_index_<<"}\n";
    trajectory_queue_.push_back(out.str());
  }
  void request_arm_locked(const std::string& source) {
    hold_depth_=cfg_.search.enabled ? static_cast<float>(cfg_.search.depth_m) : status_.depth;
    hold_yaw_=cfg_.search.enabled ? static_cast<float>(std::remainder(origin_absolute_yaw_+cfg_.search.yaw_rad,2*CV_PI)) : status_.yaw;
    armed_requested_ = true; arm_ack_ = false; arm_pending_ = true; arm_time_=0;
    event("ARM_REQUEST", source);
  }
  bool claim_autonomous_run_locked() {
    const auto latch = cfg_.socket + ".autonomous-started";
    const int fd = ::open(latch.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0640);
    if (fd >= 0) {
      const auto stamp = std::to_string(static_cast<std::uint64_t>(seconds()*1000));
      const auto ignored = ::write(fd,stamp.data(),stamp.size());
      (void)ignored;
      ::close(fd);
      event("AUTONOMOUS_LATCH",latch);
      return true;
    }
    if (errno == EEXIST) set_fault_locked("autonomous mission already started since boot");
    else set_fault_locked("cannot create autonomous start latch");
    disarm_pending_ = true;
    return false;
  }
  void capture_loop() {
    const bool csi=cfg_.camera.rfind("csi:",0)==0;
    auto source_name = cfg_.camera.rfind("file:",0)==0 ? cfg_.camera.substr(5) : cfg_.camera;
    CsiCapture capture;
    auv_vision::CameraSource camera({source_name,cfg_.camera_width,cfg_.camera_height,
      cfg_.camera_fps,cfg_.camera_pixel_format,cfg_.camera.rfind("file:",0)==0});
    CameraRectifier underwater(!cfg_.camera_matrix.empty(),cfg_.camera_width,
        cfg_.camera_height,cfg_.camera_matrix,cfg_.distortion);
    CameraRectifier surface(cfg_.traversal.enabled,cfg_.traversal.surface.width,
        cfg_.traversal.surface.height,cfg_.traversal.surface.camera_matrix,cfg_.traversal.surface.distortion);
    double last_ready=seconds();
    bool received=false;
    while (running) {
      try {
        if(csi && !capture.is_open()) {
          capture.open(cfg_.camera.back()-'0',cfg_.camera_width,cfg_.camera_height,
                       static_cast<int>(cfg_.camera_fps));
          last_ready=seconds(); received=false;
        }
        if(!csi && !camera.is_open() && !camera.open()) {
          std::this_thread::sleep_for(std::chrono::seconds(1)); continue;
        }
        cv::Mat image;
        std::vector<std::uint8_t> jpeg;
        const bool ready=csi ? capture.read(image,&jpeg) : camera.read(image);
        if(!ready) {
          if(csi && seconds()-last_ready>(received ? 2.0 : 8.0))
            throw std::runtime_error("down CSI frame timeout");
          if(!csi)camera.close();
          std::this_thread::sleep_for(std::chrono::milliseconds(csi ? 10 : 200));
          continue;
        }
        last_ready=seconds(); received=true;
        const double previous=frame_time_.load();
        if (previous>0 && last_ready>previous)
          down_hz_=down_hz_<=0 ? 1.0/(last_ready-previous) : .9*down_hz_.load()+.1/(last_ready-previous);
        bool surface_frame=false;
        {std::lock_guard<std::mutex> lock(state_mutex_);
         surface_frame=cfg_.traversal.enabled && surface_grid_phase(mission_.snapshot().phase);}
        cv::Mat corrected=surface_frame?surface.apply(image):underwater.apply(image);
        cv::Mat display=cfg_.down_preview_rectify?corrected:image;
        std::vector<std::uint8_t> raw_jpeg;
        if(cfg_.recording_enabled && cfg_.down_preview_rectify) {
          if(csi)raw_jpeg=jpeg;
          else if(!cv::imencode(".jpg",image,raw_jpeg,{cv::IMWRITE_JPEG_QUALITY,70}))
            throw std::runtime_error("raw down JPEG encode failed");
        }
        if ((!csi || cfg_.down_preview_rectify) &&
            !cv::imencode(".jpg",display,jpeg,{cv::IMWRITE_JPEG_QUALITY,70}))
          throw std::runtime_error("down JPEG encode failed");
        { std::lock_guard<std::mutex> lock(frame_mutex_);
          if(cfg_.recording_enabled)down_raw_jpeg_=raw_jpeg.empty()?jpeg:std::move(raw_jpeg);
          down_jpeg_=std::move(jpeg);
          down_preview_frame_=display;
          down_rectified_frame_=corrected;down_surface_frame_=surface_frame;
          frame_=image; frame_time_=last_ready; ++frame_sequence_; ++down_capture_frames_;
        }
      } catch(const std::exception& e) {
        capture.close(); camera.close(); fault(std::string("down camera: ")+e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  void front_loop() {
    if (!cfg_.front_enabled) return;
    const bool csi=cfg_.front_source.rfind("csi:",0)==0;
    const auto source=csi ? cfg_.front_source :
      (cfg_.front_source.rfind("file:",0)==0 ? cfg_.front_source.substr(5) : cfg_.front_source);
    CsiCapture capture;
    double last_ready=seconds();
    bool received=false;
    auv_vision::CameraSource camera({source,cfg_.front_width,cfg_.front_height,
      static_cast<double>(cfg_.front_fps),"MJPG",cfg_.front_source.rfind("file:",0)==0});
    CameraRectifier preview(cfg_.front_preview_rectify,cfg_.front_calibration_width,
        cfg_.front_calibration_height,cfg_.front_camera_matrix,cfg_.front_distortion);
    // Front metric rectifiers feed grid mapping + absolute pose when
    // metric_camera_front. Independent of the preview rectifier so metric output
    // does not depend on front_preview_rectify. Water/air models share the front
    // capture resolution; K/D come from front_metric, distinct from the preview
    // front_camera_matrix (a future optimization could reuse the preview frame
    // only if the two calibrations are proven identical).
    const bool metric_front=cfg_.traversal.enabled && cfg_.traversal.metric_camera_front;
    CameraRectifier metric_underwater(metric_front,cfg_.traversal.front_underwater.width,
        cfg_.traversal.front_underwater.height,cfg_.traversal.front_underwater.camera_matrix,
        cfg_.traversal.front_underwater.distortion);
    CameraRectifier metric_surface(metric_front,cfg_.traversal.front_surface.width,
        cfg_.traversal.front_surface.height,cfg_.traversal.front_surface.camera_matrix,
        cfg_.traversal.front_surface.distortion);
    while (running) {
      try {
        if (csi && !capture.is_open()) {
          capture.open(source.back()-'0',cfg_.front_width,cfg_.front_height,cfg_.front_fps);
          last_ready=seconds();
          received=false;
        }
        if (!csi && !camera.is_open() && !camera.open()) throw std::runtime_error("front camera unavailable");
        cv::Mat image;
        std::vector<std::uint8_t> jpeg;
        const bool ready=csi ? capture.read(image,&jpeg) : camera.read(image);
        if (!ready) {
          if (csi && seconds()-last_ready>(received ? 2.0 : 8.0)) throw std::runtime_error("CSI frame timeout");
          std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue;
        }
        const double stamp=seconds(), previous=front_time_.load();
        last_ready=stamp;
        received=true;
        cv::Mat display=preview.apply(image);
        cv::Mat metric;
        bool metric_surface_frame=false;
        if(metric_front) {
          {std::lock_guard<std::mutex> lock(state_mutex_);
           metric_surface_frame=surface_grid_phase(mission_.snapshot().phase);}
          metric=metric_surface_frame?metric_surface.apply(image):metric_underwater.apply(image);
        }
        std::vector<std::uint8_t> raw_jpeg;
        if(cfg_.recording_enabled && cfg_.front_preview_rectify) {
          if(csi)raw_jpeg=jpeg;
          else if(!cv::imencode(".jpg",image,raw_jpeg,{cv::IMWRITE_JPEG_QUALITY,70}))
            throw std::runtime_error("raw front JPEG encode failed");
        }
        if ((!csi || cfg_.front_preview_rectify) &&
            !cv::imencode(".jpg",display,jpeg,{cv::IMWRITE_JPEG_QUALITY,70}))
          throw std::runtime_error("front JPEG encode failed");
        { std::lock_guard<std::mutex> lock(front_mutex_);
          if(cfg_.recording_enabled)front_raw_jpeg_=raw_jpeg.empty()?jpeg:std::move(raw_jpeg);
          front_preview_frame_=display;
          front_frame_=image; front_jpeg_=std::move(jpeg); front_time_=stamp; ++front_sequence_;
          front_rectified_frame_=metric; front_surface_frame_=metric_surface_frame; }
        { std::lock_guard<std::mutex> lock(state_mutex_);
          ++front_frames_; front_detail_.clear();
          if (previous>0 && stamp>previous) front_hz_=front_hz_<=0 ? 1.0/(stamp-previous) :
            0.9*front_hz_+0.1/(stamp-previous);
        }
        /* Live V4L2 read already waits for the next frame. Only pace file replay. */
        if (!csi && !camera.is_live())
          std::this_thread::sleep_for(std::chrono::milliseconds(1000/cfg_.front_fps));
      } catch (const std::exception& e) {
        capture.close(); camera.close();
        { std::lock_guard<std::mutex> lock(state_mutex_); front_detail_=e.what(); }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  void localization_loop() {
    std::uint64_t seen=0; double last=0;
    while(running){
      const double now=seconds();
      if(now-last < localization_->period()) {std::this_thread::sleep_for(std::chrono::milliseconds(10));continue;}
      last=now;cv::Mat image;double stamp=0;bool corrected=false,surface_frame=false;
      const bool matching=cfg_.camera_matrix==cfg_.localization.intrinsics &&
          cfg_.distortion==cfg_.localization.distortion && !cfg_.camera_matrix.empty();
      {std::lock_guard<std::mutex> lock(frame_mutex_);
       if(seen!=frame_sequence_){seen=frame_sequence_;image=matching?down_rectified_frame_:frame_;
        corrected=matching;surface_frame=down_surface_frame_;stamp=frame_time_;}}
      try {
        if(!image.empty()){
          bool surface_phase=false;
          {std::lock_guard<std::mutex> lock(state_mutex_);
           surface_phase=cfg_.traversal.enabled && surface_grid_phase(mission_.snapshot().phase);}
          if(surface_frame || surface_phase) {
            localization_->unavailable("surface_phase_use_grid_pose");continue;
          }
          YAML::Node sensor;
          if(!cfg_.serial.empty()){
            std::lock_guard<std::mutex> lock(state_mutex_);
            sensor["stamp"]=status_time_;sensor["valid"]=status_.telemetry_valid;
            sensor["armed"]=status_.armed;sensor["depth_m"]=status_.depth;
            const double degrees=180/std::acos(-1);
            sensor["roll_deg"]=status_.roll*degrees;sensor["pitch_deg"]=status_.pitch*degrees;sensor["yaw_deg"]=status_.yaw*degrees;
          }
          localization_->process(image,stamp,now,sensor,corrected);
          if(cfg_.auto_origin && !localization_->has_origin()) {
            bool permitted=false;double yaw=0;
            {std::lock_guard<std::mutex> lock(state_mutex_);
              permitted=mission_.snapshot().phase==auv_mission::MissionPhase::kInit && safe_status(now) &&
                !status_.armed && !armed_requested_ && fault_.empty(); yaw=status_.yaw;}
            if(permitted && localization_->reset(now)) {
              {std::lock_guard<std::mutex> lock(state_mutex_);origin_absolute_yaw_=yaw;}
              event("LOCALIZATION_ORIGIN",localization_->json(now));
            }
          }
        }
        else if(!frame_time_ || now-frame_time_>.3)localization_->unavailable("image_unavailable");
        else continue;
        if(localization_->enabled())event("LOCALIZATION",localization_->json(seconds()));
      }catch(const std::exception&){localization_->unavailable("processing_error");}
    }
  }
  void recording_loop() {
    if(!recorder_)return;
    std::ofstream trajectory;
    if(cfg_.mission.surface_before_visit)trajectory.open(run_dir_+"/trajectory.jsonl");
    while(running) {
      try {
        std::vector<std::uint8_t> down,front;std::uint64_t ds=0,fs=0;double dt=0,ft=0;
        {std::lock_guard<std::mutex> lock(frame_mutex_);down=down_raw_jpeg_;ds=frame_sequence_;dt=frame_time_;}
        if(cfg_.front_enabled){std::lock_guard<std::mutex> lock(front_mutex_);front=front_raw_jpeg_;fs=front_sequence_;ft=front_time_;}
        std::ostringstream telemetry;std::deque<std::string> points;std::string planned;
        {std::lock_guard<std::mutex> lock(state_mutex_);
          telemetry.precision(15);
          points.swap(trajectory_queue_);planned.swap(plan_artifact_pending_);
          telemetry<<"{\"received_sec\":"<<status_time_<<",\"valid\":"<<(safe_status(seconds())?"true":"false")
            <<",\"armed\":"<<(status_.armed?"true":"false")
            <<",\"depth_m\":"<<(std::isfinite(status_.depth)?std::to_string(status_.depth):"null")
            <<",\"roll_rad\":"<<(std::isfinite(status_.roll)?std::to_string(status_.roll):"null")
            <<",\"pitch_rad\":"<<(std::isfinite(status_.pitch)?std::to_string(status_.pitch):"null")
            <<",\"yaw_rad\":"<<(std::isfinite(status_.yaw)?std::to_string(status_.yaw):"null")
            <<",\"phase\":\""<<(cfg_.mission_profile=="tag_docking" ? docking_.name() : auv_mission::mission_phase_name(mission_.snapshot().phase))<<"\"}";
        }
        if(cfg_.mission.surface_before_visit) {
          for(const auto& point:points)trajectory<<point;
          trajectory.flush();if(!trajectory)throw std::runtime_error("measured trajectory write failed");
          if(!planned.empty()) {
            std::ofstream route(run_dir_+"/planned_route.json");route<<planned;route.flush();
            if(!route)throw std::runtime_error("planned route write failed");
            std::lock_guard<std::mutex> lock(state_mutex_);plan_evidence_saved_=true;
          }
        }
        if(ds && !down.empty()){recorder_->append("down",down,ds,dt,"\""+json_escape(cfg_.camera)+"\"",telemetry.str());recording_down_time_=dt;}
        if(fs && !front.empty()){recorder_->append("front",front,fs,ft,"\""+json_escape(cfg_.front_source)+"\"",telemetry.str());recording_front_time_=ft;}
      }catch(const std::exception& e){
        {std::lock_guard<std::mutex> lock(state_mutex_);recording_detail_=e.what();}
        event("RECORDING_FAILED",e.what());
        if(cfg_.recording_required)fault(std::string("onboard recording: ")+e.what());
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  bool dock_ready(double now) const {
    return safe_status(now)&&depth_sample_fresh(now)&&recording_ready(now)&&!log_degraded_&&
      localization_->snapshot(now).valid&&frame_time_>0&&now-frame_time_<=cfg_.frame_timeout&&
      front_time_>0&&now-front_time_<=cfg_.frame_timeout&&processed_time_>0&&now-processed_time_<=cfg_.frame_timeout&&
      std::isfinite(status_.roll)&&std::isfinite(status_.pitch)&&std::abs(status_.roll)<=cfg_.docking.max_tilt&&
      std::abs(status_.pitch)<=cfg_.docking.max_tilt;
  }
  bool dock_front_ready(double now) const {
    return dock_front_.detected&&dock_front_.metric_valid&&dock_front_.id==cfg_.docking.id&&
      dock_front_.sequence&&now>=dock_front_.stamp&&now-dock_front_.stamp<=cfg_.docking.frame_timeout;
  }
  void dock_control_locked(double now) {
    const auto pose=localization_->snapshot(now);
    if(cfg_.auto_start&&!autonomous_start_attempted_&&docking_.phase==TagDockTask::Phase::Idle) {
      const bool ready=fault_.empty()&&cfg_.motion_enabled&&!status_.armed&&dock_ready(now)&&
        dock_front_ready(now);
      if(ready){if(!startup_ready_since_)startup_ready_since_=now;}else startup_ready_since_=0;
      if(now-boot_time_>=cfg_.startup_delay&&startup_ready_since_&&now-startup_ready_since_>=cfg_.startup_stable) {
        autonomous_start_attempted_=true;
        if(claim_autonomous_run_locked()&&docking_.start(now,depth_sample_.depth_m,pose))event("TAG_DOCK_START","autonomous one-shot latch");
        else set_fault_locked("tag docking startup refused");
      }else if(now-boot_time_>cfg_.startup_timeout){autonomous_start_attempted_=true;set_fault_locked("tag docking startup timeout");}
    }
    if(cfg_.auto_arm&&!autonomous_arm_attempted_&&docking_.phase==TagDockTask::Phase::WaitArm) {
      autonomous_arm_attempted_=true;
      if(arm_gate_ready(now))request_arm_locked("autonomous tag docking");else set_fault_locked("tag docking ARM gate rejected");
    }
    if(!fault_.empty())docking_.fail(fault_);
    const auto output=docking_.step(now,depth_sample_.depth_m,depth_sample_.sensor_sequence,
      armed_requested_&&arm_ack_&&status_.armed,dock_ready(now),pose,dock_front_,dock_down_);
    motion_={};
    if(docking_.active()&&fault_.empty()) {
      motion_.depth=hold_depth_=static_cast<float>(output.depth);motion_.yaw=hold_yaw_;
      motion_.vx=static_cast<float>(output.surge);motion_.vy=static_cast<float>(output.sway);
    }
    if(output.report) {
      std::ostringstream report;report<<"target_m="<<docking_.report_target<<" mean_m="<<docking_.report_mean
        <<" min_m="<<docking_.report_min<<" max_m="<<docking_.report_max<<" samples="<<docking_.report_samples
        <<" error_m="<<docking_.report_mean-docking_.report_target<<" pool_depth_measured=false";
      event("DEPTH_TEST_REPORT",report.str());
    }
    if(docking_.phase==TagDockTask::Phase::Fault) {
      if(fault_.empty())set_fault_locked(docking_.reason);
      armed_requested_=false;arm_pending_=false;disarm_pending_=true;motion_={};
    }
    if(dock_last_phase_!=docking_.name()){
      if(docking_.phase==TagDockTask::Phase::Hold){
        dock_evidence_frame_=dock_candidate_frame_;dock_evidence_observation_=dock_down_;
      }
      dock_last_phase_=docking_.name();event("TAG_DOCK_PHASE",dock_last_phase_+": "+docking_.reason);
    }
    if(docking_.active()&&now-dock_last_report_time_>=.2) {
      dock_last_report_time_=now;
      std::ostringstream data;data<<"phase="<<docking_.name()<<" x_m="<<pose.x<<" y_m="<<pose.y<<" session="<<pose.session
        <<" depth_m="<<depth_sample_.depth_m<<" depth_target_m="<<output.depth<<" sensor_sequence="<<depth_sample_.sensor_sequence
        <<" goal_x_m="<<docking_.goal_x<<" goal_y_m="<<docking_.goal_y
        <<" down_error_px="<<dock_down_.pixel_error<<" front_frame="<<dock_front_.sequence<<" down_frame="<<dock_down_.sequence;
      event("TAG_DOCK_TRACE",data.str());
    }
  }
  void dock_vision_loop() {
    auv_vision::AprilTagDetector detector(cfg_.apriltag_family);
    std::uint64_t down_seen=0,front_seen=0;
    while(running) {
      try {
        cv::Mat down,front;double dt=0,ft=0;std::uint64_t ds=0,fs=0;
        {std::lock_guard<std::mutex> lock(frame_mutex_);if(down_seen!=frame_sequence_){
          down=down_rectified_frame_;ds=frame_sequence_;dt=frame_time_;down_seen=ds;}}
        {std::lock_guard<std::mutex> lock(front_mutex_);if(front_seen!=front_sequence_){
          front=front_preview_frame_;fs=front_sequence_;ft=front_time_;front_seen=fs;}}
        auto observe=[&](const cv::Mat& image,std::uint64_t sequence,double stamp,bool is_front) {
          DockObservation o;o.stamp=stamp;o.sequence=sequence;
          const auto tags=detector.detect(image);
          const auto tag=std::find_if(tags.begin(),tags.end(),[this](const auto& t){return t.id==cfg_.docking.id;});
          if(tag!=tags.end())o=dock_observation(*tag,sequence,stamp,image.size(),
            is_front?cfg_.front_camera_matrix:cfg_.camera_matrix,cfg_.camera_matrix,cfg_.docking,is_front);
          return o;
        };
        if(!front.empty()){const auto o=observe(front,fs,ft,true);std::lock_guard<std::mutex> lock(state_mutex_);dock_front_=o;}
        if(!down.empty()) {
          auto o=observe(down,ds,dt,false);bool evidence=false;
          {std::lock_guard<std::mutex> lock(state_mutex_);
            const double finished=seconds();
            if(processed_time_>0&&finished>processed_time_)vision_hz_=vision_hz_<=0 ?
              1/(finished-processed_time_) : .9*vision_hz_+.1/(finished-processed_time_);
            processed_time_=finished;++processed_frames_;
            latency_ms_[latency_index_++%latency_ms_.size()]=(finished-dt)*1000;
            latency_count_=std::min(latency_count_+1,latency_ms_.size());
            dock_down_=o;dock_candidate_frame_=down;
            evidence=docking_.phase==TagDockTask::Phase::Hold&&!dock_evidence_saved_&&!dock_evidence_frame_.empty();
            if(evidence){down=dock_evidence_frame_;o=dock_evidence_observation_;ds=o.sequence;dt=o.stamp;}}
          if(evidence) {
            std::filesystem::create_directories(cfg_.debug_dir);
            const auto path=cfg_.debug_dir+"/tag18-down-centered.jpg";
            if(!cv::imwrite(path,down))throw std::runtime_error("cannot save tag docking completion evidence");
            YAML::Node meta;meta["family"]=cfg_.apriltag_family;meta["id"]=cfg_.docking.id;
            meta["size_m"]=cfg_.docking.size_m;meta["image_space"]="rectified";
            meta["frame_sequence"]=ds;meta["steady_sec"]=dt;meta["center_error_px"]=o.pixel_error;
            meta["calibration_id"]=cfg_.down_calibration_id;
            std::ofstream file(cfg_.debug_dir+"/tag18-down-centered.yaml");file<<meta;file.flush();
            if(!file)throw std::runtime_error("cannot save centered tag provenance");
            {std::lock_guard<std::mutex> lock(state_mutex_);dock_evidence_saved_=true;}
            event("TAG_DOCK_HOVER","center evidence="+path+"; active depth/yaw/visual position hold; explicit abort to stop");
          }
        }
      }catch(const std::exception& e){fault(std::string("tag docking vision: ")+e.what());}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  void vision_loop() {
    if(cfg_.mission_profile=="tag_docking"){dock_vision_loop();return;}
    auv_vision::AprilTagDetector tags(cfg_.apriltag_family);
    auv_mapping::GridMapper mapper(cfg_.grid);
    auv_vision::ConeDetector detector(cfg_.cone);
    auv_vision::ConeTracker tracker(cfg_.tracker);
    CameraRectifier underwater(!cfg_.camera_matrix.empty(),cfg_.camera_width,
        cfg_.camera_height,cfg_.camera_matrix,cfg_.distortion);
    CameraRectifier surface(cfg_.traversal.enabled,cfg_.traversal.surface.width,
        cfg_.traversal.surface.height,cfg_.traversal.surface.camera_matrix,cfg_.traversal.surface.distortion);
    // Front metric rectifiers mirror the down pair; front_loop rectifies at
    // capture time, so these only run on surface/underwater phase transitions.
    const bool metric_front=cfg_.traversal.enabled && cfg_.traversal.metric_camera_front;
    CameraRectifier front_underwater(metric_front,cfg_.traversal.front_underwater.width,
        cfg_.traversal.front_underwater.height,cfg_.traversal.front_underwater.camera_matrix,
        cfg_.traversal.front_underwater.distortion);
    CameraRectifier front_surface(metric_front,cfg_.traversal.front_surface.width,
        cfg_.traversal.front_surface.height,cfg_.traversal.front_surface.camera_matrix,
        cfg_.traversal.front_surface.distortion);
    std::uint64_t seen = 0, front_seen = 0;
    std::uint64_t epoch=perception_epoch_;int tag_id=-1,tag_votes=0;bool previous_surface=false;
    while (running) {
      cv::Mat image,cached; double stamp = 0;bool cached_surface=false;
      { std::lock_guard<std::mutex> lock(frame_mutex_);
        if (seen != frame_sequence_) { seen = frame_sequence_; image = frame_;cached=down_rectified_frame_;
          cached_surface=down_surface_frame_;stamp = frame_time_; } }
      cv::Mat front_image,front_cached;double front_stamp=0;bool front_cached_surface=false;
      if(metric_front) {
        { std::lock_guard<std::mutex> lock(front_mutex_);
          if (front_seen != front_sequence_) { front_seen = front_sequence_; front_image = front_frame_;
            front_cached=front_rectified_frame_;front_cached_surface=front_surface_frame_;front_stamp=front_time_; } }
      }
      if (image.empty()) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }
      try {
      const auto current_epoch=perception_epoch_.load();
      if(epoch!=current_epoch){epoch=current_epoch;tag_id=-1;tag_votes=0;mapper.reset();tracker.reset();}
      auv_mission::MissionPhase phase;
      bool triggered=false;
      {std::lock_guard<std::mutex> lock(state_mutex_);phase=mission_.snapshot().phase;triggered=tag_found_;}
      const bool surface_phase=cfg_.traversal.enabled && surface_grid_phase(phase);
      if(surface_phase!=previous_surface){mapper.reset();previous_surface=surface_phase;}
      // Reuse capture correction, except when a mission transition changed the model.
      if(!cached.empty() && cached_surface==surface_phase)image=cached;
      else image=surface_phase?surface.apply(image):underwater.apply(image);
      // Grid mapping + metric pose use the front camera in front-metric mode;
      // the AprilTag trigger below always stays on the down camera.
      cv::Mat metric_image;double metric_stamp=stamp;std::uint64_t metric_seq=seen;
      if(metric_front) {
        if(front_image.empty()) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }
        if(!front_cached.empty() && front_cached_surface==surface_phase)metric_image=front_cached;
        else metric_image=surface_phase?front_surface.apply(front_image):front_underwater.apply(front_image);
        metric_stamp=front_stamp;metric_seq=front_seen;
      } else metric_image=image;
      const auto found = tags.detect(image);
      if(phase==auv_mission::MissionPhase::kSearchAprilTag && !triggered) {
        const auto eligible=std::find_if(found.begin(),found.end(),[this](const auto& tag){
          return cfg_.apriltag_ids.empty()||std::find(cfg_.apriltag_ids.begin(),cfg_.apriltag_ids.end(),tag.id)!=cfg_.apriltag_ids.end();});
        if(eligible==found.end()){tag_id=-1;tag_votes=0;}
        else {if(tag_id==eligible->id)++tag_votes;else{tag_id=eligible->id;tag_votes=1;}}
        if(tag_votes>=cfg_.tag_stable_frames) {
          std::filesystem::create_directories(cfg_.debug_dir);
          const auto filename=cfg_.debug_dir+"/apriltag_"+std::to_string(tag_id)+"_"+
            std::to_string(static_cast<std::uint64_t>(stamp*1000))+".jpg";
          if(!cv::imwrite(filename,image))throw std::runtime_error("cannot save AprilTag evidence");
          {std::lock_guard<std::mutex> lock(state_mutex_);
            if(epoch==perception_epoch_ && mission_.snapshot().phase==auv_mission::MissionPhase::kSearchAprilTag) {
              tag_found_=true;tag_time_=stamp;triggered=true;}}
          if(triggered)event("APRILTAG_FOUND","id="+std::to_string(tag_id)+" image="+filename);
          mapper.reset();tracker.reset();
        }
      }
      const auto grid = mapper.process(metric_image);
      auv_mapping::MetricGridPose metric;
      if(cfg_.traversal.enabled) {
        double depth=-1;
        {std::lock_guard<std::mutex> lock(state_mutex_);if(depth_sample_fresh(seconds()))depth=depth_sample_.depth_m;}
        metric=auv_mapping::metric_grid_pose(grid,
          metric_front ? (surface_phase?cfg_.traversal.front_surface:cfg_.traversal.front_underwater)
                       : (surface_phase?cfg_.traversal.surface:cfg_.traversal.underwater),
          metric_image.size(),depth,metric_stamp,metric_seq);
        metric.surface_frame=surface_phase;
      }
      std::vector<auv_vision::ConeObservation> cones;
      cv::Mat cone_debug;
      if (triggered && phase==auv_mission::MissionPhase::kBuildMap && grid.stable && !grid.rectified.empty()) {
        const auto detection=detector.process(grid.rectified);
        cones=tracker.update(detection.observations);cone_debug=detection.debug_image;
      } else if(phase==auv_mission::MissionPhase::kBuildMap) tracker.reset();
      if(surface_phase && phase==auv_mission::MissionPhase::kRelocalizeSurface && metric.valid) {
        const auto observed=detector.process(grid.rectified);
        std::lock_guard<std::mutex> lock(state_mutex_);
        unsigned matched=0;bool consistent=map_.complete && observed.observations.size()==4;
        for(const auto& cone:observed.observations) {
          const int id=cone.row*3+cone.col;
          const std::string shape=cone.shape==auv_vision::ConeShape::kCircle?"circle_cone":
            (cone.shape==auv_vision::ConeShape::kSquare?"square_cone":"unknown");
          if(id<0||id>=9||static_cast<std::size_t>(id)>=map_.grid.cells.size()||(matched&(1U<<id))||map_.grid.cells[id].cell.object_type!=shape)consistent=false;
          else matched|=1U<<id;
        }
        metric.semantic_match=consistent;
      }
      std::array<bool,9> visited;
      { std::lock_guard<std::mutex> lock(state_mutex_); visited = visited_; }
      auto map = auv_core::fuse_semantic_map(grid, cones, tracker.ready(), visited, cfg_.expected_cones);
      map.complete=map.grid.complete=map.complete && triggered && phase==auv_mission::MissionPhase::kBuildMap;
      if (map.complete && !map_image_saved_.exchange(true)) {
        try {
          std::filesystem::create_directories(cfg_.debug_dir);
          const auto filename=cfg_.debug_dir+"/map_"+std::to_string(static_cast<std::uint64_t>(seconds()*1000))+".jpg";
          if (!cv::imwrite(filename,grid.debug_image.empty() ? metric_image : grid.debug_image))
            throw std::runtime_error("cannot save grid evidence");
          if(!cv::imwrite(filename+".rectified.jpg",grid.rectified) ||
             (!cone_debug.empty() && !cv::imwrite(filename+".cones.jpg",cone_debug)))
            throw std::runtime_error("cannot save cone evidence");
          std::ofstream artifact(filename+".json");artifact.precision(15);
          artifact<<"{\"run_id\":\""<<run_id_<<"\",\"steady_sec\":"<<stamp
            <<",\"rows\":3,\"cols\":3,\"bottom_edge\":\""<<(cfg_.grid.single_yellow_edge?"yellow":"legacy_yellow_frame")<<"\",\"orientation_valid\":"
            <<(grid.orientation_valid?"true":"false")<<",\"confidence\":"<<grid.confidence<<",\"corners\":[";
          for(std::size_t i=0;i<grid.corners.size();++i){if(i)artifact<<',';artifact<<'['<<grid.corners[i].x<<','<<grid.corners[i].y<<']';}
          artifact<<"],\"cells\":[";
          for(std::size_t i=0;i<map.grid.cells.size();++i){if(i)artifact<<',';
            const auto& c=map.grid.cells[i].cell;artifact<<"{\"row\":"<<static_cast<int>(c.row)<<",\"col\":"
              <<static_cast<int>(c.col)<<",\"object\":\""<<c.object_type<<"\"}";}
          artifact<<"],\"cones\":[";
          for(std::size_t i=0;i<cones.size();++i){if(i)artifact<<',';const auto& c=cones[i];
            artifact<<"{\"row\":"<<c.row<<",\"col\":"<<c.col<<",\"shape\":\""<<auv_vision::cone_shape_name(c.shape)
              <<"\",\"confidence\":"<<c.confidence<<",\"contour\":[";
            for(std::size_t j=0;j<c.contour.size();++j){if(j)artifact<<',';artifact<<'['<<c.contour[j].x<<','<<c.contour[j].y<<']';}
            artifact<<"]}";}
          artifact<<"]}";artifact.flush();if(!artifact)throw std::runtime_error("cannot save semantic map");
          event("MAP_IMAGE",filename);event("MAP_COMPLETE",filename+".json");
        } catch (...) { log_degraded_=true;map_image_saved_=false;throw; }
      }
      {
        std::lock_guard<std::mutex> lock(video_mutex_);
        video_frame_ = grid.debug_image.empty() ? metric_image : grid.debug_image;
        ++video_sequence_;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      if(epoch==perception_epoch_)metric_pose_=metric;
      if(epoch==perception_epoch_ && mission_.snapshot().phase==auv_mission::MissionPhase::kBuildMap && !map_.complete) map_=map;
      pose_valid_ = grid.stable && grid.position_valid && std::isfinite(grid.camera_row) && std::isfinite(grid.camera_col) &&
        grid.camera_row >= 0 && grid.camera_row <= 3 && grid.camera_col >= 0 && grid.camera_col <= 3;
      row_ = grid.camera_row; col_ = grid.camera_col; pose_time_ = metric_stamp;
      const double finished=seconds();
      if (processed_time_ > 0 && finished > processed_time_)
        vision_hz_=vision_hz_ <= 0 ? 1.0/(finished-processed_time_) :
          0.9*vision_hz_+0.1/(finished-processed_time_);
      processed_time_ = finished; ++processed_frames_;
      latency_ms_[latency_index_++ % latency_ms_.size()] = (finished-stamp)*1000.0;
      latency_count_=std::min(latency_count_+1,latency_ms_.size());
      } catch (const std::exception& e) {
        fault(std::string("vision: ")+e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  }
  void send_frame(std::uint8_t type, const std::uint8_t* payload, std::size_t len) {
    std::array<std::uint8_t,AUV_PROTOCOL_MAX_FRAME_SIZE> bytes{};
    auto size = auv_protocol_encode_frame(type,payload,len,bytes.data(),bytes.size());
    if (!size || serial_.write(bytes.data(),size) != size) throw std::runtime_error("serial frame write failed");
  }
  void send_arm(bool arm) {
    std::array<std::uint8_t,5> p{};
    auv_protocol_write_u32_le(p.data(),++sequence_); p[4] = arm ? 1 : 0;
    send_frame(AUV_PROTOCOL_MSG_SET_ARMED,p.data(),p.size());
  }
  void serial_loop() {
    auv_stm32_bridge::StreamParser parser;
    auto next = Clock::now();
    while (running) {
      next += std::chrono::milliseconds(50);
      try {
        if (!serial_.is_open()) {
          if (cfg_.serial.empty()) { std::this_thread::sleep_until(next); continue; }
          serial_.open(cfg_.serial,cfg_.baud); parser.reset();
          std::lock_guard<std::mutex> lock(state_mutex_);
          serial_connected_ = false; armed_requested_ = false; disarm_pending_ = true;
          event("SERIAL_OPEN", cfg_.serial);
        }
        std::array<std::uint8_t,256> b{};
        auto n = serial_.read(b.data(),b.size());
        for (const auto& frame : parser.consume(b.data(),n)) {
          if (frame.message_type == AUV_PROTOCOL_MSG_STATUS) {
            auv_core::Stm32Status status;
            if (!auv_core::decode_status(frame.payload,status)) continue;
            std::lock_guard<std::mutex> lock(state_mutex_);
            status_ = std::move(status); status_time_ = seconds(); serial_connected_ = true;
          } else if (frame.message_type == AUV_PROTOCOL_MSG_DEPTH && (cfg_.traversal.enabled||cfg_.mission_profile=="tag_docking")) {
            const auto sample=auv_stm32_bridge::decode_depth_sample(frame.payload);
            std::lock_guard<std::mutex> lock(state_mutex_);depth_sample_=sample;depth_sample_received_=seconds();
          } else if (frame.message_type == AUV_PROTOCOL_MSG_ACK && frame.payload.size() == 6) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            const auto acknowledged_sequence=auv_protocol_read_u32_le(frame.payload.data()+2);
            if((cfg_.search.enabled||cfg_.mission_profile=="tag_docking") && frame.payload[0]==AUV_PROTOCOL_MSG_MOTION_TARGET &&
               frame.payload[1]!=0 && frame.payload[1]!=2) {
              set_fault_locked("STM32 rejected autonomous motion target");
              armed_requested_=false;disarm_pending_=true;
            }
            if (acknowledged_sequence == arm_sequence_ && frame.payload[0] == AUV_PROTOCOL_MSG_SET_ARMED) {
              arm_ack_ = frame.payload[1] == 0;
              if (!arm_ack_) { armed_requested_ = false; disarm_pending_ = true; set_fault_locked("STM32 rejected ARM"); }
            } else if (acknowledged_sequence == gripper_command_sequence_ &&
                       frame.payload[0] == AUV_PROTOCOL_MSG_ACTUATOR_COMMAND) {
              gripper_ack_=frame.payload[1] == 0;
              if (!gripper_ack_) set_fault_locked("STM32 rejected gripper command");
            }
          } else if (frame.message_type == AUV_PROTOCOL_MSG_ACTUATOR_STATUS) {
            auv_stm32_bridge::GripperTelemetry telemetry;
            if (!auv_stm32_bridge::decode_gripper_status(frame.payload,telemetry)) continue;
            std::lock_guard<std::mutex> lock(state_mutex_);
            gripper_=telemetry; gripper_time_=seconds();
          }
        }
        const auto now = seconds();
        auv_stm32_bridge::MotionTarget motion, neutral;
        bool send_motion = false, send_disarm = false, send_neutral = false,
          send_arm_request = false, reconnect = false;
        bool control_healthy = false;
        std::optional<auv_stm32_bridge::GripperAction> gripper_command;
        std::uint32_t gripper_sequence=0;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          control_healthy = now-last_control_time_.load() <= cfg_.control_watchdog_timeout;
          if (!control_healthy) {
            set_fault_locked("control loop watchdog timeout");
            armed_requested_=false; disarm_pending_=true; motion_={};
          }
          if (serial_connected_ && !status_fresh(now)) {
            set_fault_locked("STM32 STATUS timeout"); armed_requested_ = false; disarm_pending_ = true; motion_ = {};
            if (now-status_time_ > 2*cfg_.status_timeout) { serial_connected_=false; reconnect=true; }
          }
          if (armed_requested_ && arm_time_ > 0 && now-arm_time_ > 0.5 && (!arm_ack_ || !status_.armed)) {
            set_fault_locked("ARM acknowledgement or status timeout"); armed_requested_=false; disarm_pending_=true;
          }
          if (disarm_pending_ || (serial_connected_ && status_.armed && !armed_requested_)) {
            send_disarm = true; disarm_pending_ = false;
          }
          send_neutral = send_disarm && serial_connected_ && status_.armed;
          if (status_fresh(now) && std::isfinite(status_.depth) && std::isfinite(status_.yaw)) {
            neutral.depth=status_.depth; neutral.yaw=status_.yaw;
          }
          if (arm_pending_) { send_arm_request = true; arm_pending_ = false; }
          if (gripper_pending_) {
            gripper_command=gripper_pending_;
            gripper_pending_.reset();
            gripper_sequence=++sequence_;
            gripper_command_sequence_=gripper_sequence;
            gripper_command_time_=now;
            gripper_ack_=false;
          }
          if (gripper_command_time_ > 0 && !gripper_ack_ && now-gripper_command_time_ > 0.5)
            set_fault_locked("gripper command acknowledgement timeout");
          send_motion = cfg_.motion_enabled && armed_requested_ && arm_ack_ && status_.armed &&
            safe_status(now) && (cfg_.mission_profile=="tag_docking" ? docking_.active() :
              propulsion_phase(mission_.snapshot().phase,cfg_.traversal.enabled)) && fault_.empty();
          motion = send_motion ? motion_ : auv_stm32_bridge::MotionTarget{};
        }
        if (control_healthy) {
          std::array<std::uint8_t,8> heartbeat{};
          auv_protocol_write_u32_le(heartbeat.data(),++sequence_);
          auv_protocol_write_u32_le(heartbeat.data()+4,static_cast<std::uint32_t>(now*1000));
          send_frame(AUV_PROTOCOL_MSG_HEARTBEAT,heartbeat.data(),heartbeat.size());
          ++heartbeat_count_;
        }
        if (send_disarm) {
          if (send_neutral) {
            const auto p=auv_stm32_bridge::encode_motion_target_payload(++sequence_,neutral);
            send_frame(AUV_PROTOCOL_MSG_MOTION_TARGET,p.data(),p.size());
          }
          send_arm(false);
        }
        if (send_arm_request) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (armed_requested_ && fault_.empty()) {
            // Establish a fresh Pi control source before MCU processes ARM.
            auv_stm32_bridge::MotionTarget prime;prime.depth=status_.depth;prime.yaw=status_.yaw;
            const auto payload=auv_stm32_bridge::encode_motion_target_payload(++sequence_,prime);
            send_frame(AUV_PROTOCOL_MSG_MOTION_TARGET,payload.data(),payload.size());
            arm_sequence_ = sequence_ + 1; arm_time_=seconds(); send_arm(true);
          }
        }
        if (gripper_command) {
          const auto payload=auv_stm32_bridge::encode_gripper_command(gripper_sequence,*gripper_command);
          send_frame(AUV_PROTOCOL_MSG_ACTUATOR_COMMAND,payload.data(),payload.size());
        }
        if (send_motion) {
          const auto p = auv_stm32_bridge::encode_motion_target_payload(++sequence_,motion);
          send_frame(AUV_PROTOCOL_MSG_MOTION_TARGET,p.data(),p.size());
        }
        if (reconnect) { serial_.close(); parser.reset(); }
      } catch (const std::exception& e) {
        serial_.close();
        fault(std::string("serial: ")+e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      std::this_thread::sleep_until(next);
    }
    if (serial_.is_open()) {
      try {
        auv_stm32_bridge::MotionTarget neutral;
        { std::lock_guard<std::mutex> lock(state_mutex_);
          if (status_fresh(seconds())) { neutral.depth=status_.depth; neutral.yaw=status_.yaw; } }
        if (std::isfinite(neutral.depth) && std::isfinite(neutral.yaw)) {
          const auto payload=auv_stm32_bridge::encode_motion_target_payload(++sequence_,neutral);
          send_frame(AUV_PROTOCOL_MSG_MOTION_TARGET,payload.data(),payload.size());
        }
        const auto stop_payload=auv_stm32_bridge::encode_gripper_command(
          ++sequence_,auv_stm32_bridge::GripperAction::kStop);
        send_frame(AUV_PROTOCOL_MSG_ACTUATOR_COMMAND,stop_payload.data(),stop_payload.size());
        send_arm(false);
      }
      catch (...) {}
    }
  }
  void control_loop() {
    auto next = Clock::now(); std::uint64_t old_revision = 0;
    int missed_deadlines=0;
    while (running) {
      next += std::chrono::milliseconds(50);
      const auto now = seconds();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (Clock::now() > next + std::chrono::milliseconds(100)) {
          if (++missed_deadlines >= 3) {
            set_fault_locked("control loop deadline repeatedly missed");
            armed_requested_=false; disarm_pending_=true;
          }
          next=Clock::now();
        } else missed_deadlines=0;
        if(cfg_.mission_profile=="tag_docking") {
          dock_control_locked(now);++control_ticks_;last_control_time_=seconds();
        } else {
        auto phase = mission_.snapshot().phase;
        if (cfg_.auto_start && !autonomous_start_attempted_ && phase == auv_mission::MissionPhase::kInit) {
          const bool gripper_ready = !cfg_.mission.full_mission ||
            (gripper_status_fresh(now) && gripper_.calibrated && gripper_.error_flags == 0);
          const bool startup_ready = safe_status(now) && !status_.armed && gripper_ready && recording_ready(now) &&
            (!cfg_.traversal.enabled||depth_sample_fresh(now)) &&
            (!cfg_.auto_origin || localization_->snapshot(now).valid) && frame_time_ > 0 &&
            now-frame_time_ <= cfg_.frame_timeout && processed_time_ > 0 &&
            now-processed_time_ <= cfg_.frame_timeout;
          if (startup_ready) {
            if (startup_ready_since_ <= 0) startup_ready_since_=now;
          } else startup_ready_since_=0;
          if (now-boot_time_ >= cfg_.startup_delay && startup_ready_since_ > 0 &&
              now-startup_ready_since_ >= cfg_.startup_stable) {
            autonomous_start_attempted_ = true;
            if (claim_autonomous_run_locked()) {
              const auto result = mission_.command(auv_mission::MissionCommand::kStart,now);
              event("AUTO_START",result.message);
              phase = mission_.snapshot().phase;
            }
          } else if (now-boot_time_ > cfg_.startup_timeout) {
            autonomous_start_attempted_ = true;
            set_fault_locked("autonomous startup readiness timeout");
            disarm_pending_ = true;
          }
        }
        if (phase != auv_mission::MissionPhase::kInit && phase != auv_mission::MissionPhase::kComplete &&
            phase != auv_mission::MissionPhase::kFault && phase != auv_mission::MissionPhase::kAborted) {
          if (!safe_status(now)) {
            set_fault_locked("STM32 unsafe or STATUS timeout"); armed_requested_ = false; disarm_pending_ = true;
          }
          if (!fault_.empty()) { armed_requested_ = false; disarm_pending_ = true; }
          if(!recording_ready(now)) {
            set_fault_locked("onboard recording unavailable");armed_requested_=false;disarm_pending_=true;
          }
          if (frame_time_ <= 0 || now - frame_time_ > cfg_.frame_timeout) {
            set_fault_locked("camera frame timeout"); armed_requested_ = false; disarm_pending_ = true;
          }
          if (phase == auv_mission::MissionPhase::kVisitCones && !cfg_.traversal.enabled && (!pose_valid_ || now - pose_time_ > cfg_.pose_timeout)) {
            set_fault_locked("grid pose timeout"); armed_requested_ = false; disarm_pending_ = true;
          }
          if(cfg_.traversal.enabled && (!depth_sample_fresh(now) ||
             ((phase==auv_mission::MissionPhase::kVisitCones||phase==auv_mission::MissionPhase::kPlanCones) &&
              (!surface_grid_fresh(now)||std::abs(depth_sample_.depth_m-cfg_.traversal.surface_depth_m)>cfg_.traversal.depth_tolerance_m)))) {
            set_fault_locked("A2 depth telemetry, surface height or absolute grid pose invalid");
            armed_requested_=false;disarm_pending_=true;
          }
        }
        if(cfg_.traversal.enabled && fault_.empty()) {
          if(phase==auv_mission::MissionPhase::kRelocalizeSurface) {
            const bool ready=surface_grid_fresh(now) && metric_pose_.semantic_match &&
              std::abs(depth_sample_.depth_m-cfg_.traversal.surface_depth_m)<=cfg_.traversal.depth_tolerance_m;
            if(!ready)surface_pose_votes_=0;
            else if(metric_pose_.sequence!=surface_vote_sequence_) {
              surface_vote_sequence_=metric_pose_.sequence;++surface_pose_votes_;
            }
            surface_ready_=ready && surface_pose_votes_>=cfg_.traversal.reacquire_frames;
            mission_.update_surface_pose(surface_ready_,now);
          }
          if(metric_pose_.sequence!=trajectory_sequence_ || now-last_trajectory_time_>=0.1) {
            trajectory_sequence_=metric_pose_.sequence;last_trajectory_time_=now;
            queue_trajectory_locked(now,phase==auv_mission::MissionPhase::kVisitCones?"surface_grid":"phase_observation");
          }
          if(metric_pose_.valid && now>=metric_pose_.stamp && now-metric_pose_.stamp<=cfg_.traversal.pose_timeout_sec) {
            const int cell=static_cast<int>(metric_pose_.row)*3+static_cast<int>(metric_pose_.col);
            if(cell!=observed_cell_) {
              event("CELL_OBSERVED","phase="+auv_mission::mission_phase_name(phase)+" from="+std::to_string(observed_cell_)+" to="+std::to_string(cell)+" frame="+std::to_string(metric_pose_.sequence));
              observed_cell_=cell;
            }
          }
        }
        if (status_fresh(now)) mission_.update_status(serial_connected_,status_.armed,status_.error_flags,now);
        mission_.update_apriltag(tag_found_,now);
        mission_.update_map(map_.complete, all_visited_,now);
        if (phase == auv_mission::MissionPhase::kPlanCones && !route_ready_ && map_.complete &&
            (cfg_.traversal.enabled ? !surface_grid_fresh(now) : (!pose_valid_ || now-pose_time_ > cfg_.pose_timeout))) {
          set_fault_locked("grid pose unavailable for planning");
          armed_requested_=false; disarm_pending_=true;
        }
        if (phase == auv_mission::MissionPhase::kPlanCones && !route_ready_ && map_.complete && fault_.empty()) {
          const double measured_row=cfg_.traversal.enabled?metric_pose_.row:row_;
          const double measured_col=cfg_.traversal.enabled?metric_pose_.col:col_;
          auv_planning::GridCell start{static_cast<std::int8_t>(std::clamp(static_cast<int>(measured_row),0,2)),
            static_cast<std::int8_t>(std::clamp(static_cast<int>(measured_col),0,2)),"unknown"};
          plan_ = planner_.plan(map_.grid,start);
          route_ready_ = plan_.valid && !plan_.targets.empty();
          if (route_ready_) {
            if(cfg_.traversal.enabled) {
              route_ready_=surface_route_.set_route(plan_);
              if(!route_ready_)set_fault_locked("A2 rejected repeated or nonadjacent planned route");
              std::ostringstream artifact;artifact<<"{\"run_id\":\""<<run_id_<<"\",\"kind\":\"planned_only\",\"steady_sec\":"<<now<<",\"path\":[";
              for(std::size_t i=0;i<plan_.path.size();++i){if(i)artifact<<',';artifact<<'['<<static_cast<int>(plan_.path[i].row)<<','<<static_cast<int>(plan_.path[i].col)<<']';}
              artifact<<"],\"cost\":"<<plan_.total_cost<<",\"forbid_target_reentry\":true}";
              plan_artifact_pending_=artifact.str();
            }else route_.set_route(plan_,++map_revision_);
          }
          else { set_fault_locked("empty or invalid cone route: " + plan_.reason); disarm_pending_ = true; }
          std::ostringstream route_detail;
          route_detail << plan_.reason << " path=";
          for (const auto& cell : plan_.path)
            route_detail << '(' << static_cast<int>(cell.row) << ',' << static_cast<int>(cell.col) << ')';
          event("PLAN",route_detail.str());
        }
        mission_.update_route(route_ready_&&(!cfg_.traversal.enabled||plan_evidence_saved_),route_ready_,now);
        if (gripper_status_fresh(now)) {
          mission_.update_gripper(gripper_.state == 3U,gripper_.state == 5U,now);
        }
        if (fault_.empty()) mission_.tick(now);
        else mission_.force_fault(fault_,now);
        phase = mission_.snapshot().phase;
        if(cfg_.mission.surface_before_visit && !cfg_.traversal.enabled && phase==auv_mission::MissionPhase::kSurfaceForCones) {
          set_fault_locked("A2 surface traversal is disabled pending commissioning");
          mission_.force_fault(fault_,now);phase=mission_.snapshot().phase;
        }
        if (phase == auv_mission::MissionPhase::kGrab && !gripper_close_requested_) {
          if (!armed_requested_ || !status_.armed || !gripper_status_fresh(now) ||
              !gripper_.calibrated || gripper_.error_flags != 0) {
            set_fault_locked("gripper unavailable, uncalibrated, or faulted");
            mission_.force_fault(fault_,now); phase=mission_.snapshot().phase;
          } else {
            gripper_pending_=auv_stm32_bridge::GripperAction::kClose;
            gripper_close_requested_=true; event("GRIPPER_COMMAND","close");
          }
        }
        if (phase == auv_mission::MissionPhase::kRelease && !gripper_open_requested_) {
          if (!armed_requested_ || !status_.armed || !gripper_status_fresh(now) ||
              !gripper_.calibrated || gripper_.error_flags != 0) {
            set_fault_locked("gripper unavailable, uncalibrated, or faulted");
            mission_.force_fault(fault_,now); phase=mission_.snapshot().phase;
          } else {
            gripper_pending_=auv_stm32_bridge::GripperAction::kOpen;
            gripper_open_requested_=true; event("GRIPPER_COMMAND","open");
          }
        }
        if (cfg_.auto_arm && !autonomous_arm_attempted_ &&
            (phase == auv_mission::MissionPhase::kVisitCones ||
             (cfg_.search.enabled && phase==auv_mission::MissionPhase::kSearchAprilTag))) {
          autonomous_arm_attempted_ = true;
          if (arm_gate_ready(now)) request_arm_locked("autonomous");
          else {
            set_fault_locked("autonomous ARM safety gate rejected");
            armed_requested_ = false; disarm_pending_ = true;
            mission_.force_fault(fault_,now);
            phase = mission_.snapshot().phase;
          }
        }
        if (!fault_.empty() || phase == auv_mission::MissionPhase::kFault) {
          armed_requested_ = false; disarm_pending_ = true; motion_ = {};
          if (!gripper_stop_requested_) {
            gripper_pending_=auv_stm32_bridge::GripperAction::kStop;
            gripper_stop_requested_=true;
          }
        } else {
          route_.set_mission_active(!cfg_.traversal.enabled && phase == auv_mission::MissionPhase::kVisitCones);
          route_.set_vehicle_ready(cfg_.motion_enabled && armed_requested_ && arm_ack_ && status_.armed && safe_status(now));
          route_.set_pose(pose_valid_ && now-pose_time_ <= cfg_.pose_timeout,row_,col_);
          const auto step = route_.step();
          waypoint_index_ = step.waypoint_index;
          if (!cfg_.traversal.enabled && phase == auv_mission::MissionPhase::kVisitCones && step.state == auv_control::RouteStep::State::kFault)
            { set_fault_locked(step.detail); armed_requested_ = false; disarm_pending_ = true; }
          if (step.visited_cell) {
            auto i = static_cast<std::size_t>(step.visited_cell->row*3+step.visited_cell->col);
            visited_[i] = true; event("CONE_VISITED",std::to_string(i));
          }
          if(!cfg_.traversal.enabled) {
            all_visited_ = !plan_.targets.empty();
            for (const auto& t : plan_.targets) all_visited_ &= visited_[static_cast<std::size_t>(t.row*3+t.col)];
          }
          motion_ = {};
          if (armed_requested_) { motion_.depth=hold_depth_; motion_.yaw=hold_yaw_; }
          if (step.state == auv_control::RouteStep::State::kRunning && armed_requested_ && fault_.empty()) {
            motion_.vx = static_cast<float>(step.surge); motion_.vy = static_cast<float>(step.sway);
            motion_.depth = hold_depth_; motion_.yaw = hold_yaw_;
          }
          if(cfg_.search.enabled && (phase==auv_mission::MissionPhase::kSearchAprilTag ||
                                    phase==auv_mission::MissionPhase::kBuildMap)) {
            if(armed_requested_) {
              const auto pose=localization_->snapshot(now);
              const auto observation=search_.step(pose,now,tag_found_);
              search_detail_=observation.reason;search_waypoint_=observation.waypoint;
              if(observation.fault||observation.exhausted) {
                set_fault_locked(observation.reason);armed_requested_=false;disarm_pending_=true;motion_={};
                mission_.force_fault(fault_,now);
              }else if(arm_ack_ && status_.armed){
                motion_.vx=static_cast<float>(observation.surge);motion_.vy=static_cast<float>(observation.sway);
              }
            }
          }
          if(cfg_.traversal.enabled && (phase==auv_mission::MissionPhase::kSurfaceForCones ||
             phase==auv_mission::MissionPhase::kRelocalizeSurface ||phase==auv_mission::MissionPhase::kPlanCones ||
             phase==auv_mission::MissionPhase::kVisitCones)) {
            if(!armed_requested_||!arm_ack_||!status_.armed) {
              set_fault_locked("A2 armed control continuity lost");
            }else if(phase==auv_mission::MissionPhase::kSurfaceForCones) {
              if(!ascent_started_) {
                const auto approach=center_approach_.step(metric_pose_,now);a2_detail_=approach.detail;
                motion_.vx=static_cast<float>(approach.surge);motion_.vy=static_cast<float>(approach.sway);
                if(approach.fault)set_fault_locked(approach.detail);
                else if(approach.complete) {
                  if(!ascent_.begin(metric_pose_,depth_sample_.depth_m,now))set_fault_locked("measured ascent column unavailable");
                  else {ascent_started_=true;event("ASCENT_STARTED","center alignment confirmed; zero lateral thrust");}
                }
              }else {
              const auto rise=ascent_.step(depth_sample_.depth_m,safe_status(now)&&depth_sample_fresh(now),now,depth_sample_.sensor_sequence);
              a2_detail_=rise.detail;hold_depth_=static_cast<float>(rise.target_depth);
              motion_.depth=hold_depth_;motion_.vx=motion_.vy=0;
              if(rise.fault)set_fault_locked(rise.detail);
              if(rise.complete && !surface_confirmed_) {
                surface_confirmed_=true;surface_reacquire_start_=now;surface_vote_sequence_=0;surface_pose_votes_=0;
                mission_.update_surface(true,now);event("SURFACE_CONFIRMED","independent pressure samples; underwater integration discarded for traversal");
              }
              }
            }else {
              motion_.depth=hold_depth_=static_cast<float>(cfg_.traversal.surface_depth_m);
              motion_.vx=motion_.vy=0;
              if(phase==auv_mission::MissionPhase::kVisitCones) {
                const auto actual=surface_route_.step(metric_pose_,now);a2_detail_=actual.detail;waypoint_index_=actual.waypoint;
                if(actual.entered)event("CELL_ENTERED","row="+std::to_string(actual.entered->row)+" col="+std::to_string(actual.entered->col)+" frame="+std::to_string(metric_pose_.sequence));
                if(actual.visited) {
                  const int id=actual.visited->row*3+actual.visited->col;visited_[id]=true;
                  event("CONE_VISITED","measured row="+std::to_string(actual.visited->row)+" col="+std::to_string(actual.visited->col)+" frame="+std::to_string(metric_pose_.sequence));
                }
                if(actual.fault)set_fault_locked(actual.detail);
                else {motion_.vx=static_cast<float>(actual.surge);motion_.vy=static_cast<float>(actual.sway);all_visited_=actual.complete;}
              }else a2_detail_="holding surface depth; fresh grid reacquisition / evidence gate";
            }
            if(!fault_.empty()){armed_requested_=false;disarm_pending_=true;motion_={};mission_.force_fault(fault_,now);}
          }
        }
        const auto snapshot = mission_.snapshot();
        ++control_ticks_;
        if (snapshot.revision != old_revision) { old_revision = snapshot.revision; event("MISSION",auv_mission::mission_phase_name(snapshot.phase)); }
        if (now-last_state_log_sec_ >= 1.0) {
          last_state_log_sec_=now;
          std::ostringstream detail;
          detail << auv_mission::mission_phase_name(snapshot.phase)
                 << " serial=" << serial_connected_ << " armed=" << status_.armed
                 << " grid=" << map_.complete << " pose=" << pose_valid_
                 << " waypoint=" << waypoint_index_;
          event("STATE",detail.str());
        }
        if (snapshot.phase == auv_mission::MissionPhase::kComplete) {
          if(cfg_.traversal.enabled&&!a2_completion_logged_) {
            a2_completion_logged_=true;queue_trajectory_locked(now,"a2_complete");
            event("A2_COMPLETE","four measured cone visits; stage end; full competition mission remains incomplete");
          }
          armed_requested_ = false; disarm_pending_ = true;motion_={};
        }
        last_control_time_=seconds();
        }
      }
      std::this_thread::sleep_until(next);
    }
  }
  std::string camera_calibration_json() {
    bool surface_frame=false;
    {std::lock_guard<std::mutex> lock(frame_mutex_);surface_frame=down_surface_frame_;}
    std::ostringstream out;
    auto camera=[&](bool rectify,int width,int height,const std::string& id,
        const std::string& quality,const std::vector<double>& k,const std::vector<double>& d) {
      out << "{\"preview_rectified\":" << (rectify?"true":"false")
          << ",\"calibration_id\":\"" << json_escape(id) << "\",\"quality\":\"" << json_escape(quality)
          << "\",\"width\":" << width << ",\"height\":" << height << ",\"camera_matrix\":[";
      for(std::size_t i=0;i<k.size();++i){if(i)out<<',';out<<k[i];}
      out << "],\"raw_distortion_coefficients\":[";
      for(std::size_t i=0;i<d.size();++i){if(i)out<<',';out<<d[i];}
      out << "],\"preview_distortion_coefficients\":[";
      for(std::size_t i=0;i<d.size();++i){if(i)out<<',';out<<(rectify?0:d[i]);}
      out << "]}";
    };
    out.precision(17);
    out << "{\"down\":";
    if(surface_frame) {
      const auto& model=cfg_.traversal.surface;
      camera(cfg_.down_preview_rectify,model.width,model.height,"surface-traversal",
          "independently_verified",model.camera_matrix,model.distortion);
    } else camera(cfg_.down_preview_rectify,cfg_.down_calibration_width,cfg_.down_calibration_height,
        cfg_.down_calibration_id,cfg_.down_calibration_quality,cfg_.camera_matrix,cfg_.distortion);
    out << ",\"front\":";
    camera(cfg_.front_enabled && cfg_.front_preview_rectify,cfg_.front_calibration_width,cfg_.front_calibration_height,
        cfg_.front_calibration_id,cfg_.front_calibration_quality,cfg_.front_camera_matrix,cfg_.front_distortion);
    out << '}';return out.str();
  }
  std::string command(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = seconds();
    if (cmd == "status") {
      std::ostringstream s;
      auto latency=std::vector<double>(latency_ms_.begin(),latency_ms_.begin()+latency_count_);
      std::sort(latency.begin(),latency.end());
      double p99=latency.empty() ? -1.0 : latency[static_cast<std::size_t>(0.99*(latency.size()-1))];
      s << "{\"phase\":\"" << (cfg_.mission_profile=="tag_docking" ? docking_.name() : auv_mission::mission_phase_name(mission_.snapshot().phase))
        << "\",\"run_id\":\""<<run_id_<<"\",\"recording_directory\":\""<<json_escape(run_dir_)
        << "\",\"recording_enabled\":"<<(cfg_.recording_enabled?"true":"false")
        << ",\"recording_ready\":"<<(recording_ready(now)?"true":"false")
        << ",\"recording_detail\":\""<<json_escape(recording_detail_)
        << "\",\"search_detail\":\""<<json_escape(search_detail_)<<"\",\"search_waypoint\":"<<search_waypoint_
        << ",\"origin_ready\":"<<(localization_->snapshot(now).valid?"true":"false")
        << ",\"operation_mode\":\"" << cfg_.operation_mode
        << "\",\"mission_profile\":\"" << cfg_.mission_profile
        << "\",\"auto_start\":" << (cfg_.auto_start ? "true":"false")
        << ",\"auto_arm\":" << (cfg_.auto_arm ? "true":"false")
        << ",\"serial\":" << (serial_connected_ ? "true":"false")
        << ",\"armed\":" << (status_.armed ? "true":"false")
        << ",\"motion_enabled\":" << (cfg_.motion_enabled ? "true":"false")
        << ",\"a2_enabled\":"<<(cfg_.traversal.enabled?"true":"false")
        << ",\"a2_detail\":\""<<json_escape(a2_detail_)<<"\",\"surface_confirmed\":"<<(surface_confirmed_?"true":"false")
        << ",\"surface_pose_ready\":"<<(surface_ready_?"true":"false")
        << ",\"surface_grid_valid\":"<<(surface_grid_fresh(now)?"true":"false")
        << ",\"surface_grid_row\":"<<json_number(metric_pose_.row)<<",\"surface_grid_col\":"<<json_number(metric_pose_.col)
        << ",\"depth_sensor_sequence\":"<<depth_sample_.sensor_sequence
        << ",\"grid_pose_fresh\":"<<(pose_valid_ && pose_time_>0 && now-pose_time_<=cfg_.pose_timeout?"true":"false")
        << ",\"status_fresh\":"<<(status_fresh(now)?"true":"false")
        << ",\"status_age_sec\":"<<json_number(status_time_?now-status_time_:-1)
        << ",\"safe_status\":"<<(safe_status(now)?"true":"false")
        << ",\"arm_gate_ready\":"<<(arm_gate_ready(now)?"true":"false")
        << ",\"depth_m\":"<<json_number(status_.depth)
        << ",\"roll_rad\":"<<json_number(status_.roll)<<",\"pitch_rad\":"<<json_number(status_.pitch)
        << ",\"yaw_rad\":"<<json_number(status_.yaw)<<",\"voltage_v\":"<<json_number(status_.voltage)
        << ",\"depth_sample_fresh\":"<<(depth_sample_fresh(now)?"true":"false")
        << ",\"depth_sample_age_sec\":"<<json_number(depth_sample_received_?now-depth_sample_received_+depth_sample_.age_sec:-1)
        << ",\"metric_pose_reason\":\""<<json_escape(metric_pose_.reason)<<"\""
        << ",\"metric_reprojection_px\":"<<json_number(metric_pose_.reprojection_px)
        << ",\"frame_timeout_sec\":"<<cfg_.frame_timeout<<",\"status_timeout_sec\":"<<cfg_.status_timeout
        << ",\"cmd_vx\":"<<json_number(motion_.vx)<<",\"cmd_vy\":"<<json_number(motion_.vy)
        << ",\"cmd_depth\":"<<json_number(motion_.depth)<<",\"cmd_yaw\":"<<json_number(motion_.yaw)
        << ",\"tag_docking\":{\"detail\":\""<<json_escape(docking_.reason)
        <<"\",\"target_id\":"<<cfg_.docking.id<<",\"tag_size_m\":"<<cfg_.docking.size_m
        <<",\"maximum_depth_m\":"<<cfg_.docking.max_depth
        <<",\"geometry_verified\":"<<(cfg_.docking.geometry_verified?"true":"false")
        <<",\"corridor_verified\":"<<(cfg_.docking.corridor_verified?"true":"false")
        <<",\"depth_verified\":"<<(cfg_.docking.depth_verified?"true":"false")
        <<",\"startup_ready\":"<<(cfg_.mission_profile=="tag_docking"&&docking_.phase==TagDockTask::Phase::Idle&&
          cfg_.motion_enabled&&fault_.empty()&&!status_.armed&&dock_ready(now)&&dock_front_ready(now)&&
          depth_sample_.depth_m>=cfg_.docking.min_depth&&depth_sample_.depth_m<=cfg_.docking.max_depth ? "true":"false")
        <<",\"front_detected\":"<<(dock_front_.detected?"true":"false")
        <<",\"front_metric_valid\":"<<(dock_front_.metric_valid?"true":"false")
        <<",\"front_quality_reason\":\""<<json_escape(dock_front_.quality_reason)<<"\""
        <<",\"down_detected\":"<<(dock_down_.detected?"true":"false")
        <<",\"down_metric_valid\":"<<(dock_down_.metric_valid?"true":"false")
        <<",\"down_quality_reason\":\""<<json_escape(dock_down_.quality_reason)<<"\""
        <<",\"down_center_error_px\":"<<(dock_down_.detected?json_number(dock_down_.pixel_error):"null")
        <<",\"planned_goal_x_m\":"<<json_number(docking_.goal_x)<<",\"planned_goal_y_m\":"<<json_number(docking_.goal_y)
        <<",\"depth_report_target_m\":"<<json_number(docking_.report_target)
        <<",\"depth_report_mean_m\":"<<json_number(docking_.report_mean)
        <<",\"depth_report_min_m\":"<<json_number(docking_.report_min)
        <<",\"depth_report_max_m\":"<<json_number(docking_.report_max)
        <<",\"depth_report_error_m\":"<<json_number(docking_.report_mean-docking_.report_target)
        <<",\"depth_report_samples\":"<<docking_.report_samples
        <<",\"completion_evidence_saved\":"<<(dock_evidence_saved_?"true":"false")<<"}"
        << ",\"planned_path\":[";
      for(std::size_t i=0;i<plan_.path.size();++i){if(i)s<<',';s<<'['<<static_cast<int>(plan_.path[i].row)<<','<<static_cast<int>(plan_.path[i].col)<<']';}
      s << "]"
        << ",\"telemetry_valid\":" << (status_.telemetry_valid ? "true":"false")
        << ",\"mcu_dual_mode\":" << (status_.dual_mode ? "true":"false")
        << ",\"mcu_operating_mode\":\"" << (status_.dual_mode ? (status_.autonomous_mode ? "auv" : "rov") : "fixed_profile") <<"\""
        << ",\"voltage_valid\":" << (status_.voltage_valid ? "true":"false")
        << ",\"camera_age_sec\":" << (frame_time_ ? now-frame_time_ : -1)
        << ",\"down_source\":\"" << json_escape(cfg_.camera) << "\""
        << ",\"front_source\":\"" << json_escape(cfg_.front_source) << "\""
        << ",\"camera_calibration\":" << camera_calibration_json()
        << ",\"vision_image_space\":\"" << (cfg_.camera_matrix.empty()?"raw":"rectified") << "\""
        << ",\"down_capture_frames\":" << down_capture_frames_.load()
        << ",\"down_hz\":" << down_hz_.load()
        << ",\"video_enabled\":" << (cfg_.video_enabled ? "true":"false")
        << ",\"front_enabled\":" << (cfg_.front_enabled ? "true":"false")
        << ",\"front_camera_age_sec\":" << (front_time_ ? now-front_time_ : -1)
        << ",\"front_frames\":" << front_frames_
        << ",\"front_hz\":" << front_hz_
        << ",\"front_degraded\":" << (cfg_.front_enabled &&
            (!front_time_ || now-front_time_>cfg_.frame_timeout || !front_detail_.empty()) ? "true":"false")
        << ",\"front_detail\":\"" << json_escape(front_detail_) << "\""
        << ",\"vision_frames\":" << processed_frames_
        << ",\"vision_hz\":" << vision_hz_
        << ",\"vision_latency_p99_ms\":" << p99
        << ",\"control_ticks\":" << control_ticks_
        << ",\"control_age_sec\":" << now-last_control_time_.load()
        << ",\"heartbeats\":" << heartbeat_count_.load()
        << ",\"vision_age_sec\":" << (processed_time_ ? now-processed_time_ : -1)
        << ",\"grid_complete\":" << (map_.complete ? "true":"false")
        << ",\"apriltag_found\":" << (tag_found_ ? "true":"false")
        << ",\"cone_count\":" << map_.cone_count << ",\"route_waypoints\":" << plan_.path.size()
        << ",\"pose_valid\":" << (pose_valid_ ? "true":"false")
        << ",\"row\":" << (std::isfinite(row_) ? row_ : -1.0F)
        << ",\"col\":" << (std::isfinite(col_) ? col_ : -1.0F)
        << ",\"next_waypoint_index\":" << waypoint_index_
        << ",\"next_row\":" << (waypoint_index_ < plan_.path.size() ? static_cast<int>(plan_.path[waypoint_index_].row) : -1)
        << ",\"next_col\":" << (waypoint_index_ < plan_.path.size() ? static_cast<int>(plan_.path[waypoint_index_].col) : -1)
        << ",\"error_flags\":" << status_.error_flags
        << ",\"gripper_fresh\":" << (gripper_status_fresh(now) ? "true":"false")
        << ",\"gripper_calibrated\":" << (gripper_.calibrated ? "true":"false")
        << ",\"gripper_state\":" << static_cast<unsigned>(gripper_.state)
        << ",\"gripper_error_flags\":" << static_cast<unsigned>(gripper_.error_flags)
        << ",\"cells\":[";
      for (std::size_t i=0;i<map_.grid.cells.size();++i) {
        if (i) s << ',';
        const auto& cell=map_.grid.cells[i];
        s << "{\"row\":" << static_cast<int>(cell.cell.row)
          << ",\"col\":" << static_cast<int>(cell.cell.col)
          << ",\"object\":\"" << json_escape(cell.cell.object_type)
          << "\",\"visited\":" << (visited_[i] ? "true":"false") << '}';
      }
      s << "]"
        << ",\"video_degraded\":" << (video_detail_.empty() ? "false":"true")
        << ",\"log_degraded\":" << (log_degraded_ ? "true":"false")
        << ",\"video_detail\":\"" << json_escape(video_detail_)
        << "\",\"web_detail\":\"" << json_escape(web_detail_)
        << "\",\"fault\":\"" << json_escape(fault_) << "\"}\n";
      return s.str();
    }
    if (cfg_.operation_mode == "autonomous" && cmd != "status" && cmd != "disarm") {
      event("COMMAND_REJECTED",cmd+": autonomous mode");
      return "ERR operator command disabled in autonomous mode\n";
    }
    if (cmd == "disarm" || cmd == "abort" || cmd == "pause" || cmd == "reset") {
      armed_requested_ = false; arm_ack_ = false; arm_pending_ = false;
      arm_time_=0;
      disarm_pending_ = true; motion_ = {};
      gripper_pending_=auv_stm32_bridge::GripperAction::kStop;
      gripper_stop_requested_=true;
    }
    if (cmd == "arm SAFE_TO_ARM") {
      if (!cfg_.motion_enabled || !cfg_.directions_calibrated || !cfg_.limits_calibrated || cfg_.serial.empty()) return "ERR motion configuration disabled or uncalibrated\n";
      if (!arm_gate_ready(now))
        return "ERR ARM safety gate rejected\n";
      request_arm_locked("operator"); return "OK ARM requested\n";
    }
    if (cmd == "arm") return "ERR use arm --confirm SAFE_TO_ARM\n";
    if (cmd == "disarm") {
      if(cfg_.mission_profile=="tag_docking")docking_.fail("operator DISARM");
      event("DISARM", "operator");
      if (cfg_.operation_mode == "autonomous") {
        set_fault_locked("emergency DISARM requested");
        mission_.force_fault(fault_,now);
      }
      return "OK DISARM requested\n";
    }
    if(cfg_.mission_profile=="tag_docking") {
      if(cmd=="abort"||cmd=="pause") {docking_.fail("operator "+cmd);return "OK task stopped; DISARM requested\n";}
      if(cmd=="start") {
        if(!cfg_.motion_enabled)return "ERR tag docking motion disabled; commission configuration first\n";
        if(!fault_.empty()||status_.armed||!dock_ready(now))return "ERR tag docking startup safety gate\n";
        if(!dock_front_ready(now))return "ERR fresh calibrated front tag 18 required at start\n";
        if(!docking_.start(now,depth_sample_.depth_m,localization_->snapshot(now)))return "ERR task already started or invalid depth/origin\n";
        event("TAG_DOCK_START","explicit start; waiting for explicit ARM");return "OK task started; explicit ARM required\n";
      }
      return "ERR tag docking requires a new process/run_id after termination\n";
    }
    std::optional<auv_mission::MissionCommand> c;
    if (cmd == "start") c = auv_mission::MissionCommand::kStart;
    if (cmd == "pause") c = auv_mission::MissionCommand::kPause;
    if (cmd == "resume") c = auv_mission::MissionCommand::kResume;
    if (cmd == "abort") c = auv_mission::MissionCommand::kAbort;
    if (cmd == "reset") c = auv_mission::MissionCommand::kReset;
    if (!c) return "ERR unknown command\n";
    if((cmd=="resume"||cmd=="reset") && cfg_.mission.surface_before_visit)
      return "ERR A2 requires process restart and a new run_id after pause or termination\n";
    if(cmd=="start" && cfg_.auto_origin && !localization_->snapshot(now).valid)
      return "ERR calibrated stationary localization origin required\n";
    if(cmd=="start" && !recording_ready(now)) return "ERR onboard recording required\n";
    auto r = mission_.command(*c,now);
    if (cmd == "reset" && r.accepted) {
      fault_.clear(); route_.reset(); route_ready_=false; plan_={}; map_={};
      visited_.fill(false); all_visited_=false; tag_found_=false;
      pose_valid_=false; pose_time_=0; waypoint_index_=0;
      gripper_close_requested_=false; gripper_open_requested_=false;
      gripper_stop_requested_=false;
      gripper_pending_.reset(); gripper_ack_=false; gripper_command_time_=0;
      map_image_saved_=false;
      ++perception_epoch_;search_.reset();
      ascent_.reset();surface_route_.reset();center_approach_.reset();ascent_started_=surface_confirmed_=surface_ready_=false;
      surface_pose_votes_=0;surface_vote_sequence_=0;surface_reacquire_start_=0;metric_pose_={};
      plan_evidence_saved_=a2_completion_logged_=false;plan_artifact_pending_.clear();
    }
    event("COMMAND",cmd+": "+r.message);
    return std::string(r.accepted ? "OK ":"ERR ")+r.message+"\n";
  }
  void video_loop() {
    if (!cfg_.video_enabled) return;
    std::filesystem::create_directories(cfg_.video_dir);
    std::uint64_t seen = 0;
    int pipefd[2];
    if (::pipe2(pipefd,O_CLOEXEC) != 0) { std::lock_guard<std::mutex> lock(state_mutex_); video_detail_="pipe failed"; return; }
    const std::string output=cfg_.video_dir+"/index.m3u8";
    const std::string size=std::to_string(cfg_.video_width)+"x"+std::to_string(cfg_.video_height);
    const std::string fps=std::to_string(cfg_.video_fps);
    const std::string bitrate=std::to_string(cfg_.video_bitrate_kbps)+"k";
    const std::string gop=std::to_string(std::max(1,static_cast<int>(cfg_.video_fps*cfg_.segment_time)));
    const std::string segment=std::to_string(cfg_.segment_time);
    std::vector<std::string> args={"ffmpeg","-hide_banner","-loglevel","error","-nostdin","-y",
      "-filter_threads","1","-f","rawvideo","-pixel_format","bgr24","-video_size",size,"-framerate",fps,
      "-i","pipe:0","-an","-c:v",cfg_.video_encoder,"-b:v",bitrate,"-g",gop,"-keyint_min",gop,"-sc_threshold","0"};
    if (cfg_.video_encoder=="libx264")
      args.insert(args.end(),{"-preset","ultrafast","-tune","zerolatency","-threads","1"});
    args.insert(args.end(),{"-bsf:v","extract_extradata,dump_extra=freq=keyframe","-f","hls",
      "-hls_time",segment,"-hls_list_size","6","-hls_flags","delete_segments+independent_segments",output});
    std::vector<char*> argv;
    for (auto& arg:args) argv.push_back(arg.data());
    argv.push_back(nullptr);
    pid_t child = ::fork();
    if (child == 0) {
      ::dup2(pipefd[0],STDIN_FILENO); ::close(pipefd[0]); ::close(pipefd[1]);
      ::execvp(argv[0],argv.data());
      _exit(127);
    }
    ::close(pipefd[0]);
    if (child < 0) { ::close(pipefd[1]); std::lock_guard<std::mutex> lock(state_mutex_); video_detail_="fork failed"; return; }
    auto next=Clock::now();
    while (running) {
      next+=std::chrono::milliseconds(1000/cfg_.video_fps);
      int exit_status=0;
      if (::waitpid(child,&exit_status,WNOHANG) == child) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        video_detail_="FFmpeg unavailable or hardware encoder failed";
        running_video_=false;
        break;
      }
      cv::Mat image;
      /* Preview consumes capture directly; perception rate must not cap video FPS. */
      { std::lock_guard<std::mutex> lock(frame_mutex_);
        if (cfg_.front_enabled || seen != frame_sequence_) { seen=frame_sequence_; image=down_preview_frame_; } }
      if (cfg_.front_enabled) {
        cv::Mat front;
        { std::lock_guard<std::mutex> lock(front_mutex_); front=front_preview_frame_; }
        cv::Mat composite(cfg_.video_height,cfg_.video_width,CV_8UC3,cv::Scalar(0,0,0));
        const int half=cfg_.video_width/2;
        if (!image.empty() && frame_time_>0 && seconds()-frame_time_<=cfg_.frame_timeout)
          cv::resize(image,composite(cv::Rect(0,0,half,cfg_.video_height)),{half,cfg_.video_height});
        if (!front.empty() && front_time_>0 && seconds()-front_time_<=cfg_.frame_timeout)
          cv::resize(front,composite(cv::Rect(half,0,cfg_.video_width-half,cfg_.video_height)),
            {cfg_.video_width-half,cfg_.video_height});
        cv::putText(composite,"DOWN / CSI",{8,20},cv::FONT_HERSHEY_SIMPLEX,.45,{0,255,255},1);
        cv::putText(composite,"FRONT / USB",{half+8,20},cv::FONT_HERSHEY_SIMPLEX,.45,{0,255,255},1);
        image=std::move(composite);
      }
      if (image.empty() && (frame_time_ <= 0 || seconds()-frame_time_ > cfg_.frame_timeout)) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        video_detail_="waiting for down camera frames";
      }
      if (!image.empty()) {
        cv::Mat resized;
        if (image.cols != cfg_.video_width || image.rows != cfg_.video_height)
          cv::resize(image,resized,{cfg_.video_width,cfg_.video_height});
        else resized=image;
        if (!resized.isContinuous()) resized=resized.clone();
        const auto* bytes=resized.ptr<std::uint8_t>();
        std::size_t size=static_cast<std::size_t>(cfg_.video_width)*cfg_.video_height*3U,offset=0;
        while (offset<size && running) {
          auto n=::write(pipefd[1],bytes+offset,size-offset);
          if (n <= 0) { std::lock_guard<std::mutex> lock(state_mutex_); video_detail_="FFmpeg encoder failed"; running_video_=false; break; }
          offset+=static_cast<std::size_t>(n);
        }
        if (!running_video_) break;
        { std::lock_guard<std::mutex> lock(state_mutex_);
          if (video_detail_ == "waiting for down camera frames") video_detail_.clear(); }
      }
      if (next < Clock::now()) next=Clock::now(); /* No catch-up frame bursts. */
      std::this_thread::sleep_until(next);
    }
    ::close(pipefd[1]);
    int status=0; ::waitpid(child,&status,0);
    if (running && cfg_.software_fallback && cfg_.video_encoder != "libx264") {
      std::lock_guard<std::mutex> lock(state_mutex_);
      video_detail_="hardware encoder failed; switching to explicitly enabled libx264";
      cfg_.video_encoder="libx264";
      running_video_=true;
    }
    if (running && cfg_.software_fallback && cfg_.video_encoder == "libx264" && running_video_)
      video_loop();
  }
#ifdef AUV_HAVE_HTTPLIB
  void web_loop() {
    if(!cfg_.web_enabled)return;
    if (!running) return;
    httplib::Server server;
    web_server_=&server;
    server.Get("/api/localization",[this](const httplib::Request&,httplib::Response& res){
      res.set_header("Cache-Control","no-store");res.set_content(localization_->json(seconds()),"application/json");
    });
    server.Post("/api/localization/reset",[this](const httplib::Request&,httplib::Response& res){
      if(cfg_.operation_mode=="autonomous"||cfg_.mission_profile=="tag_docking"){res.status=403;res.set_content("{\"error\":\"origin reset disabled in autonomous/tag docking mode\"}","application/json");return;}
      if(!localization_->reset(seconds())){res.status=409;res.set_content("{\"error\":\"Require calibrated fresh input, DISARM and 2 seconds stable attitude/depth\"}","application/json");return;}
      event("LOCALIZATION_RESET",localization_->json(seconds()));
      res.set_content(localization_->json(seconds()),"application/json");
    });
    server.Get("/api/status",[this](const httplib::Request&,httplib::Response& res){
      res.set_content(command("status"),"application/json");
      res.set_header("Cache-Control","no-store");
    });
    auto cached=[this](bool front,std::vector<std::uint8_t>& jpeg,double& stamp) {
      if(front) { std::lock_guard<std::mutex> lock(front_mutex_); jpeg=front_jpeg_; stamp=front_time_.load(); }
      else { std::lock_guard<std::mutex> lock(frame_mutex_); jpeg=down_jpeg_; stamp=frame_time_.load(); }
      return !jpeg.empty() && stamp>0 && seconds()-stamp<=cfg_.frame_timeout;
    };
    auto snapshot=[this,cached](bool front,httplib::Response& res) {
      std::vector<std::uint8_t> jpeg;
      double stamp;
      if(!cached(front,jpeg,stamp)) { res.status=503; return; }
      res.set_content(reinterpret_cast<const char*>(jpeg.data()),jpeg.size(),"image/jpeg");
      res.set_header("Cache-Control","no-store");
      res.set_header("X-Frame-Time-Monotonic",std::to_string(stamp));
      res.set_header("X-Camera-Source",front ? cfg_.front_source : cfg_.camera);
    };
    server.Get("/api/camera/down.jpg",[snapshot](const httplib::Request&,httplib::Response& res){snapshot(false,res);});
    server.Get("/api/camera/front.jpg",[snapshot](const httplib::Request&,httplib::Response& res){snapshot(true,res);});
    server.new_task_queue=[] { return new httplib::ThreadPool(8); };
    server.set_write_timeout(2,0);
    server.set_tcp_nodelay(true);
    auto stream=[this,cached](bool front,httplib::Response& res) {
      /* Reserve worker capacity for status and control requests. */
      if (mjpeg_clients_.fetch_add(1)>=4) {
        --mjpeg_clients_; res.status=429; return;
      }
      res.set_header("Cache-Control","no-store");
      res.set_chunked_content_provider("multipart/x-mixed-replace; boundary=auvframe",
        [this,cached,front,last=-1.0,delivered=seconds()](size_t,httplib::DataSink& sink) mutable {
          while (running && sink.is_writable()) {
            std::vector<std::uint8_t> jpeg;
            double stamp;
            if (cached(front,jpeg,stamp) && stamp!=last) {
              last=stamp; delivered=seconds();
              const auto header=std::string("--auvframe\r\nContent-Type: image/jpeg\r\nContent-Length: ")+
                std::to_string(jpeg.size())+"\r\nX-Frame-Time-Monotonic: "+std::to_string(stamp)+
                "\r\nX-Camera-Source: "+(front ? cfg_.front_source : cfg_.camera)+"\r\n\r\n";
              std::string part;
              part.reserve(header.size()+jpeg.size()+2);
              part.append(header);
              part.append(reinterpret_cast<const char*>(jpeg.data()),jpeg.size());
              part.append("\r\n");
              return sink.write(part.data(),part.size());
            }
            if (seconds()-delivered>2.0) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
          }
          return false;
        },[this](bool) { --mjpeg_clients_; });
    };
    server.Get("/api/camera/down.mjpeg",[stream](const httplib::Request&,httplib::Response& res){stream(false,res);});
    server.Get("/api/camera/front.mjpeg",[stream](const httplib::Request&,httplib::Response& res){stream(true,res);});
    server.Get("/",[this](const httplib::Request&,httplib::Response& res){
      std::ifstream in(cfg_.web_assets+"/index.html");
      if (!in) { res.status=503; return; }
      res.set_content(std::string(std::istreambuf_iterator<char>(in),{}),"text/html; charset=utf-8");
    });
    server.Get("/hls.min.js",[this](const httplib::Request&,httplib::Response& res){
      std::ifstream in(cfg_.web_assets+"/hls.min.js",std::ios::binary);
      if (!in) { res.status=503; return; }
      res.set_content(std::string(std::istreambuf_iterator<char>(in),{}),"application/javascript");
    });
    server.Get("/dashboard/config",[](const httplib::Request&,httplib::Response& res){
      res.set_content("{\"demo\":false,\"source\":\"auv\",\"camera_base\":\"\"}","application/json");
    });
    server.Get("/dashboard",[](const httplib::Request&,httplib::Response& res){res.set_redirect("/dashboard/");});
    server.set_mount_point("/dashboard",cfg_.web_assets+"/dashboard");
    server.set_mount_point("/hls",cfg_.video_dir);
    if (!server.listen(cfg_.web_bind, cfg_.web_port) && running) {
      std::lock_guard<std::mutex> lock(state_mutex_); web_detail_="HTTP bind failed";
    }
    web_server_=nullptr;
  }
#endif
  void socket_loop() {
    std::filesystem::create_directories(std::filesystem::path(cfg_.socket).parent_path());
    int fd = ::socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);
    if (fd < 0) throw std::runtime_error("control socket failed");
    sockaddr_un addr{}; addr.sun_family = AF_UNIX;
    if (cfg_.socket.size() >= sizeof(addr.sun_path)) throw std::runtime_error("control socket path too long");
    std::strncpy(addr.sun_path,cfg_.socket.c_str(),sizeof(addr.sun_path)-1);
    ::unlink(cfg_.socket.c_str());
    if (::bind(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr)) || ::listen(fd,8)) throw std::runtime_error("control socket bind failed");
    ::chmod(cfg_.socket.c_str(),0660);
    timeval timeout{0,200000}; ::setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
    while (running) {
      int peer = ::accept4(fd,nullptr,nullptr,SOCK_CLOEXEC);
      if (peer < 0) continue;
      char b[128]{}; const auto n = ::read(peer,b,sizeof(b)-1);
      if (n > 0) { std::string request(b,static_cast<std::size_t>(n));
        request.erase(request.find_last_not_of("\r\n ")+1);
        const auto answer = command(request);
        std::size_t offset=0;
        while (offset < answer.size()) {
          const auto written=::write(peer,answer.data()+offset,answer.size()-offset);
          if (written <= 0) break;
          offset+=static_cast<std::size_t>(written);
        }
      }
      ::close(peer);
    }
    ::close(fd); ::unlink(cfg_.socket.c_str());
  }
  Config cfg_;
  TagDockTask docking_;
  DockObservation dock_front_,dock_down_;
  cv::Mat dock_candidate_frame_,dock_evidence_frame_;
  DockObservation dock_evidence_observation_;
  std::string dock_last_phase_;
  double dock_last_report_time_{};
  bool dock_evidence_saved_{};
  auv_control::ObservationSearch search_;
  auv_control::ControlledAscent ascent_;
  auv_control::SurfaceRouteExecutor surface_route_;
  auv_control::CenterApproach center_approach_;
  auv_mapping::MetricGridPose metric_pose_;
  auv_stm32_bridge::DepthSampleTelemetry depth_sample_;
  double depth_sample_received_{0},surface_reacquire_start_{0},last_trajectory_time_{0};
  std::uint64_t surface_vote_sequence_{0},trajectory_sequence_{0};
  int surface_pose_votes_{0};
  int observed_cell_{-1};
  bool ascent_started_{false},surface_confirmed_{false},surface_ready_{false},plan_evidence_saved_{false},a2_completion_logged_{false};
  std::deque<std::string> trajectory_queue_;
  std::string plan_artifact_pending_,a2_detail_;
  std::unique_ptr<MissionRecorder> recorder_;
  std::string run_id_,run_dir_,recording_detail_,search_detail_;
  std::size_t search_waypoint_{0};
  std::atomic<double> recording_down_time_{0},recording_front_time_{0};
  std::atomic<std::uint64_t> perception_epoch_{0};
  std::uint64_t front_sequence_{0},log_segment_{0};
  double origin_absolute_yaw_{0},tag_time_{0};
  std::atomic<double> down_hz_{0};
  std::atomic<int> mjpeg_clients_{0};
  std::vector<std::uint8_t> down_jpeg_,front_jpeg_,down_raw_jpeg_,front_raw_jpeg_;
  std::mutex front_mutex_;
  cv::Mat front_frame_,down_preview_frame_,front_preview_frame_,down_rectified_frame_,front_rectified_frame_;
  bool down_surface_frame_{}; // guarded by frame_mutex_
  bool front_surface_frame_{}; // guarded by front_mutex_
  std::atomic<double> front_time_{0};
  std::atomic<std::uint64_t> down_capture_frames_{0};
  std::uint64_t front_frames_{};
  double front_hz_{};
  std::string front_detail_;
  std::mutex frame_mutex_,video_mutex_,state_mutex_,log_mutex_;
  std::condition_variable log_cv_;
  std::deque<std::string> log_queue_;
  std::atomic<bool> log_degraded_{false};
  std::atomic<bool> log_stop_{false};
  double log_started_{seconds()};
  cv::Mat frame_; std::atomic<double> frame_time_{0}; double pose_time_{},processed_time_{},status_time_{},gripper_time_{};
  cv::Mat video_frame_;
  std::uint64_t video_sequence_{};
  double vision_hz_{};
  std::array<double,256> latency_ms_{};
  std::size_t latency_index_{},latency_count_{};
  std::uint64_t control_ticks_{};
  std::atomic<std::uint64_t> heartbeat_count_{0};
  std::atomic<bool> map_image_saved_{false};
  double last_state_log_sec_{};
  std::uint64_t frame_sequence_{},processed_frames_{};
  auv_mission::MissionFsm mission_;
  auv_control::RouteExecutor route_;
  auv_planning::GridPlanner planner_;
  auv_core::SemanticMap map_;
  auv_core::Stm32Status status_;
  auv_stm32_bridge::GripperTelemetry gripper_;
  auv_planning::PlanResult plan_;
  std::array<bool,9> visited_{};
  auv_stm32_bridge::MotionTarget motion_{};
  auv_stm32_bridge::SerialPort serial_;
  std::uint32_t sequence_{},arm_sequence_{},gripper_command_sequence_{},map_revision_{};
  double arm_time_{};
  double boot_time_{seconds()};
  double startup_ready_since_{};
  double gripper_command_time_{};
  std::atomic<double> last_control_time_{seconds()};
  float hold_depth_{},hold_yaw_{};
  std::size_t waypoint_index_{};
  std::unique_ptr<Localization> localization_;
  bool tag_found_{},pose_valid_{},route_ready_{},all_visited_{},serial_connected_{},armed_requested_{},arm_ack_{},arm_pending_{},disarm_pending_{true};
  bool autonomous_start_attempted_{},autonomous_arm_attempted_{};
  bool gripper_ack_{},gripper_close_requested_{},gripper_open_requested_{},gripper_stop_requested_{};
  std::optional<auv_stm32_bridge::GripperAction> gripper_pending_;
  float row_{},col_{};
  std::string fault_,video_detail_,web_detail_;
  bool running_video_{true};
#ifdef AUV_HAVE_HTTPLIB
  std::atomic<httplib::Server*> web_server_{nullptr};
  std::atomic<bool> web_finished_{false};
#endif
  std::ofstream log_;
};
int main(int argc,char** argv) {
  try {
    if(argc==3 && std::string(argv[1])=="--check-config") {
      const auto c=load_config(argv[2]);
      std::cout<<"configuration valid; mode="<<c.operation_mode<<" profile="<<c.mission_profile
        <<" motion="<<c.motion_enabled<<" auto_arm="<<c.auto_arm<<" search="<<c.search.enabled<<'\n';return 0;
    }
    if (argc != 2) { std::cerr << "usage: auv_runtime [--check-config] CONFIG.yaml\n"; return 2; }
    // This process already has dedicated capture, vision and control threads.
    // Avoid OpenCV worker-pool oversubscription on the Raspberry Pi.
    cv::setNumThreads(1);
    Runtime(load_config(argv[1])).run(); return 0;
  } catch (const std::exception& e) { std::cerr << "auv_runtime: " << e.what() << '\n'; return 1; }
}
