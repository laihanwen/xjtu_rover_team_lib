#include "auv_core/semantic_map.hpp"
#include "auv_core/status_decoder.hpp"
#include "auv_control/route_executor.hpp"
#include "auv_mapping/grid_mapper.hpp"
#include "auv_mission/mission_fsm.hpp"
#include "auv_planning/grid_planner.hpp"
#include "auv_stm32_bridge/motion_target.hpp"
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
#include <sstream>
#include <thread>
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
struct Config {
  std::string camera, serial, socket, log, debug_dir;
  int camera_width{640}, camera_height{480};
  double camera_fps{30.0};
  std::string camera_pixel_format{"MJPG"}, apriltag_family{"tag36h11"};
  int baud{}, expected_cones{4};
  double status_timeout{0.5}, frame_timeout{0.5}, pose_timeout{0.5};
  bool motion_enabled{}, directions_calibrated{}, limits_calibrated{};
  auv_control::RouteExecutorConfig route;
  auv_mapping::GridMapperConfig grid;
  auv_vision::ConeDetectorConfig cone;
  auv_vision::ConeTrackerConfig tracker;
  auv_mission::MissionFsmConfig mission;
  auv_planning::GridPlannerConfig planner;
  bool video_enabled{}, software_fallback{};
  std::string video_dir,video_encoder,web_bind,web_assets;
  int web_port{},video_width{640},video_height{480},video_fps{20},video_bitrate_kbps{2000};
  double segment_time{0.5};
  std::uint64_t max_event_bytes{10*1024*1024};
  std::vector<double> camera_matrix, distortion;
};
static Config load_config(const std::string& path) {
  const auto y = YAML::LoadFile(path);
  Config c;
  c.camera = y["camera"]["source"].as<std::string>();
  c.camera_width = y["camera"]["width"].as<int>();
  c.camera_height = y["camera"]["height"].as<int>();
  c.camera_fps = y["camera"]["fps"].as<double>();
  c.camera_pixel_format = y["camera"]["pixel_format"].as<std::string>();
  c.serial = y["serial"]["device"].as<std::string>();
  c.baud = y["serial"]["baud"].as<int>();
  c.socket = y["control"]["socket"].as<std::string>();
  c.log = y["logging"]["events"].as<std::string>();
  c.debug_dir = y["logging"]["debug_dir"].as<std::string>();
  c.expected_cones = y["vision"]["expected_cones"].as<int>();
  c.apriltag_family = y["vision"]["apriltag_family"].as<std::string>();
  c.grid.stable_frames = y["vision"]["grid_stable_frames"].as<int>();
  c.cone.minimum_confidence = y["vision"]["cone_minimum_confidence"].as<double>();
  c.tracker.history_size = y["vision"]["cone_history_size"].as<int>();
  c.tracker.required_votes = y["vision"]["cone_required_votes"].as<int>();
  c.tracker.clear_votes = y["vision"]["cone_clear_votes"].as<int>();
  c.planner.maximum_targets = y["planning"]["maximum_targets"].as<std::size_t>();
  c.planner.target_object_types = y["planning"]["target_object_types"].as<std::vector<std::string>>();
  c.planner.blocked_object_types = y["planning"]["blocked_object_types"].as<std::vector<std::string>>();
  c.status_timeout = y["safety"]["status_timeout_sec"].as<double>();
  c.frame_timeout = y["safety"]["frame_timeout_sec"].as<double>();
  c.pose_timeout = y["safety"]["pose_timeout_sec"].as<double>();
  c.motion_enabled = y["motion"]["motion_commands_enabled"].as<bool>();
  c.directions_calibrated = y["motion"]["directions_calibrated"].as<bool>();
  c.limits_calibrated = y["motion"]["limits_calibrated"].as<bool>();
  c.route.surge_from_row = y["motion"]["surge_from_row"].as<double>();
  c.route.surge_from_col = y["motion"]["surge_from_col"].as<double>();
  c.route.sway_from_row = y["motion"]["sway_from_row"].as<double>();
  c.route.sway_from_col = y["motion"]["sway_from_col"].as<double>();
  c.route.maximum_speed = y["motion"]["maximum_speed"].as<double>();
  c.route.arrival_tolerance = y["motion"]["arrival_tolerance"].as<double>();
  c.route.arrival_stable_ticks = y["motion"]["arrival_stable_ticks"].as<int>();
  c.mission.self_check_timeout_sec = y["mission"]["self_check_timeout_sec"].as<double>();
  c.mission.apriltag_timeout_sec = y["mission"]["apriltag_timeout_sec"].as<double>();
  c.mission.map_timeout_sec = y["mission"]["map_timeout_sec"].as<double>();
  c.mission.planning_timeout_sec = y["mission"]["planning_timeout_sec"].as<double>();
  c.mission.cone_visit_timeout_sec = y["mission"]["cone_visit_timeout_sec"].as<double>();
  c.mission.status_timeout_sec = c.status_timeout;
  c.mission.allow_armed_during_visit = true;
  c.video_enabled = y["video"]["enabled"].as<bool>();
  c.video_dir = y["video"]["directory"].as<std::string>();
  c.video_encoder = y["video"]["encoder"].as<std::string>();
  c.video_width = y["video"]["width"].as<int>();
  c.video_height = y["video"]["height"].as<int>();
  c.video_fps = y["video"]["fps"].as<int>();
  c.video_bitrate_kbps = y["video"]["bitrate_kbps"].as<int>();
  c.segment_time = y["video"]["segment_time_sec"].as<double>();
  c.software_fallback = y["video"]["software_fallback_enabled"].as<bool>();
  c.web_bind = y["web"]["bind"].as<std::string>();
  c.web_port = y["web"]["port"].as<int>();
  c.web_assets = y["web"]["assets"].as<std::string>();
  c.max_event_bytes = y["logging"]["max_event_bytes"].as<std::uint64_t>();
  if (y["camera"]["camera_matrix"]) c.camera_matrix = y["camera"]["camera_matrix"].as<std::vector<double>>();
  if (y["camera"]["distortion_coefficients"]) c.distortion = y["camera"]["distortion_coefficients"].as<std::vector<double>>();
  if (c.camera.rfind("/dev/v4l/by-id/", 0) != 0 && c.camera.rfind("file:", 0) != 0)
    throw std::runtime_error("camera source must be /dev/v4l/by-id/... or file:...");
  if (c.camera_width <= 0 || c.camera_width > 1920 || c.camera_height <= 0 ||
      c.camera_height > 1080 || c.camera_fps <= 0 || c.camera_fps > 120 ||
      c.camera_pixel_format.size() != 4 || c.grid.stable_frames <= 0 ||
      c.cone.minimum_confidence <= 0 || c.cone.minimum_confidence > 1 ||
      c.baud <= 0 || c.expected_cones < 1 || c.expected_cones > 9 || c.status_timeout <= 0 ||
      c.frame_timeout <= 0 || c.pose_timeout <= 0 || c.socket.empty() ||
      c.log.empty() || c.debug_dir.empty())
    throw std::runtime_error("invalid runtime configuration");
  if (c.web_bind != "192.168.137.201" || c.web_port <= 0 || c.web_port > 65535 ||
      c.video_dir.empty() || c.web_assets.empty() ||
      c.video_width <= 0 || c.video_width > 1920 ||
      c.video_height <= 0 || c.video_height > 1080 ||
      c.video_fps <= 0 || c.video_fps > 60 ||
      c.video_bitrate_kbps <= 0 || c.segment_time <= 0 || c.max_event_bytes < 1024 ||
      (c.video_encoder != "h264_v4l2m2m" && c.video_encoder != "libx264"))
    throw std::runtime_error("invalid video or wired web configuration");
  if (c.motion_enabled && (c.serial.empty() || !c.directions_calibrated || !c.limits_calibrated ||
      c.route.maximum_speed <= 0 || c.route.maximum_speed > 0.2 ||
      c.camera_matrix.size() != 9 || c.distortion.empty() ||
      !std::isfinite(c.route.surge_from_row*c.route.sway_from_col-
        c.route.surge_from_col*c.route.sway_from_row) ||
      std::abs(c.route.surge_from_row*c.route.sway_from_col-
        c.route.surge_from_col*c.route.sway_from_row) < 1e-6))
    throw std::runtime_error("motion requires calibrated directions, limits and serial device");
  if (!c.camera_matrix.empty() && c.camera_matrix.size() != 9)
    throw std::runtime_error("camera_matrix must contain 9 values");
  if (!c.distortion.empty() && c.distortion.size() != 4 && c.distortion.size() != 5 &&
      c.distortion.size() != 8 && c.distortion.size() != 12 && c.distortion.size() != 14)
    throw std::runtime_error("distortion_coefficients has invalid size");
  for (auto value : c.camera_matrix) if (!std::isfinite(value)) throw std::runtime_error("camera_matrix contains NaN or infinity");
  for (auto value : c.distortion) if (!std::isfinite(value)) throw std::runtime_error("distortion contains NaN or infinity");
  if (!c.camera_matrix.empty() && (c.camera_matrix[0] <= 0 || c.camera_matrix[4] <= 0))
    throw std::runtime_error("camera focal lengths must be positive");
  return c;
}
class Runtime {
 public:
  explicit Runtime(Config cfg) : cfg_(std::move(cfg)), mission_(cfg_.mission),
    route_(cfg_.route), planner_(cfg_.planner) {
    (void)auv_vision::AprilTagDetector(cfg_.apriltag_family);
    (void)auv_mapping::GridMapper(cfg_.grid);
    (void)auv_vision::ConeDetector(cfg_.cone);
    (void)auv_vision::ConeTracker(cfg_.tracker);
    std::filesystem::create_directories(std::filesystem::path(cfg_.log).parent_path());
    log_.open(cfg_.log, std::ios::app);
    if (!log_) throw std::runtime_error("cannot open event log");
  }
  void run() {
    std::signal(SIGINT, stop_signal); std::signal(SIGTERM, stop_signal); std::signal(SIGPIPE,SIG_IGN);
    event("BOOT", "DISARM");
    std::thread logger(&Runtime::logger_loop, this);
    std::thread capture(&Runtime::capture_loop, this);
    std::thread vision(&Runtime::vision_loop, this);
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
    capture.join(); vision.join(); control.join(); serial.join(); video.join();
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
    line << "{\"timestamp_unix_ms\":" << unix_ms << ",\"steady_sec\":" << seconds()
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
          std::filesystem::rename(cfg_.log,cfg_.log+".1");
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
    return serial_connected_ && status_fresh(now) && !status_.leak_detected &&
      status_.error_flags == 0 && status_.telemetry_valid;
  }
  void capture_loop() {
    auto source_name = cfg_.camera.rfind("file:", 0) == 0 ? cfg_.camera.substr(5) : cfg_.camera;
    auv_vision::CameraSource camera({source_name,cfg_.camera_width,cfg_.camera_height,
      cfg_.camera_fps,cfg_.camera_pixel_format,false});
    while (running) {
      try {
      if (!camera.is_open() && !camera.open()) { std::this_thread::sleep_for(std::chrono::seconds(1)); continue; }
      cv::Mat image;
      if (!camera.read(image)) { camera.close(); std::this_thread::sleep_for(std::chrono::milliseconds(200)); continue; }
      std::lock_guard<std::mutex> lock(frame_mutex_);
      frame_ = image;
    frame_time_ = seconds();
      ++frame_sequence_;
      } catch (const std::exception& e) {
        camera.close(); fault(std::string("camera: ")+e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }
  }
  void vision_loop() {
    auv_vision::AprilTagDetector tags(cfg_.apriltag_family);
    auv_mapping::GridMapper mapper(cfg_.grid);
    auv_vision::ConeDetector detector(cfg_.cone);
    auv_vision::ConeTracker tracker(cfg_.tracker);
    cv::Mat intrinsics, distortion;
    if (!cfg_.camera_matrix.empty()) {
      intrinsics=cv::Mat(3,3,CV_64F,cfg_.camera_matrix.data()).clone();
      distortion=cv::Mat(cfg_.distortion).clone().reshape(1,1);
    }
    std::uint64_t seen = 0;
    while (running) {
      cv::Mat image; double stamp = 0;
      { std::lock_guard<std::mutex> lock(frame_mutex_);
        if (seen != frame_sequence_) { seen = frame_sequence_; image = frame_; stamp = frame_time_; } }
      if (image.empty()) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }
      try {
      if (!intrinsics.empty()) {
        cv::Mat corrected;
        cv::undistort(image,corrected,intrinsics,distortion);
        image=std::move(corrected);
      }
      const auto found = tags.detect(image);
      const auto grid = mapper.process(image);
      std::vector<auv_vision::ConeObservation> cones;
      if (grid.stable && !grid.rectified.empty()) cones = tracker.update(detector.process(grid.rectified).observations);
      std::array<bool,9> visited;
      { std::lock_guard<std::mutex> lock(state_mutex_); visited = visited_; }
      const auto map = auv_core::fuse_semantic_map(grid, cones, tracker.ready(), visited, cfg_.expected_cones);
      if (map.complete && !map_image_saved_.exchange(true)) {
        try {
          std::filesystem::create_directories(cfg_.debug_dir);
          const auto filename=cfg_.debug_dir+"/map_"+std::to_string(static_cast<std::uint64_t>(seconds()*1000))+".jpg";
          if (!cv::imwrite(filename,grid.debug_image.empty() ? image : grid.debug_image))
            log_degraded_=true;
          else event("MAP_IMAGE",filename);
        } catch (...) { log_degraded_=true; }
      }
      {
        std::lock_guard<std::mutex> lock(video_mutex_);
        video_frame_ = grid.debug_image.empty() ? image : grid.debug_image;
        ++video_sequence_;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      tag_found_ = tag_found_ || !found.empty();
      map_ = map;
      pose_valid_ = grid.stable && grid.position_valid && std::isfinite(grid.camera_row) && std::isfinite(grid.camera_col) &&
        grid.camera_row >= 0 && grid.camera_row <= 3 && grid.camera_col >= 0 && grid.camera_col <= 3;
      row_ = grid.camera_row; col_ = grid.camera_col; pose_time_ = stamp;
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
          } else if (frame.message_type == AUV_PROTOCOL_MSG_ACK && frame.payload.size() == 6) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (auv_protocol_read_u32_le(frame.payload.data()+2) == arm_sequence_ &&
                frame.payload[0] == AUV_PROTOCOL_MSG_SET_ARMED) {
              arm_ack_ = frame.payload[1] == 0;
              if (!arm_ack_) { armed_requested_ = false; disarm_pending_ = true; set_fault_locked("STM32 rejected ARM"); }
            }
          }
        }
        const auto now = seconds();
        auv_stm32_bridge::MotionTarget motion, neutral;
        bool send_motion = false, send_disarm = false, send_neutral = false,
          send_arm_request = false, reconnect = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
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
          send_motion = cfg_.motion_enabled && armed_requested_ && arm_ack_ && status_.armed && safe_status(now) &&
            mission_.snapshot().phase == auv_mission::MissionPhase::kVisitCones && fault_.empty();
          motion = send_motion ? motion_ : auv_stm32_bridge::MotionTarget{};
        }
        std::array<std::uint8_t,8> heartbeat{};
        auv_protocol_write_u32_le(heartbeat.data(),++sequence_);
        auv_protocol_write_u32_le(heartbeat.data()+4,static_cast<std::uint32_t>(now*1000));
        send_frame(AUV_PROTOCOL_MSG_HEARTBEAT,heartbeat.data(),heartbeat.size());
        ++heartbeat_count_;
        if (send_disarm) {
          if (send_neutral) {
            const auto p=auv_stm32_bridge::encode_motion_target_payload(++sequence_,neutral);
            send_frame(AUV_PROTOCOL_MSG_MOTION_TARGET,p.data(),p.size());
          }
          send_arm(false);
        }
        if (send_arm_request) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (armed_requested_ && fault_.empty()) { arm_sequence_ = sequence_ + 1; arm_time_=seconds(); send_arm(true); }
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
        auto phase = mission_.snapshot().phase;
        if (phase != auv_mission::MissionPhase::kInit && phase != auv_mission::MissionPhase::kComplete &&
            phase != auv_mission::MissionPhase::kFault && phase != auv_mission::MissionPhase::kAborted) {
          if (!safe_status(now)) {
            set_fault_locked("STM32 unsafe or STATUS timeout"); armed_requested_ = false; disarm_pending_ = true;
          }
          if (!fault_.empty()) { armed_requested_ = false; disarm_pending_ = true; }
          if (frame_time_ <= 0 || now - frame_time_ > cfg_.frame_timeout) {
            set_fault_locked("camera frame timeout"); armed_requested_ = false; disarm_pending_ = true;
          }
          if (phase == auv_mission::MissionPhase::kVisitCones && (!pose_valid_ || now - pose_time_ > cfg_.pose_timeout)) {
            set_fault_locked("grid pose timeout"); armed_requested_ = false; disarm_pending_ = true;
          }
        }
        if (status_fresh(now)) mission_.update_status(serial_connected_,status_.armed,status_.leak_detected,status_.error_flags,now);
        mission_.update_apriltag(tag_found_,now);
        mission_.update_map(map_.complete, all_visited_,now);
        if (phase == auv_mission::MissionPhase::kPlanCones && !route_ready_ && map_.complete &&
            (!pose_valid_ || now-pose_time_ > cfg_.pose_timeout)) {
          set_fault_locked("grid pose unavailable for planning");
          armed_requested_=false; disarm_pending_=true;
        }
        if (phase == auv_mission::MissionPhase::kPlanCones && !route_ready_ && map_.complete && fault_.empty()) {
          auv_planning::GridCell start{static_cast<std::int8_t>(std::clamp(static_cast<int>(row_),0,2)),
            static_cast<std::int8_t>(std::clamp(static_cast<int>(col_),0,2)),"unknown"};
          plan_ = planner_.plan(map_.grid,start);
          route_ready_ = plan_.valid && !plan_.targets.empty();
          if (route_ready_) route_.set_route(plan_,++map_revision_);
          else { set_fault_locked("empty or invalid cone route: " + plan_.reason); disarm_pending_ = true; }
          std::ostringstream route_detail;
          route_detail << plan_.reason << " path=";
          for (const auto& cell : plan_.path)
            route_detail << '(' << static_cast<int>(cell.row) << ',' << static_cast<int>(cell.col) << ')';
          event("PLAN",route_detail.str());
        }
        mission_.update_route(route_ready_,route_ready_,now);
        if (fault_.empty()) mission_.tick(now);
        else mission_.force_fault(fault_,now);
        phase = mission_.snapshot().phase;
        if (!fault_.empty() || phase == auv_mission::MissionPhase::kFault) {
          armed_requested_ = false; disarm_pending_ = true; motion_ = {};
        } else {
          route_.set_mission_active(phase == auv_mission::MissionPhase::kVisitCones);
          route_.set_vehicle_ready(cfg_.motion_enabled && armed_requested_ && arm_ack_ && status_.armed && safe_status(now));
          route_.set_pose(pose_valid_ && now-pose_time_ <= cfg_.pose_timeout,row_,col_);
          const auto step = route_.step();
          waypoint_index_ = step.waypoint_index;
          if (phase == auv_mission::MissionPhase::kVisitCones && step.state == auv_control::RouteStep::State::kFault)
            { set_fault_locked(step.detail); armed_requested_ = false; disarm_pending_ = true; }
          if (step.visited_cell) {
            auto i = static_cast<std::size_t>(step.visited_cell->row*3+step.visited_cell->col);
            visited_[i] = true; event("CONE_VISITED",std::to_string(i));
          }
          all_visited_ = !plan_.targets.empty();
          for (const auto& t : plan_.targets) all_visited_ &= visited_[static_cast<std::size_t>(t.row*3+t.col)];
          motion_ = {};
          if (step.state == auv_control::RouteStep::State::kRunning && armed_requested_ && fault_.empty()) {
            motion_.vx = static_cast<float>(step.surge); motion_.vy = static_cast<float>(step.sway);
            motion_.depth = hold_depth_; motion_.yaw = hold_yaw_;
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
        if (snapshot.phase == auv_mission::MissionPhase::kComplete) { armed_requested_ = false; disarm_pending_ = true; }
      }
      std::this_thread::sleep_until(next);
    }
  }
  std::string command(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = seconds();
    if (cmd == "status") {
      std::ostringstream s;
      auto latency=std::vector<double>(latency_ms_.begin(),latency_ms_.begin()+latency_count_);
      std::sort(latency.begin(),latency.end());
      double p99=latency.empty() ? -1.0 : latency[static_cast<std::size_t>(0.99*(latency.size()-1))];
      s << "{\"phase\":\"" << auv_mission::mission_phase_name(mission_.snapshot().phase)
        << "\",\"serial\":" << (serial_connected_ ? "true":"false")
        << ",\"armed\":" << (status_.armed ? "true":"false")
        << ",\"motion_enabled\":" << (cfg_.motion_enabled ? "true":"false")
        << ",\"leak\":" << (status_.leak_detected ? "true":"false")
        << ",\"telemetry_valid\":" << (status_.telemetry_valid ? "true":"false")
        << ",\"voltage_valid\":" << (status_.voltage_valid ? "true":"false")
        << ",\"camera_age_sec\":" << (frame_time_ ? now-frame_time_ : -1)
        << ",\"vision_frames\":" << processed_frames_
        << ",\"vision_hz\":" << vision_hz_
        << ",\"vision_latency_p99_ms\":" << p99
        << ",\"control_ticks\":" << control_ticks_
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
    if (cmd == "disarm" || cmd == "abort" || cmd == "pause" || cmd == "reset") {
      armed_requested_ = false; arm_ack_ = false; arm_pending_ = false;
      arm_time_=0;
      disarm_pending_ = true; motion_ = {};
    }
    if (cmd == "arm SAFE_TO_ARM") {
      if (!cfg_.motion_enabled || !cfg_.directions_calibrated || !cfg_.limits_calibrated || cfg_.serial.empty()) return "ERR motion configuration disabled or uncalibrated\n";
      if (!fault_.empty() || !safe_status(now) || status_.armed ||
          !pose_valid_ || now-pose_time_ > cfg_.pose_timeout ||
          now-frame_time_ > cfg_.frame_timeout || !map_.complete || !route_ready_ ||
          mission_.snapshot().phase != auv_mission::MissionPhase::kVisitCones)
        return "ERR ARM safety gate rejected\n";
      hold_depth_=status_.depth; hold_yaw_=status_.yaw;
      armed_requested_ = true; arm_ack_ = false; arm_pending_ = true; arm_time_=0;
      event("ARM_REQUEST", "operator"); return "OK ARM requested\n";
    }
    if (cmd == "arm") return "ERR use arm --confirm SAFE_TO_ARM\n";
    if (cmd == "disarm") { event("DISARM", "operator"); return "OK DISARM requested\n"; }
    std::optional<auv_mission::MissionCommand> c;
    if (cmd == "start") c = auv_mission::MissionCommand::kStart;
    if (cmd == "pause") c = auv_mission::MissionCommand::kPause;
    if (cmd == "resume") c = auv_mission::MissionCommand::kResume;
    if (cmd == "abort") c = auv_mission::MissionCommand::kAbort;
    if (cmd == "reset") c = auv_mission::MissionCommand::kReset;
    if (!c) return "ERR unknown command\n";
    auto r = mission_.command(*c,now);
    if (cmd == "reset" && r.accepted) {
      fault_.clear(); route_.reset(); route_ready_=false; plan_={}; map_={};
      visited_.fill(false); all_visited_=false; tag_found_=false;
      pose_valid_=false; pose_time_=0; waypoint_index_=0;
      map_image_saved_=false;
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
    pid_t child = ::fork();
    if (child == 0) {
      ::dup2(pipefd[0],STDIN_FILENO); ::close(pipefd[0]); ::close(pipefd[1]);
      std::string output=cfg_.video_dir+"/index.m3u8";
      const std::string size=std::to_string(cfg_.video_width)+"x"+std::to_string(cfg_.video_height);
      const std::string fps=std::to_string(cfg_.video_fps);
      const std::string bitrate=std::to_string(cfg_.video_bitrate_kbps)+"k";
      const std::string gop=std::to_string(std::max(1,static_cast<int>(cfg_.video_fps*cfg_.segment_time)));
      const std::string segment=std::to_string(cfg_.segment_time);
      ::execlp("ffmpeg","ffmpeg","-hide_banner","-loglevel","error","-nostdin","-y",
        "-f","rawvideo","-pixel_format","bgr24","-video_size",size.c_str(),"-framerate",fps.c_str(),
        "-i","pipe:0","-an","-c:v",cfg_.video_encoder.c_str(),"-b:v",bitrate.c_str(),
        "-g",gop.c_str(),"-keyint_min",gop.c_str(),"-sc_threshold","0","-f","hls",
        "-hls_time",segment.c_str(),"-hls_list_size","6","-hls_flags","delete_segments+independent_segments",
        output.c_str(),static_cast<char*>(nullptr));
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
      { std::lock_guard<std::mutex> lock(video_mutex_);
        if (seen != video_sequence_) { seen=video_sequence_; image=video_frame_; } }
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
    if (!running) return;
    httplib::Server server;
    web_server_=&server;
    server.Get("/api/status",[this](const httplib::Request&,httplib::Response& res){
      res.set_content(command("status"),"application/json");
      res.set_header("Cache-Control","no-store");
    });
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
        const auto answer = command(request); ::write(peer,answer.data(),answer.size()); }
      ::close(peer);
    }
    ::close(fd); ::unlink(cfg_.socket.c_str());
  }
  Config cfg_;
  std::mutex frame_mutex_,video_mutex_,state_mutex_,log_mutex_;
  std::condition_variable log_cv_;
  std::deque<std::string> log_queue_;
  std::atomic<bool> log_degraded_{false};
  std::atomic<bool> log_stop_{false};
  double log_started_{seconds()};
  cv::Mat frame_; std::atomic<double> frame_time_{0}; double pose_time_{},processed_time_{},status_time_{};
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
  auv_planning::PlanResult plan_;
  std::array<bool,9> visited_{};
  auv_stm32_bridge::MotionTarget motion_{};
  auv_stm32_bridge::SerialPort serial_;
  std::uint32_t sequence_{},arm_sequence_{},map_revision_{};
  double arm_time_{};
  float hold_depth_{},hold_yaw_{};
  std::size_t waypoint_index_{};
  bool tag_found_{},pose_valid_{},route_ready_{},all_visited_{},serial_connected_{},armed_requested_{},arm_ack_{},arm_pending_{},disarm_pending_{true};
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
  try { if (argc != 2) { std::cerr << "usage: auv_runtime CONFIG.yaml\n"; return 2; }
    Runtime(load_config(argv[1])).run(); return 0;
  } catch (const std::exception& e) { std::cerr << "auv_runtime: " << e.what() << '\n'; return 1; }
}
