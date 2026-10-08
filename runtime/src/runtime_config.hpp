#pragma once
#include "localization.hpp"
#include "auv_control/route_executor.hpp"
#include "auv_control/surface_traversal.hpp"
#include "auv_mapping/grid_mapper.hpp"
#include "auv_mission/mission_fsm.hpp"
#include "auv_planning/grid_planner.hpp"
#include "auv_vision/cone_detector.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <stdexcept>
struct Config {
  auv_control::SurfaceTraversalConfig traversal;
  auv_control::ObservationSearchConfig search;
  bool auto_origin{}, recording_enabled{}, recording_required{};
  std::string recording_dir{"/var/log/auv-runtime/runs"}, config_source;
  double recording_segment{60};
  std::uint64_t recording_reserve{256*1024*1024};
  std::vector<int> apriltag_ids;
  int tag_stable_frames{3};
  Localization::Settings localization;
  std::string camera, serial, socket, log, debug_dir;
  bool front_enabled{};
  std::string front_source{"/dev/v4l/by-id/REPLACE_WITH_FRONT_USB_CAMERA"};
  int front_width{320},front_height{240},front_fps{30};
  std::string operation_mode{"debug"};
  bool auto_start{}, auto_arm{};
  double startup_delay{5.0}, startup_timeout{30.0}, startup_stable{1.0};
  int camera_width{640}, camera_height{480};
  double camera_fps{30.0};
  std::string camera_pixel_format{"MJPG"}, apriltag_family{"tag36h11"};
  int baud{}, expected_cones{4};
  double status_timeout{0.5}, frame_timeout{0.5}, pose_timeout{0.5}, control_watchdog_timeout{0.25};
  double actuator_status_timeout{0.5};
  bool motion_enabled{}, directions_calibrated{}, limits_calibrated{};
  auv_control::RouteExecutorConfig route;
  auv_mapping::GridMapperConfig grid;
  auv_vision::ConeDetectorConfig cone;
  auv_vision::ConeTrackerConfig tracker;
  auv_mission::MissionFsmConfig mission;
  std::string mission_profile{"task_one"};
  auv_planning::GridPlannerConfig planner;
  bool video_enabled{}, software_fallback{};
  bool web_enabled{true};
  std::string video_dir,video_encoder,web_bind,web_assets;
  int web_port{},video_width{640},video_height{480},video_fps{20},video_bitrate_kbps{2000};
  double segment_time{0.5};
  std::uint64_t max_event_bytes{10*1024*1024};
  std::vector<double> camera_matrix, distortion;
};
static Config load_config(const std::string& path) {
  const auto y = YAML::LoadFile(path);
  Config c;
  c.config_source=path;
  if(const auto s=y["observation_search"]) {
    c.search.enabled=s["enabled"].as<bool>(false);
    c.search.calibrated=s["corridor_calibrated"].as<bool>(false);
    c.search.maximum_radius_m=s["maximum_radius_m"].as<double>(0);
    c.search.maximum_speed=s["maximum_speed"].as<double>(.1);
    c.search.gain=s["position_gain"].as<double>(.4);
    c.search.tolerance_m=s["arrival_tolerance_m"].as<double>(.1);
    c.search.stable_sec=s["arrival_stable_sec"].as<double>(.5);
    c.search.pose_timeout_sec=s["pose_timeout_sec"].as<double>(.3);
    c.search.waypoint_timeout_sec=s["waypoint_timeout_sec"].as<double>(20);
    c.search.depth_m=s["depth_target_m"].as<double>(0);
    c.search.yaw_rad=s["yaw_relative_rad"].as<double>(0);
    if(s["waypoints_m"]) for(const auto& p:s["waypoints_m"]) {
      const auto v=p.as<std::vector<double>>();
      if(v.size()!=2)throw std::runtime_error("search waypoint must have x,y");
      c.search.waypoints.push_back({v[0],v[1]});
    }
  }
  c.search.validate();
  c.auto_origin=y["operation"]["auto_origin"].as<bool>(false);
  if(const auto r=y["recording"]) {
    c.recording_enabled=r["enabled"].as<bool>(false);
    c.recording_required=r["required_for_motion"].as<bool>(false);
    c.recording_dir=r["directory"].as<std::string>(c.recording_dir);
    c.recording_segment=r["segment_sec"].as<double>(60);
    c.recording_reserve=r["minimum_free_bytes"].as<std::uint64_t>(c.recording_reserve);
  }
  c.localization=Localization::config(y["localization"]);
  if(const auto t=y["surface_traversal"]) {
    auto& a=c.traversal;
    a.enabled=t["enabled"].as<bool>(false);
    a.ascent_verified=t["ascent_clearance_verified"].as<bool>(false);
    a.surface_verified=t["surface_localization_verified"].as<bool>(false);
    a.center_approach_verified=t["center_approach_verified"].as<bool>(false);
    a.center_approach_radius_cells=t["center_approach_radius_cells"].as<double>(0);
    a.center_stable_sec=t["center_stable_sec"].as<double>(0.3);
    a.center_timeout_sec=t["center_timeout_sec"].as<double>(10);
    const auto cell=t["ascent_cell"].as<std::vector<int>>(std::vector<int>{-1,-1});
    if(cell.size()!=2)throw std::runtime_error("ascent_cell requires row,col");
    a.ascent_row=cell[0];a.ascent_col=cell[1];
    a.ascent_tolerance_cells=t["ascent_tolerance_cells"].as<double>(a.ascent_tolerance_cells);
    a.surface_depth_m=t["depth_target_m"].as<double>(a.surface_depth_m);
    a.depth_tolerance_m=t["depth_tolerance_m"].as<double>(a.depth_tolerance_m);
    a.ascent_rate_mps=t["ascent_rate_mps"].as<double>(a.ascent_rate_mps);
    a.surface_stable_sec=t["surface_stable_sec"].as<double>(a.surface_stable_sec);
    a.reacquire_frames=t["reacquire_frames"].as<int>(a.reacquire_frames);
    a.surface_depth_samples=t["surface_depth_samples"].as<int>(a.surface_depth_samples);
    a.depth_sample_timeout_sec=t["depth_sample_timeout_sec"].as<double>(a.depth_sample_timeout_sec);
    a.reacquire_timeout_sec=t["reacquire_timeout_sec"].as<double>(a.reacquire_timeout_sec);
    a.pose_timeout_sec=t["pose_timeout_sec"].as<double>(a.pose_timeout_sec);
    a.maximum_speed=t["maximum_speed"].as<double>(a.maximum_speed);
    a.gain=t["position_gain"].as<double>(a.gain);
    a.arrival_tolerance_cells=t["arrival_tolerance_cells"].as<double>(a.arrival_tolerance_cells);
    a.arrival_stable_sec=t["arrival_stable_sec"].as<double>(a.arrival_stable_sec);
    a.entry_margin_cells=t["entry_margin_cells"].as<double>(a.entry_margin_cells);
    a.boundary_clearance_m=t["boundary_clearance_m"].as<double>(a.boundary_clearance_m);
    a.corridor_tolerance_cells=t["corridor_tolerance_cells"].as<double>(a.corridor_tolerance_cells);
    a.pose_jump_tolerance_m=t["pose_jump_tolerance_m"].as<double>(a.pose_jump_tolerance_m);
    a.waypoint_timeout_sec=t["waypoint_timeout_sec"].as<double>(a.waypoint_timeout_sec);
    a.underwater.verified=c.localization.plane.verified;
    a.underwater.width=c.localization.width;a.underwater.height=c.localization.height;
    a.underwater.camera_matrix=c.localization.intrinsics;a.underwater.distortion=c.localization.distortion;
    a.underwater.camera_to_body=c.localization.rotation;a.underwater.camera_offset_m=c.localization.camera_offset;
    a.underwater.depth_offset_m=c.localization.depth_offset;
    a.underwater.cell_size_m=t["cell_size_m"].as<double>(0);
    a.underwater.pool_depth_m=c.localization.plane.pool_depth;
    a.underwater.height_tolerance_m=t["height_tolerance_m"].as<double>(0.15);
    a.underwater.max_reprojection_px=t["maximum_reprojection_px"].as<double>(3);
    a.underwater.minimum_confidence=t["minimum_grid_confidence"].as<double>(0.8);
    a.surface=a.underwater;a.surface.verified=a.surface_verified;
    a.surface.width=t["width"].as<int>(0);a.surface.height=t["height"].as<int>(0);
    a.surface.camera_matrix=t["camera_matrix"].as<std::vector<double>>(std::vector<double>{});
    a.surface.distortion=t["distortion_coefficients"].as<std::vector<double>>(std::vector<double>{});
  }
  c.traversal.validate();
  c.camera = y["camera"]["source"].as<std::string>();
  c.camera_width = y["camera"]["width"].as<int>();
  c.camera_height = y["camera"]["height"].as<int>();
  c.camera_fps = y["camera"]["fps"].as<double>();
  c.camera_pixel_format = y["camera"]["pixel_format"].as<std::string>();
  if (const auto front=y["camera_front"]) {
    c.front_enabled=front["enabled"].as<bool>(false);
    c.front_source=front["source"].as<std::string>("/dev/v4l/by-id/REPLACE_WITH_FRONT_USB_CAMERA");
    c.front_width=front["width"].as<int>(320);
    c.front_height=front["height"].as<int>(240);
    c.front_fps=front["fps"].as<int>(30);
  }
  if (c.front_enabled && (c.front_width<16 || c.front_width>1920 ||
      c.front_height<16 || c.front_height>1080 || c.front_fps<1 || c.front_fps>60 ||
      (c.front_source!="csi:0" && c.front_source!="csi:1" &&
       c.front_source.rfind("file:",0)!=0 && c.front_source.rfind("/dev/v4l/by-id/",0)!=0) ||
      c.front_source==c.camera))
    throw std::runtime_error("invalid or duplicate front camera configuration");
  c.serial = y["serial"]["device"].as<std::string>();
  c.baud = y["serial"]["baud"].as<int>();
  c.socket = y["control"]["socket"].as<std::string>();
  c.operation_mode = y["operation"]["mode"].as<std::string>();
  c.auto_start = y["operation"]["auto_start"].as<bool>();
  c.auto_arm = y["operation"]["auto_arm"].as<bool>();
  c.startup_delay = y["operation"]["startup_delay_sec"].as<double>();
  c.startup_timeout = y["operation"]["startup_timeout_sec"].as<double>();
  c.startup_stable = y["operation"]["startup_stable_sec"].as<double>();
  c.log = y["logging"]["events"].as<std::string>();
  c.debug_dir = y["logging"]["debug_dir"].as<std::string>();
  c.expected_cones = y["vision"]["expected_cones"].as<int>();
  c.apriltag_family = y["vision"]["apriltag_family"].as<std::string>();
  c.tag_stable_frames=y["vision"]["apriltag_stable_frames"].as<int>(3);
  if(y["vision"]["apriltag_ids"])c.apriltag_ids=y["vision"]["apriltag_ids"].as<std::vector<int>>();
  if(c.tag_stable_frames<=0)throw std::runtime_error("invalid tag confirmation count");
  c.grid.single_yellow_edge=y["vision"]["single_yellow_edge"].as<bool>(false);
  c.grid.dark_value_max=y["vision"]["grid_dark_value_max"].as<int>(95);
  c.grid.yellow_edge_minimum_support=y["vision"]["yellow_edge_minimum_support"].as<double>(.45);
  c.grid.yellow_edge_margin=y["vision"]["yellow_edge_margin"].as<double>(.2);
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
  c.control_watchdog_timeout = y["safety"]["control_watchdog_timeout_sec"].as<double>();
  c.actuator_status_timeout = y["safety"]["actuator_status_timeout_sec"].as<double>();
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
  c.mission_profile = y["mission"]["profile"].as<std::string>();
  c.mission.full_mission = c.mission_profile == "full";
  c.mission.stop_after_map = c.mission_profile == "a1_observation";
  c.mission.surface_before_visit = c.mission_profile == "a2_task_one";
  c.mission.surface_relocalize_timeout_sec=c.traversal.reacquire_timeout_sec;
  c.planner.forbid_target_reentry=c.mission.surface_before_visit;
  c.mission.allow_armed_during_observation=c.search.enabled;
  c.mission.cucumber_search_timeout_sec = y["mission"]["cucumber_search_timeout_sec"].as<double>();
  c.mission.cucumber_align_timeout_sec = y["mission"]["cucumber_align_timeout_sec"].as<double>();
  c.mission.gripper_timeout_sec = y["mission"]["gripper_timeout_sec"].as<double>();
  c.mission.transport_timeout_sec = y["mission"]["transport_timeout_sec"].as<double>();
  c.mission.valve_search_timeout_sec = y["mission"]["valve_search_timeout_sec"].as<double>();
  c.mission.valve_align_timeout_sec = y["mission"]["valve_align_timeout_sec"].as<double>();
  c.mission.valve_rotate_timeout_sec = y["mission"]["valve_rotate_timeout_sec"].as<double>();
  c.mission.return_home_timeout_sec = y["mission"]["return_home_timeout_sec"].as<double>();
  c.mission.surface_timeout_sec = y["mission"]["surface_timeout_sec"].as<double>();
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
  c.web_enabled=y["web"]["enabled"].as<bool>(true);
  c.web_port = y["web"]["port"].as<int>();
  c.web_assets = y["web"]["assets"].as<std::string>();
  c.max_event_bytes = y["logging"]["max_event_bytes"].as<std::uint64_t>();
  if (y["camera"]["camera_matrix"]) c.camera_matrix = y["camera"]["camera_matrix"].as<std::vector<double>>();
  if (y["camera"]["distortion_coefficients"]) c.distortion = y["camera"]["distortion_coefficients"].as<std::vector<double>>();
  if (c.camera != "csi:0" && c.camera != "csi:1" &&
      c.camera.rfind("/dev/v4l/by-id/", 0) != 0 && c.camera.rfind("file:", 0) != 0)
    throw std::runtime_error("camera source must be csi:0/1, /dev/v4l/by-id/... or file:...");
  for(double value : {c.camera_fps,c.startup_delay,c.startup_timeout,c.startup_stable,
      c.status_timeout,c.frame_timeout,c.pose_timeout,c.control_watchdog_timeout,c.actuator_status_timeout,
      c.route.maximum_speed,c.route.arrival_tolerance,c.route.surge_from_row,c.route.surge_from_col,
      c.route.sway_from_row,c.route.sway_from_col,c.mission.self_check_timeout_sec,
      c.mission.apriltag_timeout_sec,c.mission.map_timeout_sec,c.mission.planning_timeout_sec,
      c.mission.cone_visit_timeout_sec,c.mission.surface_timeout_sec,c.segment_time})
    if(!std::isfinite(value))throw std::runtime_error("nonfinite runtime parameter");
  if (c.camera_width <= 0 || c.camera_width > 1920 || c.camera_height <= 0 ||
      c.camera_height > 1080 || c.camera_fps <= 0 || c.camera_fps > 120 ||
      c.camera_pixel_format.size() != 4 || c.grid.stable_frames <= 0 ||
      c.cone.minimum_confidence <= 0 || c.cone.minimum_confidence > 1 ||
      c.baud <= 0 || c.expected_cones < 1 || c.expected_cones > 9 || c.status_timeout <= 0 ||
      c.frame_timeout <= 0 || c.pose_timeout <= 0 || c.control_watchdog_timeout <= 0 ||
      c.actuator_status_timeout <= 0 ||
      c.control_watchdog_timeout >= c.status_timeout || c.socket.empty() ||
      c.log.empty() || c.debug_dir.empty() || c.startup_delay < 0 ||
      c.startup_timeout <= c.startup_delay || c.startup_stable <= 0 ||
      c.startup_stable >= c.startup_timeout-c.startup_delay)
    throw std::runtime_error("invalid runtime configuration");
  if (c.operation_mode != "debug" && c.operation_mode != "autonomous")
    throw std::runtime_error("operation.mode must be debug or autonomous");
  if (c.mission_profile != "task_one" && c.mission_profile != "full" && c.mission_profile != "a1_observation" && c.mission_profile!="a2_task_one")
    throw std::runtime_error("invalid mission.profile");
  if(c.recording_enabled && (c.recording_dir.empty()||!std::isfinite(c.recording_segment)||
      c.recording_segment<=0||c.recording_reserve<1024))throw std::runtime_error("invalid recording settings");
  if(c.recording_required && !c.recording_enabled)throw std::runtime_error("required onboard recording is disabled");
  if(c.search.enabled && (!c.motion_enabled||!c.localization.enabled||!c.auto_origin||
      !c.recording_required || (!c.mission.stop_after_map&&!c.mission.surface_before_visit)))
    throw std::runtime_error("search requires motion, localization, auto_origin, recording and A1/A2 profile");
  if(c.traversal.enabled && (!c.mission.surface_before_visit||!c.search.enabled||!c.grid.single_yellow_edge||
      c.expected_cones!=4||!c.recording_required||!c.front_enabled||
      c.traversal.surface.width!=c.camera_width||c.traversal.surface.height!=c.camera_height||
      c.traversal.underwater.width!=c.camera_width||c.traversal.underwater.height!=c.camera_height||
      c.traversal.maximum_speed>c.route.maximum_speed||
      c.traversal.surface_depth_m>=c.search.depth_m||c.localization.plane.depth_zero!=0||
      c.camera_matrix!=c.localization.intrinsics||c.distortion!=c.localization.distortion))
    throw std::runtime_error("A2 requires complete A1, dual recording, matching underwater calibration and measured ascent/surface limits");
  if(c.traversal.enabled && (c.mission.surface_timeout_sec<=
      (c.search.depth_m-c.traversal.surface_depth_m)/c.traversal.ascent_rate_mps+c.traversal.surface_stable_sec+
      c.traversal.center_timeout_sec+c.traversal.surface_depth_samples*c.traversal.depth_sample_timeout_sec ||
      c.traversal.ascent_row!=1||c.traversal.ascent_col!=1))
    throw std::runtime_error("A2 requires center ascent column and sufficient ascent timeout");
  if(c.mission.surface_before_visit&&c.motion_enabled&&!c.traversal.enabled)
    throw std::runtime_error("A2 motion requires enabled calibrated surface traversal");
  if (c.operation_mode == "debug" && (c.auto_start || c.auto_arm))
    throw std::runtime_error("automatic operation is only valid in autonomous mode");
  if (c.operation_mode == "autonomous" && !c.auto_start)
    throw std::runtime_error("autonomous mode requires auto_start");
  if (c.auto_arm && !c.motion_enabled)
    throw std::runtime_error("auto_arm requires motion_commands_enabled");
  bool wired_web_address=false;
  for(const auto& prefix : {std::string("192.168.137."),std::string("192.168.50.")}) {
    if(c.web_bind.rfind(prefix,0)==0) {
      const auto suffix=c.web_bind.substr(prefix.size());
      if(!suffix.empty() && suffix.size()<=3 && std::all_of(suffix.begin(),suffix.end(),[](char x){return x>='0'&&x<='9';})) {
        const int host=std::stoi(suffix);wired_web_address=host>=2&&host<=254;
      }
    }
  }
  if (!wired_web_address || c.web_port <= 0 || c.web_port > 65535 ||
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
      (!c.search.enabled && (!std::isfinite(c.route.surge_from_row*c.route.sway_from_col-
        c.route.surge_from_col*c.route.sway_from_row) ||
      std::abs(c.route.surge_from_row*c.route.sway_from_col-
        c.route.surge_from_col*c.route.sway_from_row) < 1e-6))))
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
