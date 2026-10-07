// Copyright 2026 hanwen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "auv_interfaces/msg/semantic_cell.hpp"
#include "auv_interfaces/msg/cone_detection.hpp"
#include "auv_interfaces/msg/cone_detection_array.hpp"
#include "auv_interfaces/msg/grid_cell.hpp"
#include "auv_interfaces/msg/grid_pose.hpp"
#include "auv_interfaces/msg/semantic_map.hpp"
#include "auv_mapping/grid_mapper.hpp"
#include "auv_core/semantic_map.hpp"
#include "auv_vision/cone_detector.hpp"
#include "auv_mapping/semantic_mapper_node.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/calib3d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace auv_mapping
{

class SemanticMapperNode final : public rclcpp::Node
{
public:
  explicit SemanticMapperNode(const rclcpp::NodeOptions & options)
  : Node("auv_semantic_mapper", options)
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/down/image_raw");
    map_topic_ = declare_parameter<std::string>("map_topic", "/semantic_map");
    rectified_topic_ = declare_parameter<std::string>(
      "rectified_topic", "/mapping/rectified_image");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/mapping/debug_image");
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", false);
    require_calibration_ = declare_parameter<bool>("require_calibration", true);
    cone_detections_topic_ = declare_parameter<std::string>(
      "cone_detections_topic", "/cones/detections");
    cone_detection_timeout_sec_ = declare_parameter<double>(
      "cone_detection_timeout_sec", 1.0);
    grid_pose_topic_ = declare_parameter<std::string>(
      "grid_pose_topic", "/mapping/grid_pose");
    visited_cell_topic_ = declare_parameter<std::string>(
      "visited_cell_topic", "/planning/visited_cell");
    expected_cone_count_ = declare_parameter<int>("expected_cone_count", 4);
    if (cone_detection_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("cone_detection_timeout_sec must be positive");
    }
    if (grid_pose_topic_.empty() || visited_cell_topic_.empty() ||
      expected_cone_count_ < 0 || expected_cone_count_ > 9)
    {
      throw std::invalid_argument(
              "mapping pose/visited topics and expected_cone_count are invalid");
    }

    GridMapperConfig config;
    const auto hsv_lower = declare_parameter<std::vector<int64_t>>(
      "hsv_lower", std::vector<int64_t>{15, 60, 60});
    const auto hsv_upper = declare_parameter<std::vector<int64_t>>(
      "hsv_upper", std::vector<int64_t>{40, 255, 255});
    if (hsv_lower.size() != 3U || hsv_upper.size() != 3U) {
      throw std::invalid_argument("hsv_lower and hsv_upper must each contain three values");
    }
    for (std::size_t i = 0; i < 3U; ++i) {
      if (hsv_lower[i] < 0 || hsv_upper[i] > 255 || hsv_lower[i] > hsv_upper[i]) {
        throw std::invalid_argument("HSV thresholds are invalid");
      }
    }
    config.hsv_lower = cv::Scalar(hsv_lower[0], hsv_lower[1], hsv_lower[2]);
    config.hsv_upper = cv::Scalar(hsv_upper[0], hsv_upper[1], hsv_upper[2]);
    config.clahe_clip_limit = declare_parameter<double>("clahe_clip_limit", 2.0);
    config.morphology_kernel = declare_parameter<int>("morphology_kernel", 5);
    config.minimum_area_ratio = declare_parameter<double>("minimum_area_ratio", 0.15);
    config.maximum_area_ratio = declare_parameter<double>("maximum_area_ratio", 0.95);
    config.polygon_epsilon_ratio = declare_parameter<double>("polygon_epsilon_ratio", 0.03);
    config.minimum_corner_angle_degrees = declare_parameter<double>(
      "minimum_corner_angle_degrees", 20.0);
    config.maximum_corner_angle_degrees = declare_parameter<double>(
      "maximum_corner_angle_degrees", 160.0);
    config.output_size = declare_parameter<int>("output_size", 600);
    config.line_band_ratio = declare_parameter<double>("line_band_ratio", 0.035);
    config.minimum_line_support = declare_parameter<double>("minimum_line_support", 0.45);
    config.stable_frames = declare_parameter<int>("stable_frames", 3);
    config.yellow_oriented_frames = declare_parameter<int>("yellow_oriented_frames", 3);
    config.maximum_corner_jitter_ratio = declare_parameter<double>(
      "maximum_corner_jitter_ratio", 0.02);
    config.white_s_max = declare_parameter<double>("white_s_max", 60.0);
    config.white_v_min = declare_parameter<double>("white_v_min", 150.0);
    config.yellow_edge_min_coverage = declare_parameter<double>(
      "yellow_edge_min_coverage", 0.5);
    config.yellow_edge_margin = declare_parameter<double>("yellow_edge_margin", 0.3);
    mapper_ = std::make_unique<GridMapper>(config);

    configure_calibration();
    if (image_topic_.empty() || map_topic_.empty() || rectified_topic_.empty()) {
      throw std::invalid_argument("mapping topic names must not be empty");
    }

    map_publisher_ = create_publisher<auv_interfaces::msg::SemanticMap>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local());
    grid_pose_publisher_ = create_publisher<auv_interfaces::msg::GridPose>(
      grid_pose_topic_, rclcpp::SensorDataQoS());
    rectified_publisher_ = image_transport::create_publisher(
      *this, rectified_topic_, rclcpp::SensorDataQoS());
    if (publish_debug_image_) {
      debug_publisher_ = image_transport::create_publisher(
        *this, debug_topic_, rclcpp::SensorDataQoS());
    }
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {process_image(message);});
    cone_subscription_ = create_subscription<auv_interfaces::msg::ConeDetectionArray>(
      cone_detections_topic_, rclcpp::SensorDataQoS(),
      [this](auv_interfaces::msg::ConeDetectionArray::ConstSharedPtr message) {
        latest_cones_ = std::move(message);
        latest_cones_received_ = now();
      });
    visited_cell_subscription_ = create_subscription<auv_interfaces::msg::GridCell>(
      visited_cell_topic_, rclcpp::QoS(10).reliable(),
      [this](auv_interfaces::msg::GridCell::ConstSharedPtr message) {
        if (message->row < 0 || message->row >= 3 || message->col < 0 || message->col >= 3) {
          RCLCPP_WARN(get_logger(), "ignored out-of-range visited cell (%d,%d)",
            message->row, message->col);
          return;
        }
        visited_cells_[static_cast<std::size_t>(message->row * 3 + message->col)] = true;
      });

    RCLCPP_INFO(
      get_logger(), "semantic mapper ready: input=%s calibration=%s",
      image_topic_.c_str(), calibration_enabled_ ? "enabled" : "disabled");
  }

private:
  void configure_calibration()
  {
    const bool calibration_configured = declare_parameter<bool>(
      "calibration_configured", false);
    const auto intrinsics = declare_parameter<std::vector<double>>(
      "camera_matrix", std::vector<double>{});
    const auto distortion = declare_parameter<std::vector<double>>(
      "distortion_coefficients", std::vector<double>{});
    if (intrinsics.empty()) {
      if (calibration_configured) {
        throw std::invalid_argument(
                "calibration_configured is true but camera_matrix is empty");
      }
      if (!distortion.empty()) {
        throw std::invalid_argument("distortion_coefficients requires camera_matrix");
      }
      if (require_calibration_) {
        throw std::invalid_argument(
                "camera calibration is required; configure camera_matrix and "
                "distortion_coefficients or explicitly set require_calibration=false");
      }
      RCLCPP_WARN(
        get_logger(), "camera calibration disabled; do not use this mode for underwater mapping");
      return;
    }
    if (intrinsics.size() != 9U) {
      throw std::invalid_argument("camera_matrix must contain 9 row-major values");
    }
    if (distortion.empty() && require_calibration_) {
      throw std::invalid_argument(
              "distortion_coefficients must be configured for calibrated underwater mapping");
    }
    if (!distortion.empty() && distortion.size() != 4U && distortion.size() != 5U &&
      distortion.size() != 8U && distortion.size() != 12U && distortion.size() != 14U)
    {
      throw std::invalid_argument("distortion_coefficients must contain 4, 5, 8, 12, or 14 values");
    }
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (std::size_t i = 0; i < intrinsics.size(); ++i) {
      camera_matrix_.at<double>(static_cast<int>(i / 3U), static_cast<int>(i % 3U)) =
        intrinsics[i];
    }
    distortion_coefficients_ = cv::Mat(distortion).clone().reshape(1, 1);
    calibration_enabled_ = true;
  }

  void publish_map(
    const std_msgs::msg::Header & header, const GridResult & result)
  {
    std::vector<auv_vision::ConeObservation> observations;
    const bool fresh = latest_cones_ &&
      (now() - latest_cones_received_).seconds() <= cone_detection_timeout_sec_;
    if (fresh) {
      for (const auto & detection : latest_cones_->detections) {
        auv_vision::ConeObservation observation;
        observation.row = detection.row;
        observation.col = detection.col;
        observation.confidence = detection.detection.confidence;
        if (detection.shape == auv_interfaces::msg::ConeDetection::SHAPE_CIRCLE) {
          observation.shape = auv_vision::ConeShape::kCircle;
        } else if (detection.shape == auv_interfaces::msg::ConeDetection::SHAPE_SQUARE) {
          observation.shape = auv_vision::ConeShape::kSquare;
        }
        observations.push_back(observation);
      }
    }
    const auto fused = auv_core::fuse_semantic_map(
      result, observations, fresh && latest_cones_->stable,
      visited_cells_, expected_cone_count_);
    auv_interfaces::msg::SemanticMap map;
    map.header = header;
    map.rows = fused.grid.rows;
    map.cols = fused.grid.cols;
    map.complete = fused.complete;
    for (const auto & planning_cell : fused.grid.cells) {
      auv_interfaces::msg::SemanticCell cell;
      cell.row = planning_cell.cell.row;
      cell.col = planning_cell.cell.col;
      cell.object_type = planning_cell.cell.object_type;
      cell.visited = planning_cell.visited;
      cell.confidence = result.geometry_valid ? result.confidence : 0.0F;
      bool cone_confidence_set = false;
      for (const auto & observation : observations) {
        if (observation.row == cell.row && observation.col == cell.col &&
          cell.object_type != "unknown")
        {
          cell.confidence = cone_confidence_set ?
            std::max(cell.confidence, observation.confidence) : observation.confidence;
          cone_confidence_set = true;
        }
      }
      map.cells.push_back(std::move(cell));
    }
    map_publisher_->publish(map);
  }

  void process_image(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    cv_bridge::CvImageConstPtr converted;
    try {
      converted = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "image conversion failed: %s", error.what());
      return;
    }

    cv::Mat input;
    if (calibration_enabled_) {
      cv::undistort(
        converted->image, input, camera_matrix_, distortion_coefficients_);
    } else {
      input = converted->image;
    }

    const GridResult result = mapper_->process(input);
    auv_interfaces::msg::GridPose pose;
    pose.header = message->header;
    pose.valid = result.stable && result.position_valid;
    pose.row = result.camera_row;
    pose.col = result.camera_col;
    pose.confidence = result.confidence;
    grid_pose_publisher_->publish(pose);
    publish_map(message->header, result);
    if (result.stable) {
      rectified_publisher_.publish(
        *cv_bridge::CvImage(
          message->header, sensor_msgs::image_encodings::BGR8,
          result.rectified).toImageMsg());
    }
    if (publish_debug_image_) {
      debug_publisher_.publish(
        *cv_bridge::CvImage(
          message->header, sensor_msgs::image_encodings::BGR8,
          result.debug_image).toImageMsg());
    }
    if (!result.geometry_valid) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000, "grid incomplete: %s", result.reason.c_str());
    }
  }

  std::string image_topic_;
  std::string map_topic_;
  std::string rectified_topic_;
  std::string debug_topic_;
  std::string cone_detections_topic_;
  std::string grid_pose_topic_;
  std::string visited_cell_topic_;
  bool publish_debug_image_{false};
  bool require_calibration_{true};
  bool calibration_enabled_{false};
  double cone_detection_timeout_sec_{1.0};
  int expected_cone_count_{4};
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  std::unique_ptr<GridMapper> mapper_;
  rclcpp::Publisher<auv_interfaces::msg::SemanticMap>::SharedPtr map_publisher_;
  rclcpp::Publisher<auv_interfaces::msg::GridPose>::SharedPtr grid_pose_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<auv_interfaces::msg::ConeDetectionArray>::SharedPtr cone_subscription_;
  rclcpp::Subscription<auv_interfaces::msg::GridCell>::SharedPtr visited_cell_subscription_;
  auv_interfaces::msg::ConeDetectionArray::ConstSharedPtr latest_cones_;
  rclcpp::Time latest_cones_received_{0, 0, RCL_ROS_TIME};
  image_transport::Publisher rectified_publisher_;
  image_transport::Publisher debug_publisher_;
  std::array<bool, 9> visited_cells_{};
};

std::shared_ptr<rclcpp::Node> make_semantic_mapper_node(
  const rclcpp::NodeOptions & options)
{
  return std::make_shared<SemanticMapperNode>(options);
}

}  // namespace auv_mapping
