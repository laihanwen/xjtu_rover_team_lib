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

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "auv_interfaces/msg/april_tag_detection.hpp"
#include "auv_interfaces/msg/april_tag_detection_array.hpp"
#include "auv_vision/apriltag_detector.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace auv_vision
{

geometry_msgs::msg::Quaternion rotation_to_quaternion(const cv::Mat & rotation)
{
  geometry_msgs::msg::Quaternion quaternion;
  const double trace = cv::trace(rotation)[0];
  const bool x_axis_is_largest =
    rotation.at<double>(0, 0) > rotation.at<double>(1, 1) &&
    rotation.at<double>(0, 0) > rotation.at<double>(2, 2);
  if (trace > 0.0) {
    const double scale = 2.0 * std::sqrt(trace + 1.0);
    quaternion.w = 0.25 * scale;
    quaternion.x = (rotation.at<double>(2, 1) - rotation.at<double>(1, 2)) / scale;
    quaternion.y = (rotation.at<double>(0, 2) - rotation.at<double>(2, 0)) / scale;
    quaternion.z = (rotation.at<double>(1, 0) - rotation.at<double>(0, 1)) / scale;
  } else if (x_axis_is_largest) {
    const double scale = 2.0 * std::sqrt(
      1.0 + rotation.at<double>(0, 0) - rotation.at<double>(1, 1) -
      rotation.at<double>(2, 2));
    quaternion.w = (rotation.at<double>(2, 1) - rotation.at<double>(1, 2)) / scale;
    quaternion.x = 0.25 * scale;
    quaternion.y = (rotation.at<double>(0, 1) + rotation.at<double>(1, 0)) / scale;
    quaternion.z = (rotation.at<double>(0, 2) + rotation.at<double>(2, 0)) / scale;
  } else if (rotation.at<double>(1, 1) > rotation.at<double>(2, 2)) {
    const double scale = 2.0 * std::sqrt(
      1.0 + rotation.at<double>(1, 1) - rotation.at<double>(0, 0) -
      rotation.at<double>(2, 2));
    quaternion.w = (rotation.at<double>(0, 2) - rotation.at<double>(2, 0)) / scale;
    quaternion.x = (rotation.at<double>(0, 1) + rotation.at<double>(1, 0)) / scale;
    quaternion.y = 0.25 * scale;
    quaternion.z = (rotation.at<double>(1, 2) + rotation.at<double>(2, 1)) / scale;
  } else {
    const double scale = 2.0 * std::sqrt(
      1.0 + rotation.at<double>(2, 2) - rotation.at<double>(0, 0) -
      rotation.at<double>(1, 1));
    quaternion.w = (rotation.at<double>(1, 0) - rotation.at<double>(0, 1)) / scale;
    quaternion.x = (rotation.at<double>(0, 2) + rotation.at<double>(2, 0)) / scale;
    quaternion.y = (rotation.at<double>(1, 2) + rotation.at<double>(2, 1)) / scale;
    quaternion.z = 0.25 * scale;
  }
  return quaternion;
}

class AprilTagDetectorNode final : public rclcpp::Node
{
public:
  AprilTagDetectorNode()
  : Node("auv_apriltag_detector"),
    detector_(
      declare_parameter<std::string>("family", "tag36h11"),
      declare_parameter<double>("decimate", 1.0),
      declare_parameter<bool>("refine_edges", true))
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/down/image_raw");
    detections_topic_ = declare_parameter<std::string>(
      "detections_topic", "/apriltag/detections");
    debug_topic_ = declare_parameter<std::string>(
      "debug_topic", "/apriltag/debug_image");
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", false);
    tag_size_ = declare_parameter<double>("tag_size", 0.16);
    const auto intrinsics = declare_parameter<std::vector<double>>(
      "camera_matrix", std::vector<double>{});
    const auto distortion = declare_parameter<std::vector<double>>(
      "distortion_coefficients", std::vector<double>{});

    if (image_topic_.empty() || detections_topic_.empty() || tag_size_ <= 0.0) {
      throw std::invalid_argument("image_topic, detections_topic and tag_size must be valid");
    }
    if (!intrinsics.empty()) {
      if (intrinsics.size() != 9U) {
        throw std::invalid_argument("camera_matrix must contain 9 row-major values");
      }
      camera_matrix_ = cv::Mat(3, 3, CV_64F);
      for (std::size_t i = 0; i < intrinsics.size(); ++i) {
        camera_matrix_.at<double>(static_cast<int>(i / 3U), static_cast<int>(i % 3U)) =
          intrinsics[i];
      }
      distortion_coefficients_ = cv::Mat(distortion).clone().reshape(1, 1);
      pose_enabled_ = true;
    } else if (!distortion.empty()) {
      throw std::invalid_argument("distortion_coefficients requires camera_matrix");
    }

    publisher_ = create_publisher<auv_interfaces::msg::AprilTagDetectionArray>(
      detections_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {process_image(message);});
    if (publish_debug_image_) {
      debug_publisher_ = image_transport::create_publisher(
        *this, debug_topic_, rclcpp::SensorDataQoS());
    }

    RCLCPP_INFO(
      get_logger(), "AprilTag detector ready: family=%s input=%s pose=%s",
      detector_.family().c_str(), image_topic_.c_str(), pose_enabled_ ? "enabled" : "disabled");
  }

private:
  bool estimate_pose(
    const AprilTagObservation & observation, geometry_msgs::msg::Pose & pose) const
  {
    const float half_size = static_cast<float>(tag_size_ * 0.5);
    const std::vector<cv::Point3f> object_points{
      {-half_size, half_size, 0.0F}, {half_size, half_size, 0.0F},
      {half_size, -half_size, 0.0F}, {-half_size, -half_size, 0.0F}};
    const std::vector<cv::Point2f> image_points{
      observation.corners.begin(), observation.corners.end()};
    cv::Mat rotation_vector;
    cv::Mat translation_vector;
    if (!cv::solvePnP(
        object_points, image_points, camera_matrix_, distortion_coefficients_,
        rotation_vector, translation_vector, false, cv::SOLVEPNP_IPPE_SQUARE))
    {
      return false;
    }

    cv::Mat rotation;
    cv::Rodrigues(rotation_vector, rotation);
    pose.position.x = translation_vector.at<double>(0);
    pose.position.y = translation_vector.at<double>(1);
    pose.position.z = translation_vector.at<double>(2);
    pose.orientation = rotation_to_quaternion(rotation);
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) && pose.position.z > 0.0;
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

    const auto observations = detector_.detect(converted->image);
    auv_interfaces::msg::AprilTagDetectionArray output;
    output.header = message->header;
    output.detections.reserve(observations.size());
    for (const auto & observation : observations) {
      auv_interfaces::msg::AprilTagDetection detection;
      detection.id = observation.id;
      detection.family = detector_.family();
      detection.center.x = observation.center.x;
      detection.center.y = observation.center.y;
      detection.tag_size = static_cast<float>(tag_size_);
      for (std::size_t i = 0; i < observation.corners.size(); ++i) {
        detection.corners[i].x = observation.corners[i].x;
        detection.corners[i].y = observation.corners[i].y;
      }
      detection.pose_valid = pose_enabled_ && estimate_pose(observation, detection.pose);
      output.detections.push_back(std::move(detection));
    }
    publisher_->publish(output);

    if (publish_debug_image_) {
      cv::Mat debug = converted->image.clone();
      for (const auto & observation : observations) {
        for (std::size_t i = 0; i < observation.corners.size(); ++i) {
          cv::line(
            debug, observation.corners[i], observation.corners[(i + 1U) % 4U],
            cv::Scalar(0, 255, 0), 2);
        }
        cv::putText(
          debug, std::to_string(observation.id), observation.center,
          cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
      }
      debug_publisher_.publish(
        *cv_bridge::CvImage(message->header, sensor_msgs::image_encodings::BGR8,
          debug).toImageMsg());
    }
  }

  AprilTagDetector detector_;
  std::string image_topic_;
  std::string detections_topic_;
  std::string debug_topic_;
  bool publish_debug_image_{false};
  double tag_size_{0.16};
  bool pose_enabled_{false};
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  rclcpp::Publisher<auv_interfaces::msg::AprilTagDetectionArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  image_transport::Publisher debug_publisher_;
};

}  // namespace auv_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auv_vision::AprilTagDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
