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

#include "auv_vision/valve_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "opencv2/imgproc.hpp"

namespace auv_vision
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

bool valid_config(const ValveDetectorConfig & config)
{
  return std::isfinite(config.minimum_area_ratio) &&
         std::isfinite(config.maximum_area_ratio) &&
         std::isfinite(config.minimum_circularity) &&
         std::isfinite(config.minimum_aspect_score) &&
         std::isfinite(config.minimum_confidence) &&
         std::isfinite(config.canny_low) && std::isfinite(config.canny_high) &&
         std::isfinite(config.handle_minimum_length_ratio) &&
         std::isfinite(config.handle_maximum_center_offset_ratio) &&
         config.minimum_area_ratio > 0.0 &&
         config.maximum_area_ratio > config.minimum_area_ratio &&
         config.maximum_area_ratio <= 1.0 &&
         config.minimum_circularity > 0.0 && config.minimum_circularity <= 1.0 &&
         config.minimum_aspect_score > 0.0 && config.minimum_aspect_score <= 1.0 &&
         config.minimum_confidence > 0.0 && config.minimum_confidence <= 1.0 &&
         config.blur_kernel >= 1 && (config.blur_kernel % 2) == 1 &&
         config.canny_low >= 0.0 && config.canny_high > config.canny_low &&
         config.morphology_kernel >= 1 && (config.morphology_kernel % 2) == 1 &&
         config.handle_minimum_length_ratio > 0.0 &&
         config.handle_maximum_center_offset_ratio >= 0.0;
}

float normalized_line_angle(float angle)
{
  const float half_pi = static_cast<float>(kPi / 2.0);
  const float pi = static_cast<float>(kPi);
  while (angle >= half_pi) {angle -= pi;}
  while (angle < -half_pi) {angle += pi;}
  return angle;
}

}  // namespace

ValveDetector::ValveDetector(ValveDetectorConfig config)
: config_(std::move(config))
{
  if (!valid_config(config_)) {
    throw std::invalid_argument("invalid valve detector configuration");
  }
}

ValveDetectorResult ValveDetector::process(const cv::Mat & bgr_image) const
{
  if (bgr_image.empty() || bgr_image.type() != CV_8UC3) {
    throw std::invalid_argument("valve detector expects a non-empty BGR8 image");
  }

  ValveDetectorResult result;
  result.debug_image = bgr_image.clone();

  cv::Mat gray;
  cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(
    gray, gray, cv::Size(config_.blur_kernel, config_.blur_kernel), 0.0);
  cv::Canny(gray, result.edge_mask, config_.canny_low, config_.canny_high);
  const cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE,
    cv::Size(config_.morphology_kernel, config_.morphology_kernel));
  cv::morphologyEx(result.edge_mask, result.edge_mask, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(result.edge_mask.clone(), contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  const double image_area = static_cast<double>(bgr_image.rows * bgr_image.cols);
  double best_score = 0.0;
  std::vector<cv::Point> best_contour;

  for (const auto & contour : contours) {
    const double area = std::abs(cv::contourArea(contour));
    const double area_ratio = area / image_area;
    if (area_ratio < config_.minimum_area_ratio || area_ratio > config_.maximum_area_ratio) {
      continue;
    }
    const double perimeter = cv::arcLength(contour, true);
    if (perimeter <= 0.0) {continue;}
    const double circularity = std::min(1.0, 4.0 * kPi * area / (perimeter * perimeter));
    const cv::Rect bounds = cv::boundingRect(contour);
    const double aspect = static_cast<double>(std::min(bounds.width, bounds.height)) /
      static_cast<double>(std::max(bounds.width, bounds.height));
    if (circularity < config_.minimum_circularity ||
      aspect < config_.minimum_aspect_score)
    {
      continue;
    }
    const double score = 0.65 * circularity + 0.35 * aspect;
    if (score > best_score) {
      best_score = score;
      best_contour = contour;
    }
  }

  if (best_contour.empty() || best_score < config_.minimum_confidence) {
    cv::putText(
      result.debug_image, "VALVE: not detected", cv::Point(12, 28),
      cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 255), 2);
    return result;
  }

  cv::Point2f center;
  float radius = 0.0F;
  cv::minEnclosingCircle(best_contour, center, radius);
  result.observation.detected = true;
  result.observation.center = center;
  result.observation.radius = radius;
  result.observation.circularity = static_cast<float>(
    std::min(1.0, 4.0 * kPi * std::abs(cv::contourArea(best_contour)) /
    std::pow(cv::arcLength(best_contour, true), 2.0)));
  result.observation.confidence = static_cast<float>(best_score);

  cv::Mat valve_mask = cv::Mat::zeros(result.edge_mask.size(), CV_8UC1);
  cv::circle(valve_mask, center, cvRound(radius * 0.95F), cv::Scalar(255), cv::FILLED);
  cv::Mat valve_edges;
  cv::bitwise_and(result.edge_mask, valve_mask, valve_edges);
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(
    valve_edges, lines, 1.0, kPi / 180.0, 20,
    std::max(5.0, radius * config_.handle_minimum_length_ratio), 8.0);
  double best_length = 0.0;
  cv::Vec4i best_line{};
  for (const auto & line : lines) {
    const cv::Point2f first(line[0], line[1]);
    const cv::Point2f second(line[2], line[3]);
    const cv::Point2f midpoint = (first + second) * 0.5F;
    const double length = cv::norm(first - second);
    const double center_offset = cv::norm(midpoint - center);
    const bool endpoints_inside = cv::norm(first - center) <= radius &&
      cv::norm(second - center) <= radius;
    if (endpoints_inside &&
      center_offset <= radius * config_.handle_maximum_center_offset_ratio &&
      length <= 2.1 * radius && length > best_length)
    {
      best_length = length;
      best_line = line;
    }
  }
  if (best_length > 0.0) {
    result.observation.handle_angle_valid = true;
    result.observation.handle_angle_rad = normalized_line_angle(
      std::atan2(
        static_cast<float>(best_line[3] - best_line[1]),
        static_cast<float>(best_line[2] - best_line[0])));
    cv::line(
      result.debug_image, cv::Point(best_line[0], best_line[1]),
      cv::Point(best_line[2], best_line[3]), cv::Scalar(0, 255, 255), 3);
  }

  cv::circle(result.debug_image, center, cvRound(radius), cv::Scalar(0, 255, 0), 2);
  cv::circle(result.debug_image, center, 3, cv::Scalar(255, 0, 0), -1);
  cv::putText(
    result.debug_image, "VALVE", cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
    0.65, cv::Scalar(0, 255, 0), 2);
  return result;
}

}  // namespace auv_vision
