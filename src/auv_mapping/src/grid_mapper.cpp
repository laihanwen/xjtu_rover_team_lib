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

#include "auv_mapping/grid_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "opencv2/imgproc.hpp"

namespace auv_mapping
{

GridMapper::GridMapper(GridMapperConfig config)
: config_(std::move(config))
{
  if (config_.morphology_kernel <= 0 || config_.morphology_kernel % 2 == 0) {
    throw std::invalid_argument("morphology_kernel must be a positive odd number");
  }
  if (config_.minimum_area_ratio <= 0.0 ||
    config_.maximum_area_ratio <= config_.minimum_area_ratio ||
    config_.maximum_area_ratio > 1.0)
  {
    throw std::invalid_argument("grid area ratios are invalid");
  }
  if (config_.polygon_epsilon_ratio <= 0.0 || config_.polygon_epsilon_ratio >= 0.2 ||
    config_.clahe_clip_limit <= 0.0 || config_.minimum_corner_angle_degrees <= 0.0 ||
    config_.maximum_corner_angle_degrees >= 180.0 ||
    config_.maximum_corner_angle_degrees <= config_.minimum_corner_angle_degrees ||
    config_.output_size < 90 || config_.line_band_ratio <= 0.0 ||
    config_.line_band_ratio >= 0.15 || config_.minimum_line_support <= 0.0 ||
    config_.minimum_line_support > 1.0 || config_.stable_frames <= 0 ||
    config_.maximum_corner_jitter_ratio <= 0.0)
  {
    throw std::invalid_argument("grid mapper configuration is invalid");
  }
}

void GridMapper::reset()
{
  previous_corners_valid_ = false;
  stable_count_ = 0;
}

std::array<cv::Point2f, 4> GridMapper::order_corners(
  const std::array<cv::Point2f, 4> & corners)
{
  cv::Point2f center(0.0F, 0.0F);
  for (const auto & corner : corners) {
    center += corner;
  }
  center *= 0.25F;

  std::array<cv::Point2f, 4> ordered = corners;
  std::sort(
    ordered.begin(), ordered.end(), [&center](const cv::Point2f & lhs, const cv::Point2f & rhs) {
      return std::atan2(lhs.y - center.y, lhs.x - center.x) <
             std::atan2(rhs.y - center.y, rhs.x - center.x);
    });
  const auto top_left = std::min_element(
    ordered.begin(), ordered.end(), [](const cv::Point2f & lhs, const cv::Point2f & rhs) {
      return lhs.x + lhs.y < rhs.x + rhs.y;
    });
  std::rotate(ordered.begin(), top_left, ordered.end());

  const cv::Point2f first_edge = ordered[1] - ordered[0];
  const cv::Point2f second_edge = ordered[2] - ordered[1];
  if (first_edge.x * second_edge.y - first_edge.y * second_edge.x < 0.0F) {
    std::swap(ordered[1], ordered[3]);
  }
  return ordered;
}

bool GridMapper::find_outer_grid(
  const cv::Mat & mask, std::array<cv::Point2f, 4> & corners,
  double & area_ratio) const
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  const double image_area = static_cast<double>(mask.rows) * mask.cols;
  double best_area = 0.0;
  std::array<cv::Point2f, 4> best{};

  for (const auto & contour : contours) {
    const double area = std::abs(cv::contourArea(contour));
    const double ratio = area / image_area;
    if (ratio < config_.minimum_area_ratio || ratio > config_.maximum_area_ratio) {
      continue;
    }
    const double perimeter = cv::arcLength(contour, true);
    std::vector<cv::Point> polygon;
    cv::approxPolyDP(contour, polygon, perimeter * config_.polygon_epsilon_ratio, true);
    if (polygon.size() != 4U || !cv::isContourConvex(polygon) || area <= best_area) {
      continue;
    }
    std::array<cv::Point2f, 4> candidate{};
    for (std::size_t i = 0; i < candidate.size(); ++i) {
      candidate[i] = polygon[i];
    }
    candidate = order_corners(candidate);
    bool sides_valid = true;
    bool angles_valid = true;
    const double minimum_side = 0.08 * std::min(mask.cols, mask.rows);
    for (std::size_t i = 0; i < candidate.size(); ++i) {
      if (cv::norm(candidate[i] - candidate[(i + 1U) % candidate.size()]) < minimum_side) {
        sides_valid = false;
      }
      const cv::Point2f previous = candidate[(i + candidate.size() - 1U) % candidate.size()] -
        candidate[i];
      const cv::Point2f next = candidate[(i + 1U) % candidate.size()] - candidate[i];
      const double cosine = std::clamp(
        static_cast<double>(previous.dot(next)) / (cv::norm(previous) * cv::norm(next)),
        -1.0, 1.0);
      const double angle = std::acos(cosine) * 180.0 / CV_PI;
      if (angle < config_.minimum_corner_angle_degrees ||
        angle > config_.maximum_corner_angle_degrees)
      {
        angles_valid = false;
      }
    }
    if (sides_valid && angles_valid) {
      best_area = area;
      best = candidate;
    }
  }

  if (best_area == 0.0) {
    return false;
  }
  corners = best;
  area_ratio = best_area / image_area;
  return true;
}

double GridMapper::measure_grid_line_support(const cv::Mat & rectified) const
{
  cv::Mat gray;
  cv::Mat edges;
  cv::cvtColor(rectified, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.0);
  cv::Canny(gray, edges, 50.0, 150.0);

  const int size = rectified.rows;
  const int half_band = std::max(2, static_cast<int>(size * config_.line_band_ratio));
  double minimum_support = 1.0;
  for (int division = 1; division <= 2; ++division) {
    const int coordinate = size * division / 3;
    const int start = std::max(0, coordinate - half_band);
    const int width = std::min(size, coordinate + half_band + 1) - start;
    const cv::Mat vertical_band = edges(cv::Rect(start, 0, width, size));
    const cv::Mat horizontal_band = edges(cv::Rect(0, start, size, width));

    cv::Mat vertical_rows;
    cv::Mat horizontal_columns;
    cv::reduce(vertical_band, vertical_rows, 1, cv::REDUCE_MAX);
    cv::reduce(horizontal_band, horizontal_columns, 0, cv::REDUCE_MAX);
    const double vertical_support =
      static_cast<double>(cv::countNonZero(vertical_rows)) / size;
    const double horizontal_support =
      static_cast<double>(cv::countNonZero(horizontal_columns)) / size;
    minimum_support = std::min(minimum_support, std::min(vertical_support, horizontal_support));
  }
  return minimum_support;
}

bool GridMapper::update_stability(
  const std::array<cv::Point2f, 4> & corners, const cv::Size & image_size)
{
  if (!previous_corners_valid_) {
    previous_corners_ = corners;
    previous_corners_valid_ = true;
    stable_count_ = 1;
    return stable_count_ >= config_.stable_frames;
  }

  double maximum_jitter = 0.0;
  for (std::size_t i = 0; i < corners.size(); ++i) {
    maximum_jitter = std::max(maximum_jitter, cv::norm(corners[i] - previous_corners_[i]));
  }
  const double diagonal = std::hypot(image_size.width, image_size.height);
  if (maximum_jitter / diagonal <= config_.maximum_corner_jitter_ratio) {
    ++stable_count_;
  } else {
    stable_count_ = 1;
  }
  previous_corners_ = corners;
  return stable_count_ >= config_.stable_frames;
}

cv::Mat GridMapper::draw_debug(
  const cv::Mat & image, const cv::Mat & mask, const GridResult & result) const
{
  cv::Mat debug = image.clone();
  cv::Mat yellow_overlay = cv::Mat::zeros(image.size(), image.type());
  yellow_overlay.setTo(cv::Scalar(0, 180, 255), mask);
  cv::addWeighted(debug, 1.0, yellow_overlay, 0.25, 0.0, debug);
  const bool has_corners = std::any_of(
    result.corners.begin(), result.corners.end(), [](const cv::Point2f & corner) {
      return corner.x != 0.0F || corner.y != 0.0F;
    });
  if (has_corners) {
    for (std::size_t i = 0; i < result.corners.size(); ++i) {
      cv::line(
        debug, result.corners[i], result.corners[(i + 1U) % result.corners.size()],
        result.stable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 3);
      cv::putText(
        debug, std::to_string(i), result.corners[i], cv::FONT_HERSHEY_SIMPLEX,
        0.6, cv::Scalar(255, 0, 0), 2);
    }
  }
  cv::putText(
    debug, result.reason, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
    result.stable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
  return debug;
}

GridResult GridMapper::process(const cv::Mat & bgr_image)
{
  if (bgr_image.empty() || bgr_image.type() != CV_8UC3) {
    reset();
    throw std::invalid_argument("grid mapper requires a non-empty BGR8 image");
  }

  cv::Mat lab;
  cv::Mat enhanced;
  cv::Mat blurred;
  cv::Mat hsv;
  cv::Mat mask;
  cv::cvtColor(bgr_image, lab, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> lab_channels;
  cv::split(lab, lab_channels);
  cv::createCLAHE(config_.clahe_clip_limit, cv::Size(8, 8))->apply(
    lab_channels[0], lab_channels[0]);
  cv::merge(lab_channels, lab);
  cv::cvtColor(lab, enhanced, cv::COLOR_Lab2BGR);
  cv::GaussianBlur(enhanced, blurred, cv::Size(5, 5), 0.0);
  cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, config_.hsv_lower, config_.hsv_upper, mask);
  const cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(config_.morphology_kernel, config_.morphology_kernel));
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  GridResult result;
  double area_ratio = 0.0;
  if (!find_outer_grid(mask, result.corners, area_ratio)) {
    reset();
    result.reason = "no valid yellow quadrilateral";
    result.debug_image = draw_debug(bgr_image, mask, result);
    return result;
  }

  const float maximum = static_cast<float>(config_.output_size - 1);
  const std::array<cv::Point2f, 4> destination{
    cv::Point2f(0.0F, 0.0F), cv::Point2f(maximum, 0.0F),
    cv::Point2f(maximum, maximum), cv::Point2f(0.0F, maximum)};
  const cv::Mat transform = cv::getPerspectiveTransform(result.corners.data(), destination.data());
  cv::warpPerspective(
    bgr_image, result.rectified, transform,
    cv::Size(config_.output_size, config_.output_size));

  const double line_support = measure_grid_line_support(result.rectified);
  if (line_support < config_.minimum_line_support) {
    reset();
    result.reason = "internal grid lines missing";
    result.debug_image = draw_debug(bgr_image, mask, result);
    return result;
  }

  result.geometry_valid = true;
  result.stable = update_stability(result.corners, bgr_image.size());
  const double normalized_area = std::min(
    1.0, area_ratio / std::max(config_.minimum_area_ratio, 0.5));
  result.confidence = static_cast<float>(
    std::clamp(0.4 * normalized_area + 0.6 * line_support, 0.0, 1.0));
  result.reason = result.stable ? "grid stable" : "waiting for stable frames";
  result.debug_image = draw_debug(bgr_image, mask, result);
  return result;
}

}  // namespace auv_mapping
