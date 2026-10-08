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

#include "auv_vision/cone_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "opencv2/imgproc.hpp"

namespace auv_vision
{
namespace
{

void validate_range(const cv::Scalar & lower, const cv::Scalar & upper, const char * name)
{
  for (int channel = 0; channel < 3; ++channel) {
    if (lower[channel] < 0.0 || upper[channel] > 255.0 || lower[channel] > upper[channel]) {
      throw std::invalid_argument(std::string(name) + " range is invalid");
    }
  }
}

float aspect_score(const cv::Rect & bounds)
{
  const float largest = static_cast<float>(std::max(bounds.width, bounds.height));
  if (largest <= 0.0F) {
    return 0.0F;
  }
  return static_cast<float>(std::min(bounds.width, bounds.height)) / largest;
}

}  // namespace

const char * cone_shape_name(const ConeShape shape)
{
  switch (shape) {
    case ConeShape::kCircle:
      return "circle_cone";
    case ConeShape::kSquare:
      return "square_cone";
    default:
      return "unknown";
  }
}

ConeDetector::ConeDetector(ConeDetectorConfig config)
: config_(std::move(config))
{
  validate_range(config_.hsv_lower_1, config_.hsv_upper_1, "primary HSV");
  validate_range(config_.hsv_lower_2, config_.hsv_upper_2, "secondary HSV");
  validate_range(config_.lab_lower, config_.lab_upper, "LAB");
  if (config_.hsv_upper_1[0] > 179.0 || config_.hsv_upper_2[0] > 179.0) {
    throw std::invalid_argument("OpenCV HSV hue must not exceed 179");
  }
  if (config_.morphology_kernel <= 0 || config_.morphology_kernel % 2 == 0 ||
    config_.clahe_clip_limit <= 0.0 || config_.cell_margin_ratio < 0.0 ||
    config_.cell_margin_ratio >= 0.4 || config_.minimum_area_ratio <= 0.0 ||
    config_.maximum_area_ratio <= config_.minimum_area_ratio ||
    config_.maximum_area_ratio > 1.0 || config_.polygon_epsilon_ratio <= 0.0 ||
    config_.polygon_epsilon_ratio >= 0.2 || config_.minimum_solidity <= 0.0 ||
    config_.minimum_solidity > 1.0 || config_.minimum_aspect_score <= 0.0 ||
    config_.minimum_aspect_score > 1.0 || config_.circle_minimum_circularity <= 0.0 ||
    config_.circle_minimum_circularity > 1.0 || config_.square_minimum_extent <= 0.0 ||
    config_.square_minimum_extent > 1.0 || config_.minimum_confidence <= 0.0 ||
    config_.minimum_confidence > 1.0)
  {
    throw std::invalid_argument("cone detector configuration is invalid");
  }
}

ConeDetectorResult ConeDetector::process(const cv::Mat & rectified_bgr) const
{
  if (rectified_bgr.empty() || rectified_bgr.type() != CV_8UC3 ||
    rectified_bgr.rows < 90 || rectified_bgr.cols < 90)
  {
    throw std::invalid_argument("cone detector requires a BGR8 image at least 90x90");
  }

  cv::Mat lab;
  cv::Mat enhanced;
  cv::cvtColor(rectified_bgr, lab, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> lab_channels;
  cv::split(lab, lab_channels);
  cv::createCLAHE(config_.clahe_clip_limit, cv::Size(8, 8))->apply(
    lab_channels[0], lab_channels[0]);
  cv::merge(lab_channels, lab);
  cv::cvtColor(lab, enhanced, cv::COLOR_Lab2BGR);

  cv::Mat hsv;
  cv::Mat mask;
  cv::cvtColor(enhanced, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, config_.hsv_lower_1, config_.hsv_upper_1, mask);
  if (config_.use_second_hsv_range) {
    cv::Mat secondary;
    cv::inRange(hsv, config_.hsv_lower_2, config_.hsv_upper_2, secondary);
    cv::bitwise_or(mask, secondary, mask);
  }
  if (config_.use_lab_mask) {
    cv::Mat lab_mask;
    cv::inRange(lab, config_.lab_lower, config_.lab_upper, lab_mask);
    cv::bitwise_and(mask, lab_mask, mask);
  }
  const cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size(config_.morphology_kernel, config_.morphology_kernel));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  ConeDetectorResult result;
  result.mask = mask;
  result.debug_image = rectified_bgr.clone();
  const int cell_width = rectified_bgr.cols / 3;
  const int cell_height = rectified_bgr.rows / 3;

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const int cell_x0 = col * cell_width;
      const int cell_y0 = row * cell_height;
      const int cell_x1 = col == 2 ? rectified_bgr.cols : (col + 1) * cell_width;
      const int cell_y1 = row == 2 ? rectified_bgr.rows : (row + 1) * cell_height;
      const int margin_x = static_cast<int>((cell_x1 - cell_x0) * config_.cell_margin_ratio);
      const int margin_y = static_cast<int>((cell_y1 - cell_y0) * config_.cell_margin_ratio);
      const cv::Rect roi(
        cell_x0 + margin_x, cell_y0 + margin_y,
        cell_x1 - cell_x0 - 2 * margin_x, cell_y1 - cell_y0 - 2 * margin_y);
      cv::rectangle(result.debug_image, roi, cv::Scalar(90, 90, 90), 1);

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(
        mask(roi).clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      ConeObservation best;
      for (const auto & local_contour : contours) {
        const double area = std::abs(cv::contourArea(local_contour));
        const double roi_area = static_cast<double>(roi.area());
        const double area_ratio = area / roi_area;
        if (area_ratio < config_.minimum_area_ratio || area_ratio > config_.maximum_area_ratio) {
          continue;
        }
        const double perimeter = cv::arcLength(local_contour, true);
        if (perimeter <= 0.0) {
          continue;
        }
        const cv::Rect bounds = cv::boundingRect(local_contour);
        const bool touches_border = bounds.x <= 0 || bounds.y <= 0 ||
          bounds.x + bounds.width >= roi.width - 1 ||
          bounds.y + bounds.height >= roi.height - 1;
        if (touches_border) {
          continue;
        }
        std::vector<cv::Point> hull;
        cv::convexHull(local_contour, hull);
        const double hull_area = std::abs(cv::contourArea(hull));
        if (hull_area <= 0.0) {
          continue;
        }
        const double solidity = area / hull_area;
        const double extent = area / static_cast<double>(bounds.area());
        const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
        const double aspect = aspect_score(bounds);
        if (solidity < config_.minimum_solidity || aspect < config_.minimum_aspect_score) {
          continue;
        }
        std::vector<cv::Point> polygon;
        cv::approxPolyDP(
          local_contour, polygon, perimeter * config_.polygon_epsilon_ratio, true);

        ConeShape shape = ConeShape::kUnknown;
        double confidence = 0.0;
        if (polygon.size() == 4U && extent >= config_.square_minimum_extent) {
          shape = ConeShape::kSquare;
          confidence = (aspect + extent + solidity) / 3.0;
        } else {
          if (polygon.size() >= 6U &&
            circularity >= config_.circle_minimum_circularity)
          {
            shape = ConeShape::kCircle;
            confidence = (aspect + std::min(1.0, circularity) + solidity) / 3.0;
          }
        }
        confidence = std::clamp(confidence, 0.0, 1.0);
        if (shape == ConeShape::kUnknown || confidence < config_.minimum_confidence ||
          confidence <= best.confidence)
        {
          continue;
        }

        const cv::Moments moments = cv::moments(local_contour);
        if (moments.m00 == 0.0) {
          continue;
        }
        best.shape = shape;
        best.row = row;
        best.col = col;
        best.center = cv::Point2f(
          static_cast<float>(roi.x + moments.m10 / moments.m00),
          static_cast<float>(roi.y + moments.m01 / moments.m00));
        best.width = static_cast<float>(bounds.width);
        best.height = static_cast<float>(bounds.height);
        best.area = static_cast<float>(area);
        best.circularity = static_cast<float>(circularity);
        best.confidence = static_cast<float>(confidence);
        best.contour.clear();
        for (const auto& point : local_contour) best.contour.push_back(point + roi.tl());
      }

      if (best.shape != ConeShape::kUnknown) {
        result.observations.push_back(best);
        const cv::Scalar color = best.shape == ConeShape::kCircle ?
          cv::Scalar(0, 255, 0) : cv::Scalar(255, 80, 0);
        cv::circle(result.debug_image, best.center, 5, color, cv::FILLED);
        cv::putText(
          result.debug_image,
          std::string(cone_shape_name(best.shape)) + " " +
          cv::format("%.2f", best.confidence),
          best.center + cv::Point2f(7.0F, -7.0F), cv::FONT_HERSHEY_SIMPLEX,
          0.45, color, 1, cv::LINE_AA);
      }
    }
  }
  return result;
}

ConeTracker::ConeTracker(ConeTrackerConfig config)
: config_(std::move(config))
{
  if (config_.history_size <= 0 || config_.required_votes <= 0 ||
    config_.required_votes > config_.history_size || config_.clear_votes <= 0 ||
    config_.clear_votes > config_.history_size)
  {
    throw std::invalid_argument("cone tracker configuration is invalid");
  }
}

void ConeTracker::reset()
{
  for (auto & history : history_) {
    history.clear();
  }
  stable_.fill(ConeObservation{});
  update_count_ = 0U;
}

bool ConeTracker::ready() const noexcept
{
  return update_count_ >= static_cast<std::size_t>(config_.history_size);
}

std::vector<ConeObservation> ConeTracker::update(
  const std::vector<ConeObservation> & observations)
{
  ++update_count_;
  std::array<CellSample, 9> samples{};
  for (const auto & observation : observations) {
    if (observation.row < 0 || observation.row >= 3 || observation.col < 0 ||
      observation.col >= 3 || observation.shape == ConeShape::kUnknown)
    {
      continue;
    }
    const std::size_t index = static_cast<std::size_t>(observation.row * 3 + observation.col);
    if (observation.confidence > samples[index].observation.confidence) {
      samples[index].shape = observation.shape;
      samples[index].observation = observation;
    }
  }

  for (std::size_t index = 0; index < history_.size(); ++index) {
    auto & history = history_[index];
    history.push_back(samples[index]);
    while (history.size() > static_cast<std::size_t>(config_.history_size)) {
      history.pop_front();
    }

    int unknown_votes = 0;
    int circle_votes = 0;
    int square_votes = 0;
    ConeObservation latest_circle;
    ConeObservation latest_square;
    float circle_confidence_sum = 0.0F;
    float square_confidence_sum = 0.0F;
    for (const auto & sample : history) {
      switch (sample.shape) {
        case ConeShape::kCircle:
          ++circle_votes;
          circle_confidence_sum += sample.observation.confidence;
          latest_circle = sample.observation;
          break;
        case ConeShape::kSquare:
          ++square_votes;
          square_confidence_sum += sample.observation.confidence;
          latest_square = sample.observation;
          break;
        default:
          ++unknown_votes;
          break;
      }
    }
    if (circle_votes >= config_.required_votes && circle_votes > square_votes) {
      latest_circle.confidence = circle_confidence_sum / circle_votes;
      stable_[index] = latest_circle;
    } else if (square_votes >= config_.required_votes && square_votes > circle_votes) {
      latest_square.confidence = square_confidence_sum / square_votes;
      stable_[index] = latest_square;
    } else if (unknown_votes >= config_.clear_votes) {
      stable_[index] = ConeObservation{};
    }
  }

  std::vector<ConeObservation> stable_observations;
  for (const auto & observation : stable_) {
    if (observation.shape != ConeShape::kUnknown) {
      stable_observations.push_back(observation);
    }
  }
  return stable_observations;
}

}  // namespace auv_vision
