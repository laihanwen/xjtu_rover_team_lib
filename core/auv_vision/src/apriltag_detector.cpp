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

#include "auv_vision/apriltag_detector.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "opencv2/imgproc.hpp"

namespace auv_vision
{

int AprilTagDetector::dictionary_id(const std::string & family)
{
  if (family == "tag16h5") {return cv::aruco::DICT_APRILTAG_16h5;}
  if (family == "tag25h9") {return cv::aruco::DICT_APRILTAG_25h9;}
  if (family == "tag36h10") {return cv::aruco::DICT_APRILTAG_36h10;}
  if (family == "tag36h11") {return cv::aruco::DICT_APRILTAG_36h11;}
  throw std::invalid_argument(
          "unsupported AprilTag family: " + family +
          " (expected tag16h5, tag25h9, tag36h10 or tag36h11)");
}

AprilTagDetector::AprilTagDetector(
  const std::string & family, double decimate, bool refine_edges)
: family_(family),
  dictionary_(cv::aruco::getPredefinedDictionary(dictionary_id(family)))
{
  if (decimate < 1.0) {
    throw std::invalid_argument("decimate must be at least 1.0");
  }
  parameters_.aprilTagQuadDecimate = static_cast<float>(decimate);
  parameters_.cornerRefinementMethod = refine_edges ?
    cv::aruco::CORNER_REFINE_APRILTAG : cv::aruco::CORNER_REFINE_NONE;
}

std::vector<AprilTagObservation> AprilTagDetector::detect(const cv::Mat & image) const
{
  if (image.empty()) {return {};}

  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  } else {
    throw std::invalid_argument("AprilTag input must have 1, 3 or 4 channels");
  }

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::ArucoDetector detector(dictionary_, parameters_);
  detector.detectMarkers(gray, corners, ids);

  std::vector<AprilTagObservation> observations;
  observations.reserve(ids.size());
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (corners[i].size() != 4U) {continue;}
    AprilTagObservation observation;
    observation.id = ids[i];
    observation.center = cv::Point2f{};
    for (std::size_t j = 0; j < 4U; ++j) {
      observation.corners[j] = corners[i][j];
      observation.center += corners[i][j];
    }
    observation.center *= 0.25F;
    observations.push_back(observation);
  }
  std::sort(
    observations.begin(), observations.end(),
    [](const auto & left, const auto & right) {return left.id < right.id;});
  return observations;
}

const std::string & AprilTagDetector::family() const noexcept
{
  return family_;
}

}  // namespace auv_vision
