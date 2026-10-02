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

#ifndef AUV_VISION__CONE_DETECTOR_HPP_
#define AUV_VISION__CONE_DETECTOR_HPP_

#include <array>
#include <cstdint>
#include <deque>
#include <vector>

#include "opencv2/core.hpp"

namespace auv_vision
{

enum class ConeShape : std::uint8_t
{
  kUnknown = 0,
  kCircle = 1,
  kSquare = 2,
};

struct ConeObservation
{
  ConeShape shape{ConeShape::kUnknown};
  int row{-1};
  int col{-1};
  cv::Point2f center{};
  float width{0.0F};
  float height{0.0F};
  float area{0.0F};
  float circularity{0.0F};
  float confidence{0.0F};
};

struct ConeDetectorConfig
{
  cv::Scalar hsv_lower_1{0, 80, 50};
  cv::Scalar hsv_upper_1{20, 255, 255};
  bool use_second_hsv_range{true};
  cv::Scalar hsv_lower_2{165, 80, 50};
  cv::Scalar hsv_upper_2{179, 255, 255};
  bool use_lab_mask{false};
  cv::Scalar lab_lower{0, 0, 0};
  cv::Scalar lab_upper{255, 255, 255};
  double clahe_clip_limit{2.0};
  int morphology_kernel{5};
  double cell_margin_ratio{0.12};
  double minimum_area_ratio{0.025};
  double maximum_area_ratio{0.65};
  double polygon_epsilon_ratio{0.035};
  double minimum_solidity{0.80};
  double minimum_aspect_score{0.70};
  double circle_minimum_circularity{0.80};
  double square_minimum_extent{0.62};
  double minimum_confidence{0.72};
};

struct ConeDetectorResult
{
  std::vector<ConeObservation> observations;
  cv::Mat mask;
  cv::Mat debug_image;
};

class ConeDetector
{
public:
  explicit ConeDetector(ConeDetectorConfig config = ConeDetectorConfig());

  ConeDetectorResult process(const cv::Mat & rectified_bgr) const;

private:
  ConeDetectorConfig config_;
};

struct ConeTrackerConfig
{
  int history_size{5};
  int required_votes{3};
  int clear_votes{3};
};

class ConeTracker
{
public:
  explicit ConeTracker(ConeTrackerConfig config = ConeTrackerConfig());

  std::vector<ConeObservation> update(
    const std::vector<ConeObservation> & observations);
  void reset();

private:
  struct CellSample
  {
    ConeShape shape{ConeShape::kUnknown};
    ConeObservation observation{};
  };

  ConeTrackerConfig config_;
  std::array<std::deque<CellSample>, 9> history_{};
  std::array<ConeObservation, 9> stable_{};
};

const char * cone_shape_name(ConeShape shape);

}  // namespace auv_vision

#endif  // AUV_VISION__CONE_DETECTOR_HPP_
