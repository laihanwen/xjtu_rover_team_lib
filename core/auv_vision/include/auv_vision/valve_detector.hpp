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

#ifndef AUV_VISION__VALVE_DETECTOR_HPP_
#define AUV_VISION__VALVE_DETECTOR_HPP_

#include "opencv2/core.hpp"

namespace auv_vision
{

struct ValveDetectorConfig
{
  double minimum_area_ratio{0.002};
  double maximum_area_ratio{0.65};
  double minimum_circularity{0.72};
  double minimum_aspect_score{0.72};
  double minimum_confidence{0.72};
  int blur_kernel{5};
  double canny_low{50.0};
  double canny_high{150.0};
  int morphology_kernel{3};
  double handle_minimum_length_ratio{0.30};
  double handle_maximum_center_offset_ratio{0.35};
};

struct ValveObservation
{
  bool detected{false};
  cv::Point2f center{};
  float radius{0.0F};
  float circularity{0.0F};
  float confidence{0.0F};
  bool handle_angle_valid{false};
  float handle_angle_rad{0.0F};
};

struct ValveDetectorResult
{
  ValveObservation observation;
  cv::Mat edge_mask;
  cv::Mat debug_image;
};

class ValveDetector
{
public:
  explicit ValveDetector(ValveDetectorConfig config = ValveDetectorConfig());

  ValveDetectorResult process(const cv::Mat & bgr_image) const;

private:
  ValveDetectorConfig config_;
};

}  // namespace auv_vision

#endif  // AUV_VISION__VALVE_DETECTOR_HPP_
