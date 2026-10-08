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

#ifndef AUV_VISION__APRILTAG_DETECTOR_HPP_
#define AUV_VISION__APRILTAG_DETECTOR_HPP_

#include <array>
#include <string>
#include <vector>

#include "opencv2/objdetect/aruco_detector.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"

namespace auv_vision
{

struct AprilTagObservation
{
  int id{-1};
  cv::Point2f center{};
  std::array<cv::Point2f, 4> corners{};
};

class AprilTagDetector
{
public:
  explicit AprilTagDetector(
    const std::string & family = "tag36h11", double decimate = 1.0,
    bool refine_edges = true);

  std::vector<AprilTagObservation> detect(const cv::Mat & image) const;
  const std::string & family() const noexcept;

  static int dictionary_id(const std::string & family);

private:
  std::string family_;
  cv::aruco::Dictionary dictionary_;
  cv::aruco::DetectorParameters parameters_;
};

}  // namespace auv_vision

#endif  // AUV_VISION__APRILTAG_DETECTOR_HPP_
