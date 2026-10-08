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

#ifndef AUV_MAPPING__GRID_MAPPER_HPP_
#define AUV_MAPPING__GRID_MAPPER_HPP_

#include <array>
#include <string>

#include "opencv2/core.hpp"

namespace auv_mapping
{

struct GridMapperConfig
{
  // Legacy fixtures retain the closed yellow frame. Competition uses one yellow edge.
  bool single_yellow_edge{false};
  int dark_value_max{95};
  double yellow_edge_minimum_support{0.45};
  double yellow_edge_margin{0.20};
  cv::Scalar hsv_lower{15, 60, 60};
  cv::Scalar hsv_upper{40, 255, 255};
  double clahe_clip_limit{2.0};
  int morphology_kernel{5};
  double minimum_area_ratio{0.15};
  double maximum_area_ratio{0.95};
  double polygon_epsilon_ratio{0.03};
  double minimum_corner_angle_degrees{20.0};
  double maximum_corner_angle_degrees{160.0};
  int output_size{600};
  double line_band_ratio{0.035};
  double minimum_line_support{0.45};
  int stable_frames{3};
  double maximum_corner_jitter_ratio{0.02};
};

struct GridResult
{
  bool orientation_valid{false};
  int yellow_edge{-1};
  bool geometry_valid{false};
  bool stable{false};
  float confidence{0.0F};
  bool position_valid{false};
  float camera_row{0.0F};
  float camera_col{0.0F};
  std::string reason{"grid not processed"};
  std::array<cv::Point2f, 4> corners{};
  cv::Mat rectified;
  cv::Mat debug_image;
};

class GridMapper
{
public:
  explicit GridMapper(GridMapperConfig config = GridMapperConfig());

  GridResult process(const cv::Mat & bgr_image);
  void reset();

  static std::array<cv::Point2f, 4> order_corners(
    const std::array<cv::Point2f, 4> & corners);

private:
  bool find_outer_grid(
    const cv::Mat & mask, std::array<cv::Point2f, 4> & corners,
    double & area_ratio) const;
  double measure_grid_line_support(const cv::Mat & rectified) const;
  bool update_stability(
    const std::array<cv::Point2f, 4> & corners, const cv::Size & image_size);
  cv::Mat draw_debug(
    const cv::Mat & image, const cv::Mat & mask,
    const GridResult & result) const;

  GridMapperConfig config_;
  std::array<cv::Point2f, 4> previous_corners_{};
  bool previous_corners_valid_{false};
  int stable_count_{0};
};

}  // namespace auv_mapping

#endif  // AUV_MAPPING__GRID_MAPPER_HPP_
