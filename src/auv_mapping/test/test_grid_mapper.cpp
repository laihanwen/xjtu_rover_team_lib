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

#include "auv_mapping/grid_mapper.hpp"
#include "gtest/gtest.h"
#include "opencv2/imgproc.hpp"

namespace
{

cv::Mat make_grid(const bool draw_internal_lines = true)
{
  cv::Mat canonical(500, 500, CV_8UC3, cv::Scalar(220, 220, 220));
  cv::rectangle(canonical, cv::Rect(8, 8, 484, 484), cv::Scalar(0, 255, 255), 18);
  if (draw_internal_lines) {
    cv::line(canonical, cv::Point(167, 10), cv::Point(167, 490), cv::Scalar(20, 20, 20), 8);
    cv::line(canonical, cv::Point(333, 10), cv::Point(333, 490), cv::Scalar(20, 20, 20), 8);
    cv::line(canonical, cv::Point(10, 167), cv::Point(490, 167), cv::Scalar(20, 20, 20), 8);
    cv::line(canonical, cv::Point(10, 333), cv::Point(490, 333), cv::Scalar(20, 20, 20), 8);
  }

  const std::array<cv::Point2f, 4> source{
    cv::Point2f(0.0F, 0.0F), cv::Point2f(499.0F, 0.0F),
    cv::Point2f(499.0F, 499.0F), cv::Point2f(0.0F, 499.0F)};
  const std::array<cv::Point2f, 4> destination{
    cv::Point2f(115.0F, 80.0F), cv::Point2f(550.0F, 115.0F),
    cv::Point2f(590.0F, 430.0F), cv::Point2f(70.0F, 455.0F)};
  const cv::Mat transform = cv::getPerspectiveTransform(source.data(), destination.data());
  cv::Mat perspective(520, 660, CV_8UC3, cv::Scalar(5, 10, 15));
  cv::warpPerspective(
    canonical, perspective, transform, perspective.size(), cv::INTER_LINEAR,
    cv::BORDER_TRANSPARENT);
  return perspective;
}

TEST(GridMapper, OrdersCornersTopLeftClockwise)
{
  const std::array<cv::Point2f, 4> shuffled{
    cv::Point2f(90.0F, 80.0F), cv::Point2f(10.0F, 10.0F),
    cv::Point2f(100.0F, 15.0F), cv::Point2f(15.0F, 90.0F)};
  const auto ordered = auv_mapping::GridMapper::order_corners(shuffled);
  EXPECT_EQ(ordered[0], cv::Point2f(10.0F, 10.0F));
  EXPECT_EQ(ordered[1], cv::Point2f(100.0F, 15.0F));
  EXPECT_EQ(ordered[2], cv::Point2f(90.0F, 80.0F));
  EXPECT_EQ(ordered[3], cv::Point2f(15.0F, 90.0F));
}

TEST(GridMapper, RequiresStableValidGrid)
{
  auv_mapping::GridMapper mapper;
  const cv::Mat image = make_grid();
  const auto first = mapper.process(image);
  const auto second = mapper.process(image);
  const auto third = mapper.process(image);
  EXPECT_TRUE(first.geometry_valid) << first.reason;
  EXPECT_FALSE(first.stable);
  EXPECT_TRUE(second.geometry_valid);
  EXPECT_FALSE(second.stable);
  EXPECT_TRUE(third.geometry_valid);
  EXPECT_TRUE(third.stable);
  EXPECT_EQ(third.rectified.size(), cv::Size(600, 600));
  EXPECT_GT(third.confidence, 0.45F);
}

TEST(GridMapper, RejectsYellowQuadrilateralWithoutInternalGrid)
{
  auv_mapping::GridMapper mapper;
  const auto result = mapper.process(make_grid(false));
  EXPECT_FALSE(result.geometry_valid);
  EXPECT_FALSE(result.stable);
  EXPECT_EQ(result.reason, "internal grid lines missing");
}

TEST(GridMapper, RejectsBlankFrameAndResetsStability)
{
  auv_mapping::GridMapper mapper;
  const cv::Mat grid = make_grid();
  EXPECT_FALSE(mapper.process(grid).stable);
  EXPECT_FALSE(mapper.process(grid).stable);
  const cv::Mat blank(grid.size(), grid.type(), cv::Scalar(0, 0, 0));
  EXPECT_FALSE(mapper.process(blank).geometry_valid);
  EXPECT_FALSE(mapper.process(grid).stable);
}

TEST(GridMapper, HandlesBrightnessBlurAndModerateNoise)
{
  cv::Mat degraded;
  make_grid().convertTo(degraded, -1, 0.72, 8.0);
  cv::GaussianBlur(degraded, degraded, cv::Size(5, 5), 1.2);
  cv::Mat noise(degraded.size(), degraded.type());
  cv::randn(noise, cv::Scalar::all(0), cv::Scalar::all(5));
  cv::add(degraded, noise, degraded, cv::noArray(), degraded.type());

  auv_mapping::GridMapper mapper;
  const auto result = mapper.process(degraded);
  EXPECT_TRUE(result.geometry_valid) << result.reason;
}

TEST(GridMapper, RejectsOccludedBorderAndResetsCornerStability)
{
  auv_mapping::GridMapper mapper;
  const cv::Mat grid = make_grid();
  EXPECT_FALSE(mapper.process(grid).stable);
  EXPECT_FALSE(mapper.process(grid).stable);

  cv::Mat shifted;
  const cv::Mat translation = (cv::Mat_<double>(2, 3) << 1.0, 0.0, 30.0, 0.0, 1.0, 0.0);
  cv::warpAffine(grid, shifted, translation, grid.size());
  EXPECT_FALSE(mapper.process(shifted).stable);

  cv::Mat occluded = grid.clone();
  cv::rectangle(occluded, cv::Rect(40, 45, 170, 150), cv::Scalar(0, 0, 0), cv::FILLED);
  const auto result = mapper.process(occluded);
  EXPECT_FALSE(result.geometry_valid);
  EXPECT_FALSE(result.stable);
}

TEST(GridMapper, ValidatesConfigurationAndInput)
{
  auv_mapping::GridMapperConfig config;
  config.morphology_kernel = 4;
  EXPECT_THROW(
    {
      const auv_mapping::GridMapper invalid_mapper(config);
      (void)invalid_mapper;
    },
    std::invalid_argument);
  auv_mapping::GridMapper mapper;
  EXPECT_THROW(mapper.process(cv::Mat()), std::invalid_argument);
}

}  // namespace
