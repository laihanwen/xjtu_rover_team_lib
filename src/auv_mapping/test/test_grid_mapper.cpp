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

// Competition scene constants: a dark blue pool floor, white edges and
// internal divisions, and a single yellow bottom-reference edge.
auv_mapping::GridMapperConfig competition_config() {
  auv_mapping::GridMapperConfig config; config.single_yellow_edge=true; config.white_grid_edges=true;
  return config;
}
const cv::Scalar kPoolFloor(120, 60, 20);
const cv::Scalar kWhite(255, 255, 255);
const cv::Scalar kYellow(0, 255, 255);

// Warp a canonical 500x500 grid onto a larger dark-blue canvas so the mapper
// must recover the quadrilateral under perspective rather than read an
// axis-aligned rectangle.
cv::Mat warp_to_perspective(const cv::Mat & canonical)
{
  const std::array<cv::Point2f, 4> source{
    cv::Point2f(0.0F, 0.0F), cv::Point2f(499.0F, 0.0F),
    cv::Point2f(499.0F, 499.0F), cv::Point2f(0.0F, 499.0F)};
  const std::array<cv::Point2f, 4> destination{
    cv::Point2f(115.0F, 80.0F), cv::Point2f(550.0F, 115.0F),
    cv::Point2f(590.0F, 430.0F), cv::Point2f(70.0F, 455.0F)};
  const cv::Mat transform = cv::getPerspectiveTransform(source.data(), destination.data());
  cv::Mat perspective(520, 660, CV_8UC3, kPoolFloor);
  cv::warpPerspective(
    canonical, perspective, transform, perspective.size(), cv::INTER_LINEAR,
    cv::BORDER_TRANSPARENT);
  return perspective;
}

// Draw the canonical 500x500 grid: three white edges plus one yellow edge
// (yellow_side: 0=top, 1=right, 2=bottom, 3=left), with white 2x2 internal
// divisions, over a dark-blue pool floor.
cv::Mat make_canonical_grid(const bool draw_internal_lines, const int yellow_side)
{
  cv::Mat canonical(500, 500, CV_8UC3, kPoolFloor);
  cv::rectangle(canonical, cv::Rect(8, 8, 484, 484), kWhite, 18);
  if (yellow_side == 0) {
    cv::line(canonical, cv::Point(8, 8), cv::Point(492, 8), kYellow, 18);
  } else if (yellow_side == 1) {
    cv::line(canonical, cv::Point(492, 8), cv::Point(492, 492), kYellow, 18);
  } else if (yellow_side == 3) {
    cv::line(canonical, cv::Point(8, 8), cv::Point(8, 492), kYellow, 18);
  } else {
    cv::line(canonical, cv::Point(8, 492), cv::Point(492, 492), kYellow, 18);
  }
  if (draw_internal_lines) {
    cv::line(canonical, cv::Point(167, 10), cv::Point(167, 490), kWhite, 8);
    cv::line(canonical, cv::Point(333, 10), cv::Point(333, 490), kWhite, 8);
    cv::line(canonical, cv::Point(10, 167), cv::Point(490, 167), kWhite, 8);
    cv::line(canonical, cv::Point(10, 333), cv::Point(490, 333), kWhite, 8);
  }
  return canonical;
}

cv::Mat make_grid(const bool draw_internal_lines = true, const int yellow_side = 2)
{
  return warp_to_perspective(make_canonical_grid(draw_internal_lines, yellow_side));
}

// A legacy-style grid whose four border edges are all yellow: no single edge
// stands out, so yellow-edge orientation must stay unconfirmed indefinitely.
cv::Mat make_all_yellow_grid()
{
  cv::Mat canonical = make_canonical_grid(true, 2);
  cv::line(canonical, cv::Point(8, 8), cv::Point(492, 8), kYellow, 18);
  cv::line(canonical, cv::Point(492, 8), cv::Point(492, 492), kYellow, 18);
  cv::line(canonical, cv::Point(8, 8), cv::Point(8, 492), kYellow, 18);
  return warp_to_perspective(canonical);
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

TEST(GridMapper, RequiresConsecutiveStableFrames)
{
  auv_mapping::GridMapper mapper(competition_config());
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

TEST(GridMapper, ConfirmsYellowOrientationOverConsecutiveFrames)
{
  auv_mapping::GridMapper mapper(competition_config());
  const cv::Mat image = make_grid(true, 0);
  const auto first = mapper.process(image);
  const auto second = mapper.process(image);
  const auto third = mapper.process(image);
  EXPECT_TRUE(first.geometry_valid) << first.reason;
  EXPECT_FALSE(first.orientation_valid);
  EXPECT_TRUE(second.geometry_valid);
  EXPECT_FALSE(second.orientation_valid);
  EXPECT_TRUE(third.geometry_valid);
  EXPECT_TRUE(third.orientation_valid);
}

TEST(GridMapper, RejectsGridWithoutInternalGrid)
{
  auv_mapping::GridMapper mapper(competition_config());
  const auto result = mapper.process(make_grid(false));
  EXPECT_FALSE(result.geometry_valid);
  EXPECT_FALSE(result.stable);
  EXPECT_EQ(result.reason, "internal grid lines missing");
}

TEST(GridMapper, OrientsYellowEdgeToBottom)
{
  // Whichever of the four sides carries the yellow edge, after rectification
  // it must land on the bottom so the generated map uses the yellow edge as
  // its bottom edge.
  for (int yellow_side = 0; yellow_side < 4; ++yellow_side) {
    auv_mapping::GridMapper mapper(competition_config());
    const auto result = mapper.process(make_grid(true, yellow_side));
    ASSERT_TRUE(result.geometry_valid) << result.reason << " side=" << yellow_side;

    cv::Mat hsv;
    cv::cvtColor(result.rectified, hsv, cv::COLOR_BGR2HSV);
    cv::Mat bottom_yellow;
    cv::Mat top_yellow;
    cv::inRange(
      hsv(cv::Rect(0, 555, 600, 45)), cv::Scalar(15, 60, 60), cv::Scalar(40, 255, 255),
      bottom_yellow);
    cv::inRange(
      hsv(cv::Rect(0, 0, 600, 45)), cv::Scalar(15, 60, 60), cv::Scalar(40, 255, 255),
      top_yellow);
    const double bottom_ratio =
      static_cast<double>(cv::countNonZero(bottom_yellow)) / bottom_yellow.total();
    const double top_ratio =
      static_cast<double>(cv::countNonZero(top_yellow)) / top_yellow.total();
    EXPECT_GT(bottom_ratio, 0.2) << "yellow edge should map to the bottom band (side="
                                 << yellow_side << ")";
    EXPECT_LT(top_ratio, 0.1) << "top band should be a white edge, not yellow (side="
                              << yellow_side << ")";
  }
}

TEST(GridMapper, KeepsOrientationUnconfirmedWhenNoSingleYellowEdge)
{
  auv_mapping::GridMapper mapper(competition_config());
  const cv::Mat image = make_all_yellow_grid();
  for (int frame = 0; frame < 5; ++frame) {
    const auto result = mapper.process(image);
    EXPECT_FALSE(result.geometry_valid) << "ambiguous yellow must invalidate geometry";
    EXPECT_FALSE(result.orientation_valid)
      << "all-yellow border must not confirm orientation (frame=" << frame << ")";
  }
}

TEST(GridMapper, RejectsBlankFrameAndResetsState)
{
  auv_mapping::GridMapper mapper(competition_config());
  const cv::Mat grid = make_grid();
  for (int frame = 0; frame < 3; ++frame) {
    mapper.process(grid);
  }
  const cv::Mat blank(grid.size(), grid.type(), cv::Scalar(0, 0, 0));
  const auto blank_result = mapper.process(blank);
  EXPECT_FALSE(blank_result.geometry_valid);
  EXPECT_FALSE(blank_result.stable);
  EXPECT_FALSE(blank_result.orientation_valid);
  const auto after = mapper.process(grid);
  EXPECT_TRUE(after.geometry_valid) << after.reason;
  EXPECT_FALSE(after.stable) << "stability must restart after a rejected frame";
  EXPECT_FALSE(after.orientation_valid) << "orientation count must restart after a rejected frame";
}

TEST(GridMapper, HandlesBrightnessBlurAndModerateNoise)
{
  cv::Mat degraded;
  make_grid().convertTo(degraded, -1, 0.72, 8.0);
  cv::GaussianBlur(degraded, degraded, cv::Size(5, 5), 1.2);
  cv::Mat noise(degraded.size(), degraded.type());
  cv::randn(noise, cv::Scalar::all(0), cv::Scalar::all(5));
  cv::add(degraded, noise, degraded, cv::noArray(), degraded.type());

  auv_mapping::GridMapper mapper(competition_config());
  const auto result = mapper.process(degraded);
  EXPECT_TRUE(result.geometry_valid) << result.reason;
}

TEST(GridMapper, RejectsOccludedBorderAndResetsCornerStability)
{
  auv_mapping::GridMapper mapper(competition_config());
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

TEST(GridMapper, ReportsCameraPositionAndConfidence)
{
  auv_mapping::GridMapper mapper(competition_config());
  auv_mapping::GridResult result;
  for (int frame=0; frame<3; ++frame) result = mapper.process(make_grid());
  ASSERT_TRUE(result.geometry_valid) << result.reason;
  EXPECT_TRUE(result.position_valid);
  EXPECT_GE(result.camera_row, 0.0F);
  EXPECT_LE(result.camera_row, 3.0F);
  EXPECT_GE(result.camera_col, 0.0F);
  EXPECT_LE(result.camera_col, 3.0F);
  EXPECT_GE(result.confidence, 0.0F);
  EXPECT_LE(result.confidence, 1.0F);
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
  auv_mapping::GridMapper mapper(competition_config());
  EXPECT_THROW(mapper.process(cv::Mat()), std::invalid_argument);
}

}  // namespace
