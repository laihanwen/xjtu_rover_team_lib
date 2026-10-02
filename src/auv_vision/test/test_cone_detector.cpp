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

#include <algorithm>
#include <vector>

#include "auv_vision/cone_detector.hpp"
#include "gtest/gtest.h"
#include "opencv2/imgproc.hpp"

namespace
{

cv::Mat make_cone_scene()
{
  cv::Mat image(600, 600, CV_8UC3, cv::Scalar(180, 180, 180));
  for (int division = 0; division <= 3; ++division) {
    const int coordinate = std::min(599, division * 200);
    cv::line(
      image, cv::Point(coordinate, 0), cv::Point(coordinate, 599),
      cv::Scalar(0, 255, 255), 10);
    cv::line(
      image, cv::Point(0, coordinate), cv::Point(599, coordinate),
      cv::Scalar(0, 255, 255), 10);
  }
  const cv::Scalar cone_color(0, 80, 255);
  cv::circle(image, cv::Point(100, 100), 45, cone_color, cv::FILLED, cv::LINE_AA);
  cv::rectangle(image, cv::Rect(460, 260, 80, 80), cone_color, cv::FILLED);
  return image;
}

auv_vision::ConeObservation make_observation(
  const auv_vision::ConeShape shape, const int row, const int col,
  const float confidence = 0.9F)
{
  auv_vision::ConeObservation observation;
  observation.shape = shape;
  observation.row = row;
  observation.col = col;
  observation.center = cv::Point2f(col * 200.0F + 100.0F, row * 200.0F + 100.0F);
  observation.confidence = confidence;
  return observation;
}

TEST(ConeDetector, DetectsCircleAndSquareInCorrectCells)
{
  const auv_vision::ConeDetector detector;
  const auto result = detector.process(make_cone_scene());
  ASSERT_EQ(result.observations.size(), 2U);
  const auto circle = std::find_if(
    result.observations.begin(), result.observations.end(), [](const auto & observation) {
      return observation.shape == auv_vision::ConeShape::kCircle;
    });
  const auto square = std::find_if(
    result.observations.begin(), result.observations.end(), [](const auto & observation) {
      return observation.shape == auv_vision::ConeShape::kSquare;
    });
  ASSERT_NE(circle, result.observations.end());
  EXPECT_EQ(circle->row, 0);
  EXPECT_EQ(circle->col, 0);
  EXPECT_GT(circle->circularity, 0.85F);
  ASSERT_NE(square, result.observations.end());
  EXPECT_EQ(square->row, 1);
  EXPECT_EQ(square->col, 2);
  EXPECT_GT(square->confidence, 0.75F);
  EXPECT_EQ(result.mask.size(), cv::Size(600, 600));
  EXPECT_EQ(result.debug_image.size(), cv::Size(600, 600));
}

TEST(ConeDetector, RejectsSmallAndMarginObjects)
{
  cv::Mat image(600, 600, CV_8UC3, cv::Scalar(180, 180, 180));
  const cv::Scalar cone_color(0, 80, 255);
  cv::circle(image, cv::Point(100, 100), 5, cone_color, cv::FILLED);
  cv::rectangle(image, cv::Rect(190, 260, 45, 70), cone_color, cv::FILLED);
  const auv_vision::ConeDetector detector;
  EXPECT_TRUE(detector.process(image).observations.empty());
}

TEST(ConeDetector, ValidatesInputAndConfiguration)
{
  const auv_vision::ConeDetector detector;
  EXPECT_THROW(detector.process(cv::Mat()), std::invalid_argument);
  auv_vision::ConeDetectorConfig config;
  config.morphology_kernel = 4;
  EXPECT_THROW(
    {
      const auv_vision::ConeDetector invalid(config);
      (void)invalid;
    },
    std::invalid_argument);
}

TEST(ConeTracker, RequiresVotesAndClearsStaleShape)
{
  auv_vision::ConeTracker tracker;
  const std::vector<auv_vision::ConeObservation> circle{
    make_observation(auv_vision::ConeShape::kCircle, 0, 1)};
  EXPECT_TRUE(tracker.update(circle).empty());
  EXPECT_TRUE(tracker.update(circle).empty());
  auto stable = tracker.update(circle);
  ASSERT_EQ(stable.size(), 1U);
  EXPECT_EQ(stable.front().shape, auv_vision::ConeShape::kCircle);

  EXPECT_EQ(tracker.update({}).size(), 1U);
  EXPECT_EQ(tracker.update({}).size(), 1U);
  EXPECT_TRUE(tracker.update({}).empty());
}

TEST(ConeTracker, SwitchesOnlyAfterNewShapeWins)
{
  auv_vision::ConeTracker tracker;
  const std::vector<auv_vision::ConeObservation> circle{
    make_observation(auv_vision::ConeShape::kCircle, 2, 2)};
  const std::vector<auv_vision::ConeObservation> square{
    make_observation(auv_vision::ConeShape::kSquare, 2, 2)};
  tracker.update(circle);
  tracker.update(circle);
  tracker.update(circle);
  EXPECT_EQ(tracker.update(square).front().shape, auv_vision::ConeShape::kCircle);
  EXPECT_EQ(tracker.update(square).front().shape, auv_vision::ConeShape::kCircle);
  EXPECT_EQ(tracker.update(square).front().shape, auv_vision::ConeShape::kSquare);
}

}  // namespace
