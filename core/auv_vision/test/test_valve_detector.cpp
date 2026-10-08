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

#include <cmath>
#include <limits>
#include <stdexcept>

#include "auv_vision/valve_detector.hpp"
#include "gtest/gtest.h"
#include "opencv2/imgproc.hpp"

TEST(ValveDetector, RejectsInvalidInputAndConfiguration)
{
  EXPECT_THROW(auv_vision::ValveDetector().process(cv::Mat()), std::invalid_argument);
  auv_vision::ValveDetectorConfig config;
  config.blur_kernel = 4;
  EXPECT_THROW(auv_vision::ValveDetector detector(config), std::invalid_argument);
  config.blur_kernel = 5;
  config.minimum_confidence = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(auv_vision::ValveDetector detector(config), std::invalid_argument);
}

TEST(ValveDetector, RejectsNonCircularTarget)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::rectangle(image, cv::Rect(170, 190, 300, 100), cv::Scalar(255, 255, 255), 8);
  const auto result = auv_vision::ValveDetector().process(image);
  EXPECT_FALSE(result.observation.detected);
  EXPECT_FALSE(result.observation.handle_angle_valid);
}

TEST(ValveDetector, CircleWithoutHandleHasNoAngle)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::circle(image, cv::Point(320, 240), 90, cv::Scalar(255, 255, 255), 8);
  const auto result = auv_vision::ValveDetector().process(image);
  ASSERT_TRUE(result.observation.detected);
  EXPECT_FALSE(result.observation.handle_angle_valid);
}

TEST(ValveDetector, ReportsNoTargetInBlankImage)
{
  const cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  const auto result = auv_vision::ValveDetector().process(image);
  EXPECT_FALSE(result.observation.detected);
  EXPECT_FALSE(result.debug_image.empty());
}

TEST(ValveDetector, DetectsCircularValveAndHandle)
{
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  const cv::Point center(320, 240);
  cv::circle(image, center, 90, cv::Scalar(255, 255, 255), 8);
  cv::line(image, cv::Point(250, 240), cv::Point(390, 240), cv::Scalar(255, 255, 255), 8);

  const auto result = auv_vision::ValveDetector().process(image);
  ASSERT_TRUE(result.observation.detected);
  EXPECT_NEAR(result.observation.center.x, center.x, 8.0);
  EXPECT_NEAR(result.observation.center.y, center.y, 8.0);
  EXPECT_GT(result.observation.radius, 75.0F);
  EXPECT_TRUE(result.observation.handle_angle_valid);
  EXPECT_NEAR(result.observation.handle_angle_rad, 0.0F, 0.20F);
}

TEST(ValveDetector, TracksRepresentativeHandleAngles)
{
  const cv::Point center(320, 240);
  for (const double expected : {-1.0, -0.5, 0.5, 1.0}) {
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::circle(image, center, 90, cv::Scalar(255, 255, 255), 8);
    const cv::Point offset(
      cvRound(70.0 * std::cos(expected)), cvRound(70.0 * std::sin(expected)));
    cv::line(image, center - offset, center + offset, cv::Scalar(255, 255, 255), 8);
    const auto result = auv_vision::ValveDetector().process(image);
    ASSERT_TRUE(result.observation.detected);
    ASSERT_TRUE(result.observation.handle_angle_valid);
    EXPECT_NEAR(result.observation.handle_angle_rad, expected, 0.20F);
  }
}
