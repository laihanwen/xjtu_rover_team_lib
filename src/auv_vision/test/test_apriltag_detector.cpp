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

#include <gtest/gtest.h>

#include <stdexcept>

#include "auv_vision/apriltag_detector.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/core.hpp"

TEST(AprilTagDetector, DetectsGeneratedTag36h11)
{
  constexpr int tag_id = 7;
  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  cv::Mat marker;
  cv::aruco::generateImageMarker(dictionary, tag_id, 200, marker, 1);

  cv::Mat scene(400, 400, CV_8UC1, cv::Scalar(255));
  marker.copyTo(scene(cv::Rect(100, 100, marker.cols, marker.rows)));

  auv_vision::AprilTagDetector detector("tag36h11");
  const auto observations = detector.detect(scene);
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations.front().id, tag_id);
  EXPECT_NEAR(observations.front().center.x, 199.5F, 2.0F);
  EXPECT_NEAR(observations.front().center.y, 199.5F, 2.0F);
}

TEST(AprilTagDetector, RejectsUnknownFamily)
{
  EXPECT_THROW(auv_vision::AprilTagDetector("unknown"), std::invalid_argument);
}

TEST(AprilTagDetector, EmptyImageHasNoDetections)
{
  auv_vision::AprilTagDetector detector;
  EXPECT_TRUE(detector.detect(cv::Mat{}).empty());
}
