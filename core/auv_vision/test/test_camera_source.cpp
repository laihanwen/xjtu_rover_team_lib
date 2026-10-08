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

#include <filesystem>
#include <stdexcept>
#include <string>

#include "auv_vision/camera_source.hpp"
#include "gtest/gtest.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"

namespace
{

TEST(CameraSource, RejectsInvalidSettings)
{
  auv_vision::CameraSourceConfig config;
  config.width = 0;
  EXPECT_THROW(auv_vision::CameraSource source(config), std::invalid_argument);
  config.width = 640;
  config.pixel_format = "BAD";
  EXPECT_THROW(auv_vision::CameraSource source(config), std::invalid_argument);
}

TEST(CameraSource, EmptySourceStaysClosed)
{
  auv_vision::CameraSource source(auv_vision::CameraSourceConfig{});
  EXPECT_FALSE(source.open());
  EXPECT_FALSE(source.is_open());
}

TEST(CameraSource, StableV4lPathIsTreatedAsLive)
{
  auv_vision::CameraSourceConfig config;
  config.source = "/dev/v4l/by-id/auv-camera-that-does-not-exist";
  auv_vision::CameraSource source(config);
  EXPECT_FALSE(source.open());
  EXPECT_TRUE(source.is_live());
}

TEST(CameraSource, ReadsAndLoopsAnImageSequence)
{
  const auto directory = std::filesystem::temp_directory_path() / "auv_camera_source_test";
  std::filesystem::create_directories(directory);
  const cv::Mat first(8, 12, CV_8UC3, cv::Scalar(10, 20, 30));
  const cv::Mat second(8, 12, CV_8UC3, cv::Scalar(40, 50, 60));
  ASSERT_TRUE(cv::imwrite((directory / "frame_00.png").string(), first));
  ASSERT_TRUE(cv::imwrite((directory / "frame_01.png").string(), second));

  auv_vision::CameraSourceConfig config;
  config.source = (directory / "frame_%02d.png").string();
  config.loop = true;
  auv_vision::CameraSource source(config);
  ASSERT_TRUE(source.open());
  EXPECT_FALSE(source.is_live());

  cv::Mat frame;
  ASSERT_TRUE(source.read(frame));
  EXPECT_EQ(frame.at<cv::Vec3b>(0, 0), cv::Vec3b(10, 20, 30));
  ASSERT_TRUE(source.read(frame));
  EXPECT_EQ(frame.at<cv::Vec3b>(0, 0), cv::Vec3b(40, 50, 60));
  ASSERT_TRUE(source.read(frame));
  EXPECT_EQ(frame.at<cv::Vec3b>(0, 0), cv::Vec3b(10, 20, 30));

  source.close();
  std::filesystem::remove_all(directory);
}

}  // namespace
