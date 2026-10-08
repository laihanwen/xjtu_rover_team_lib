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
#include <fstream>
#include <stdexcept>

#include "auv_vision/yolo_detector.hpp"
#include "gtest/gtest.h"

namespace
{

TEST(YoloDetector, LetterboxPreservesAspectRatio)
{
  cv::Mat image(100, 200, CV_8UC3, cv::Scalar(1, 2, 3));
  auv_vision::LetterboxTransform transform;
  const cv::Mat output = auv_vision::YoloDetector::letterbox(image, 640, 640, transform);
  EXPECT_EQ(output.cols, 640);
  EXPECT_EQ(output.rows, 640);
  EXPECT_FLOAT_EQ(transform.scale, 3.2F);
  EXPECT_FLOAT_EQ(transform.pad_x, 0.0F);
  EXPECT_FLOAT_EQ(transform.pad_y, 160.0F);
  EXPECT_EQ(transform.original_width, 200);
  EXPECT_EQ(transform.original_height, 100);
}

TEST(YoloDetector, DecodesClassesClipsBoxesAndAppliesPerClassNms)
{
  const int dimensions[] = {1, 7, 3};
  cv::Mat output(3, dimensions, CV_32F, cv::Scalar(0));
  float * values = output.ptr<float>();
  const auto set_value = [values](int attribute, int candidate, float value) {
      values[attribute * 3 + candidate] = value;
    };
  set_value(0, 0, 320.0F);
  set_value(1, 0, 320.0F);
  set_value(2, 0, 200.0F);
  set_value(3, 0, 100.0F);
  set_value(4, 0, 0.9F);
  set_value(0, 1, 322.0F);
  set_value(1, 1, 320.0F);
  set_value(2, 1, 200.0F);
  set_value(3, 1, 100.0F);
  set_value(4, 1, 0.8F);
  set_value(0, 2, 630.0F);
  set_value(1, 2, 320.0F);
  set_value(2, 2, 80.0F);
  set_value(3, 2, 80.0F);
  set_value(5, 2, 0.75F);

  auv_vision::LetterboxTransform transform;
  transform.scale = 1.0F;
  transform.original_width = 640;
  transform.original_height = 640;
  auv_vision::YoloDetectorConfig config;
  const auto detections = auv_vision::YoloDetector::decode(output, transform, config);
  ASSERT_EQ(detections.size(), 2U);
  EXPECT_EQ(detections[0].class_name, "sea_cucumber");
  EXPECT_FLOAT_EQ(detections[0].confidence, 0.9F);
  EXPECT_EQ(detections[1].class_name, "turtle");
  EXPECT_LE(detections[1].box.x + detections[1].box.width, 640.0F);
}

TEST(YoloDetector, RejectsWrongOutputShapeAndConfiguration)
{
  auv_vision::LetterboxTransform transform;
  auv_vision::YoloDetectorConfig config;
  EXPECT_THROW(
    auv_vision::YoloDetector::decode(cv::Mat::zeros(2, 2, CV_32F), transform, config),
    std::invalid_argument);
  config.class_names.clear();
  const int dimensions[] = {1, 7, 1};
  cv::Mat output(3, dimensions, CV_32F, cv::Scalar(0));
  EXPECT_THROW(
    auv_vision::YoloDetector::decode(output, transform, config),
    std::invalid_argument);
}

TEST(YoloDetector, ComputesSha256AndRejectsMissingModel)
{
  const auto temporary = std::filesystem::temp_directory_path() / "auv_yolo_sha_test.txt";
  {
    std::ofstream stream(temporary);
    stream << "abc";
  }
  EXPECT_EQ(
    auv_vision::YoloDetector::file_sha256(temporary.string()),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  std::filesystem::remove(temporary);
  EXPECT_THROW(
    auv_vision::YoloDetector("/missing/model.onnx", std::string(64U, '0')),
    std::invalid_argument);
}

}  // namespace
