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

#ifndef AUV_VISION__YOLO_DETECTOR_HPP_
#define AUV_VISION__YOLO_DETECTOR_HPP_

#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/dnn.hpp"

namespace auv_vision
{

struct YoloDetection
{
  int class_id{-1};
  std::string class_name;
  float confidence{0.0F};
  cv::Rect2f box;
};

struct YoloDetectorConfig
{
  int input_width{640};
  int input_height{640};
  float confidence_threshold{0.5F};
  float nms_threshold{0.45F};
  std::vector<std::string> class_names{"sea_cucumber", "turtle", "starfish"};
};

struct LetterboxTransform
{
  float scale{1.0F};
  float pad_x{0.0F};
  float pad_y{0.0F};
  int original_width{0};
  int original_height{0};
};

class YoloDetector
{
public:
  YoloDetector(
    const std::string & model_path, const std::string & expected_sha256,
    YoloDetectorConfig config = {});

  std::vector<YoloDetection> detect(const cv::Mat & image);
  static cv::Mat letterbox(
    const cv::Mat & image, int width, int height, LetterboxTransform & transform);
  static std::vector<YoloDetection> decode(
    const cv::Mat & output, const LetterboxTransform & transform,
    const YoloDetectorConfig & config);
  static std::string file_sha256(const std::string & path);

private:
  YoloDetectorConfig config_;
  cv::dnn::Net network_;
};

}  // namespace auv_vision

#endif  // AUV_VISION__YOLO_DETECTOR_HPP_
