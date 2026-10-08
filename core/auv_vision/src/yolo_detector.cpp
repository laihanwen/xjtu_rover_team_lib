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

#include "auv_vision/yolo_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "openssl/evp.h"
#include "opencv2/imgproc.hpp"

namespace auv_vision
{

namespace
{

void validate_config(const YoloDetectorConfig & config)
{
  if (config.input_width <= 0 || config.input_height <= 0 || config.class_names.empty()) {
    throw std::invalid_argument("YOLO input dimensions and class names must be valid");
  }
  if (config.confidence_threshold <= 0.0F || config.confidence_threshold > 1.0F ||
    config.nms_threshold <= 0.0F || config.nms_threshold > 1.0F)
  {
    throw std::invalid_argument("YOLO confidence and NMS thresholds must be within (0, 1]");
  }
}

}  // namespace

std::string YoloDetector::file_sha256(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("unable to open model for SHA-256: " + path);
  }
  EVP_MD_CTX * context = EVP_MD_CTX_new();
  if (context == nullptr || EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("unable to initialize SHA-256");
  }
  std::array<char, 1024 * 1024> buffer{};
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = stream.gcount();
    if (count > 0 && EVP_DigestUpdate(context, buffer.data(),
        static_cast<std::size_t>(count)) != 1)
    {
      EVP_MD_CTX_free(context);
      throw std::runtime_error("unable to update SHA-256");
    }
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0U;
  if (EVP_DigestFinal_ex(context, digest.data(), &digest_size) != 1) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("unable to finalize SHA-256");
  }
  EVP_MD_CTX_free(context);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0U; index < digest_size; ++index) {
    output << std::setw(2) << static_cast<int>(digest[index]);
  }
  return output.str();
}

YoloDetector::YoloDetector(
  const std::string & model_path, const std::string & expected_sha256,
  YoloDetectorConfig config)
: config_(std::move(config))
{
  validate_config(config_);
  if (model_path.empty() || !std::filesystem::is_regular_file(model_path)) {
    throw std::invalid_argument("YOLO model_path must reference an existing file");
  }
  if (expected_sha256.size() != 64U) {
    throw std::invalid_argument("model_sha256 must contain exactly 64 hexadecimal characters");
  }
  if (file_sha256(model_path) != expected_sha256) {
    throw std::invalid_argument("YOLO model SHA-256 does not match model_sha256");
  }
  network_ = cv::dnn::readNetFromONNX(model_path);
  if (network_.empty()) {
    throw std::runtime_error("OpenCV could not load the ONNX model");
  }
  network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  network_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

cv::Mat YoloDetector::letterbox(
  const cv::Mat & image, int width, int height, LetterboxTransform & transform)
{
  if (image.empty() || image.type() != CV_8UC3 || width <= 0 || height <= 0) {
    throw std::invalid_argument("letterbox requires a non-empty BGR8 image and positive size");
  }
  transform.scale = std::min(
    static_cast<float>(width) / image.cols,
    static_cast<float>(height) / image.rows);
  const int resized_width = static_cast<int>(std::round(image.cols * transform.scale));
  const int resized_height = static_cast<int>(std::round(image.rows * transform.scale));
  const int left = (width - resized_width) / 2;
  const int top = (height - resized_height) / 2;
  transform.pad_x = static_cast<float>(left);
  transform.pad_y = static_cast<float>(top);
  transform.original_width = image.cols;
  transform.original_height = image.rows;
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(resized_width, resized_height));
  cv::Mat output(height, width, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(output(cv::Rect(left, top, resized_width, resized_height)));
  return output;
}

std::vector<YoloDetection> YoloDetector::decode(
  const cv::Mat & output, const LetterboxTransform & transform,
  const YoloDetectorConfig & config)
{
  validate_config(config);
  if (output.empty() || output.type() != CV_32F || output.dims != 3 ||
    output.size[0] != 1)
  {
    throw std::invalid_argument("YOLO output must be a float tensor with shape [1,C,N]");
  }
  const int attributes = 4 + static_cast<int>(config.class_names.size());
  bool channels_first = output.size[1] == attributes;
  if (!channels_first && output.size[2] != attributes) {
    throw std::invalid_argument("YOLO output class count does not match class_names");
  }
  const int candidates = channels_first ? output.size[2] : output.size[1];
  const float * values = output.ptr<float>();
  const auto value_at = [&](int candidate, int attribute) {
      return channels_first ? values[attribute * candidates + candidate] :
             values[candidate * attributes + attribute];
    };
  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> class_ids;
  for (int candidate = 0; candidate < candidates; ++candidate) {
    int best_class = 0;
    float best_confidence = value_at(candidate, 4);
    for (int class_id = 1; class_id < static_cast<int>(config.class_names.size()); ++class_id) {
      const float confidence = value_at(candidate, 4 + class_id);
      if (confidence > best_confidence) {
        best_confidence = confidence;
        best_class = class_id;
      }
    }
    if (best_confidence < config.confidence_threshold) {
      continue;
    }
    const float center_x = (value_at(candidate, 0) - transform.pad_x) / transform.scale;
    const float center_y = (value_at(candidate, 1) - transform.pad_y) / transform.scale;
    const float width = value_at(candidate, 2) / transform.scale;
    const float height = value_at(candidate, 3) / transform.scale;
    const int left = std::max(0, static_cast<int>(std::round(center_x - width / 2.0F)));
    const int top = std::max(0, static_cast<int>(std::round(center_y - height / 2.0F)));
    const int right = std::min(
      transform.original_width, static_cast<int>(std::round(center_x + width / 2.0F)));
    const int bottom = std::min(
      transform.original_height, static_cast<int>(std::round(center_y + height / 2.0F)));
    if (right <= left || bottom <= top) {
      continue;
    }
    boxes.emplace_back(left, top, right - left, bottom - top);
    confidences.push_back(best_confidence);
    class_ids.push_back(best_class);
  }

  std::vector<YoloDetection> detections;
  for (int class_id = 0; class_id < static_cast<int>(config.class_names.size()); ++class_id) {
    std::vector<cv::Rect> class_boxes;
    std::vector<float> class_confidences;
    std::vector<std::size_t> source_indexes;
    for (std::size_t index = 0U; index < boxes.size(); ++index) {
      if (class_ids[index] == class_id) {
        class_boxes.push_back(boxes[index]);
        class_confidences.push_back(confidences[index]);
        source_indexes.push_back(index);
      }
    }
    std::vector<int> kept;
    cv::dnn::NMSBoxes(
      class_boxes, class_confidences, config.confidence_threshold,
      config.nms_threshold, kept);
    for (const int local_index : kept) {
      const std::size_t source = source_indexes[static_cast<std::size_t>(local_index)];
      detections.push_back({
          class_id, config.class_names[static_cast<std::size_t>(class_id)],
          confidences[source], boxes[source]});
    }
  }
  std::sort(
    detections.begin(), detections.end(),
    [](const YoloDetection & left, const YoloDetection & right) {
      return left.confidence > right.confidence;
    });
  return detections;
}

std::vector<YoloDetection> YoloDetector::detect(const cv::Mat & image)
{
  LetterboxTransform transform;
  const cv::Mat prepared = letterbox(
    image, config_.input_width, config_.input_height, transform);
  cv::Mat blob = cv::dnn::blobFromImage(
    prepared, 1.0 / 255.0, cv::Size(config_.input_width, config_.input_height),
    cv::Scalar(), true, false, CV_32F);
  network_.setInput(blob);
  std::vector<cv::Mat> outputs;
  network_.forward(outputs, network_.getUnconnectedOutLayersNames());
  if (outputs.size() != 1U) {
    throw std::runtime_error("YOLO model must expose exactly one detection output");
  }
  return decode(outputs.front(), transform, config_);
}

}  // namespace auv_vision
