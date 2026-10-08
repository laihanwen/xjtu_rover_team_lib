#pragma once
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

// Keep the original K and full frame size, so corrected pixels use K and zero D.
// Each capture worker owns its maps; raw capture frames remain available to odometry.
class CameraRectifier {
 public:
  CameraRectifier(bool enabled, int width, int height,
                  const std::vector<double>& k, const std::vector<double>& d)
      : enabled_(enabled), size_(width, height) {
    if (!enabled_) return;
    if (k.size()!=9 || d.empty() || width<=0 || height<=0)
      throw std::runtime_error("rectification requires intrinsics, distortion and image size");
    cv::Mat camera(3,3,CV_64F,const_cast<double*>(k.data()));
    cv::initUndistortRectifyMap(camera,cv::Mat(d),cv::Mat(),camera,size_,
                              CV_16SC2,map1_,map2_);
    cv::Mat full(size_,CV_8UC1,cv::Scalar(255));
    cv::remap(full,valid_,map1_,map2_,cv::INTER_LINEAR,cv::BORDER_CONSTANT);
    cv::threshold(valid_,valid_,254,255,cv::THRESH_BINARY);
    // Exclude black borders and patches crossing the remap boundary from flow.
    cv::erode(valid_,valid_,cv::getStructuringElement(cv::MORPH_RECT,{21,21}),
              {-1,-1},1,cv::BORDER_CONSTANT,cv::Scalar(0));
  }
  cv::Mat apply(const cv::Mat& raw) const {
    if (!enabled_) return raw;
    if (raw.size()!=size_) throw std::runtime_error("camera calibration resolution mismatch");
    cv::Mat corrected;
    cv::remap(raw,corrected,map1_,map2_,cv::INTER_LINEAR,cv::BORDER_CONSTANT);
    return corrected;
  }
  const cv::Mat& valid_mask() const { return valid_; }
 private:
  bool enabled_;
  cv::Size size_;
  cv::Mat map1_,map2_,valid_;
};
