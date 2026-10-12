#pragma once
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <cmath>

namespace auv_mapping {
// Image-space ridge extraction. It rejects broad floor colour patches and
// finds pipe centres rather than opposite colour-boundary edges. Width is a
// detector pixel budget, not an invented physical pipe diameter.
inline cv::Mat pipe_centerlines(const cv::Mat& mask,float maximum_radius=10) {
  if(mask.empty()||mask.type()!=CV_8UC1||!std::isfinite(maximum_radius)||maximum_radius<2)
    throw std::invalid_argument("invalid pipe centre mask or radius");
  cv::Mat distance,peaks,ridge;
  cv::distanceTransform(mask,distance,cv::DIST_L2,3);
  cv::dilate(distance,peaks,cv::getStructuringElement(cv::MORPH_RECT,{3,3}));
  ridge=cv::Mat::zeros(mask.size(),CV_8U);
  for(int y=1;y<mask.rows-1;++y)for(int x=1;x<mask.cols-1;++x) {
    const float d=distance.at<float>(y,x);
    if(d>=1.5F&&d<=maximum_radius&&d>=peaks.at<float>(y,x)-.05F)
      ridge.at<unsigned char>(y,x)=255;
  }
  // A bounded gap close keeps lines intact around a measured junction.
  cv::morphologyEx(ridge,ridge,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,{3,3}));
  return ridge;
}
}
