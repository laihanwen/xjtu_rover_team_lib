#pragma once
#include <opencv2/imgproc.hpp>
#include <algorithm>

// Passive cue only. A yellow stripe candidate is not a verified 3x3 boundary.
class YellowEdgeWatch {
 public:
  struct Result {bool candidate{},confirmed{},newly_confirmed{};double length_px{};};
  Result update(const cv::Mat& bgr,const cv::Scalar& lower,const cv::Scalar& upper){
    Result result;
    if(bgr.empty())return result;
    cv::Mat hsv,mask;
    cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);
    cv::inRange(hsv,lower,upper,mask);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    const double minimum_length=.20*std::min(bgr.cols,bgr.rows);
    for(const auto& contour:contours){
      const auto box=cv::minAreaRect(contour);
      const double length=std::max(box.size.width,box.size.height);
      const double width=std::min(box.size.width,box.size.height);
      const double area=cv::contourArea(contour);
      if(length<minimum_length||width<2||length<4*width||area<60||
         area>.10*bgr.total()||area/(std::max(1.,length*width))<.30)continue;
      result.candidate=true;result.length_px=std::max(result.length_px,length);
    }
    votes_=result.candidate?std::min(votes_+1,3):0;
    result.newly_confirmed=votes_==3&&!confirmed_;
    confirmed_|=result.newly_confirmed;
    result.confirmed=confirmed_;
    return result;
  }
 private:
  int votes_{};bool confirmed_{};
};
