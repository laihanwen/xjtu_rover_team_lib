#pragma once
#include "auv_vision/cone_detector.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <mutex>
#include <cmath>
#include <algorithm>

namespace auv_vision {
// Alternative B: grayscale body segmentation, independent of red/white
// stripe colour. ONNX contract: 320x320 RGB-replicated grayscale input /255,
// output [1,3,320,320] probabilities: background, circle BODY, square BODY.
// Training labels must exclude bases. No supplied weights means no decisions.
class ShapeConeSegmenter {
 public:
  static std::vector<ConeObservation> select(const std::string& mode,
      const std::vector<ConeObservation>& colour,const std::vector<ConeObservation>& shape,
      cv::Size size,double minimum_confidence) {
    std::vector<ConeObservation> result;
    for(const auto& b:shape) {
      if(b.confidence<minimum_confidence)continue;
      if(mode=="shape"){result.push_back(b);continue;}
      const auto a=std::find_if(colour.begin(),colour.end(),[&](const auto& v){return
        v.row==b.row&&v.col==b.col&&v.shape==b.shape&&v.confidence>=minimum_confidence;});
      if(a!=colour.end()&&cv::norm(a->center-b.center)<.1*std::min(size.width,size.height)/3)
        result.push_back(b);
    }
    return result;
  }
  explicit ShapeConeSegmenter(const std::string& path):net_(cv::dnn::readNetFromONNX(path)) {
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }
  static std::vector<ConeObservation> decode(const cv::Mat& probabilities,cv::Size size) {
    if(probabilities.dims!=4||!probabilities.isContinuous()||probabilities.type()!=CV_32F||probabilities.size[0]!=1||
       probabilities.size[1]!=3||probabilities.size[2]!=320||probabilities.size[3]!=320)
      throw std::invalid_argument("shape model output must be [1,3,320,320] float probabilities");
    cv::Mat labels(320,320,CV_8U,cv::Scalar(0));
    const auto* p=probabilities.ptr<float>();const int area=320*320;
    for(int i=0;i<area;++i) {
      float sum=0;int best=0;
      for(int c=0;c<3;++c) {
        const float v=p[c*area+i];
        if(!std::isfinite(v)||v<0||v>1)throw std::invalid_argument("invalid shape probabilities");
        sum+=v;if(v>p[best*area+i])best=c;
      }
      if(std::abs(sum-1)>0.01)throw std::invalid_argument("unnormalized shape probabilities");
      if(best&&p[best*area+i]>=.85)labels.ptr<unsigned char>()[i]=static_cast<unsigned char>(best);
    }
    cv::resize(labels,labels,size,0,0,cv::INTER_NEAREST);
    std::vector<ConeObservation> out;
    for(int c=1;c<=2;++c) {
      cv::Mat mask=labels==c;std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
      for(const auto& contour:contours) {
        const auto bounds=cv::boundingRect(contour);const auto centre=cv::Point2f(bounds.x+bounds.width*.5F,bounds.y+bounds.height*.5F);
        const int row=std::min(2,int(centre.y*3/size.height)),col=std::min(2,int(centre.x*3/size.width));
        const cv::Rect cell(col*size.width/3,row*size.height/3,size.width/3,size.height/3);
        const double area_ratio=std::abs(cv::contourArea(contour))/cell.area();
        // A cut body or one crossing cell boundaries cannot authorize a class.
        if(area_ratio<.025||area_ratio>.65||bounds.x<=cell.x+3||bounds.y<=cell.y+3||
           bounds.br().x>=cell.br().x-3||bounds.br().y>=cell.br().y-3)continue;
        ConeObservation value;value.shape=c==1?ConeShape::kCircle:ConeShape::kSquare;
        value.row=row;value.col=col;value.center=centre;value.contour=contour;
        value.width=float(bounds.width);value.height=float(bounds.height);
        value.area=float(std::abs(cv::contourArea(contour)));value.confidence=.85F;out.push_back(std::move(value));
      }
    }
    // Two instances in a cell are ambiguous, even if their classes agree.
    auto all=out;
    out.erase(std::remove_if(out.begin(),out.end(),[&](const auto& a){return
      std::count_if(all.begin(),all.end(),[&](const auto& b){return a.row==b.row&&a.col==b.col;})!=1;}),out.end());
    return out;
  }
  std::vector<ConeObservation> process(const cv::Mat& image,cv::Mat* background=nullptr) {
    cv::Mat gray,replicated;cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray,replicated,cv::COLOR_GRAY2BGR);
    const auto blob=cv::dnn::blobFromImage(replicated,1.0/255,{320,320});
    std::lock_guard<std::mutex> lock(mutex_);net_.setInput(blob);
    auto probabilities=net_.forward();auto observations=decode(probabilities,image.size());
    if(background) {
      cv::Mat support(320,320,CV_8U,cv::Scalar(0));const auto* p=probabilities.ptr<float>();
      for(int i=0;i<320*320;++i)if(p[i]>=.995F)support.ptr<unsigned char>()[i]=255;
      cv::resize(support,*background,image.size(),0,0,cv::INTER_NEAREST);
    }
    return observations;
  }
 private:cv::dnn::Net net_;std::mutex mutex_;
};
}
