#include "../src/camera_rectifier.hpp"
#include <iostream>
static void require(bool value){if(!value)throw std::runtime_error("camera rectifier regression");}
int main(){
  const std::vector<double> k{250,0,160,0,250,120,0,0,1},zero{0,0,0,0,0},d{.2,-.05,.01,0,0};
  cv::Mat raw(240,320,CV_8UC3);cv::randu(raw,0,255);const auto original=raw.clone();
  CameraRectifier identity(true,320,240,k,zero),disabled(false,0,0,{},{}),warp(true,320,240,k,d);
  require(cv::norm(identity.apply(raw),raw,cv::NORM_INF)==0);
  require(disabled.apply(raw).data==raw.data);
  auto corrected=warp.apply(raw);require(corrected.size()==raw.size());
  cv::Mat reference;cv::undistort(raw,reference,cv::Mat(3,3,CV_64F,const_cast<double*>(k.data())),cv::Mat(d));
  require(cv::norm(corrected,reference,cv::NORM_INF)<=1);
  require(cv::norm(raw,original,cv::NORM_INF)==0);
  require(warp.valid_mask().type()==CV_8UC1 && warp.valid_mask().size()==raw.size());
  require(warp.valid_mask().at<unsigned char>(0,0)==0);
  require(cv::countNonZero(warp.valid_mask())>raw.total()/2);
  bool rejected=false;try{warp.apply(cv::Mat(480,640,CV_8UC3));}catch(const std::exception&){rejected=true;}
  require(rejected);std::cout<<"raw preservation, pixel correction and dimension guard passed\n";
  // Switching models must always remap the preserved raw frame: feeding the
  // previous phase's corrected image through the next model distorts it twice.
  const std::vector<double> surface_k{270,0,155,0,265,118,0,0,1},surface_d{-.15,.03,0,.01,0};
  CameraRectifier surface(true,320,240,surface_k,surface_d);
  for(bool near_surface:{false,true,false,true}) {
    const auto actual=near_surface?surface.apply(raw):warp.apply(raw);
    const auto& model_k=near_surface?surface_k:k;
    const auto& model_d=near_surface?surface_d:d;
    cv::undistort(raw,reference,cv::Mat(3,3,CV_64F,const_cast<double*>(model_k.data())),cv::Mat(model_d));
    require(cv::norm(actual,reference,cv::NORM_INF)<=1);
    require(cv::norm(raw,original,cv::NORM_INF)==0);
  }
  require(cv::norm(surface.apply(corrected),surface.apply(raw),cv::NORM_INF)>1);
}
