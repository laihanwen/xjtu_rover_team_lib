#include "auv_vision/shape_cone_segmenter.hpp"
#include "auv_mapping/pipe_centerlines.hpp"
#include <stdexcept>
static void check(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
int main(int argc,char** argv) {
  if(argc==2) {
    auv_vision::ShapeConeSegmenter smoke(argv[1]);
    cv::Mat image(600,600,CV_8UC3,cv::Scalar(0,0,0));
    check(smoke.process(image).empty(),"ONNX background produces no cone");
    cv::circle(image,{100,100},40,{255,255,255},cv::FILLED);
    cv::Mat background;
    check(smoke.process(image,&background).size()==1,"real OpenCV DNN body output contract");
    check(background.at<unsigned char>(100,100)==0&&background.at<unsigned char>(300,300)==255,
      "uncertain/foreground body cannot become empty-floor support");
    auv_vision::ConeDetectorConfig experiment;experiment.mode="shape";
    experiment.shape_model_path=argv[1];experiment.shape_model_id="synthetic-smoke-only";
    auto result=auv_vision::ConeDetector(experiment).process(image);
    check(result.shape_observations.size()==1&&result.observations.empty()&&!result.classifier_ready,
      "unverified weights may be evaluated without authorizing a semantic class");
  }
  const int sizes[]={1,3,320,320};cv::Mat probabilities(4,sizes,CV_32F,cv::Scalar(0));
  float* p=probabilities.ptr<float>();const int area=320*320;
  std::fill(p,p+area,1.F);
  for(int y=30;y<70;++y)for(int x=30;x<70;++x){p[y*320+x]=.05F;p[area+y*320+x]=.95F;}
  auto body=auv_vision::ShapeConeSegmenter::decode(probabilities,{600,600});
  check(body.size()==1&&body[0].shape==auv_vision::ConeShape::kCircle,"body semantic mask");
  check(body[0].row==0&&body[0].col==0,"body cell");
  auto other=body;other[0].shape=auv_vision::ConeShape::kSquare;
  check(auv_vision::ShapeConeSegmenter::select("agreement",other,body,{600,600},.72).empty(),"class conflict unknown");
  check(auv_vision::ShapeConeSegmenter::select("agreement",body,body,{600,600},.72).size()==1,"matching body decision");
  other=body;other[0].center.x+=100;
  check(auv_vision::ShapeConeSegmenter::select("agreement",other,body,{600,600},.72).empty(),"different instance unknown");
  p[0]=2;bool threw=false;try{auv_vision::ShapeConeSegmenter::decode(probabilities,{600,600});}catch(const std::invalid_argument&){threw=true;}
  check(threw,"invalid probabilities rejected");
  auv_vision::ConeDetectorConfig cfg;cfg.mode="agreement";
  auto result=auv_vision::ConeDetector(cfg).process(cv::Mat(600,600,CV_8UC3,cv::Scalar(0,0,0)));
  check(result.observations.empty()&&!result.classifier_ready,"missing shape model cannot fall back silently");
  cv::Mat split(600,600,CV_8UC3,cv::Scalar(60,60,60));
  cv::rectangle(split,{50,50,30,30},{0,80,255},cv::FILLED);
  cv::rectangle(split,{100,100,30,30},{0,80,255},cv::FILLED);
  check(auv_vision::ConeDetector().process(split).observations.empty(),"separate red pieces are not a whole square cone");
  cv::Mat pipe(240,320,CV_8U,cv::Scalar(0));
  cv::rectangle(pipe,{40,20,7,180},cv::Scalar(255),cv::FILLED);
  cv::rectangle(pipe,{100,20,100,180},cv::Scalar(255),cv::FILLED);
  const auto centre=auv_mapping::pipe_centerlines(pipe);
  check(cv::countNonZero(centre(cv::Rect(41,30,5,150)))>100,"thin measured pipe ridge");
  check(cv::countNonZero(centre(cv::Rect(120,40,60,140)))==0,"broad floor interior rejected");
}
