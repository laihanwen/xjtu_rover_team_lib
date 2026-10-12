#pragma once
#include "auv_mapping/local_grid_association.hpp"
#include "auv_mapping/plane_odometry.hpp"
#include "auv_mapping/metric_grid_pose.hpp"
#include "auv_mapping/pipe_centerlines.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <limits>

namespace auv_mapping {
struct LocalGridDetection {
  bool valid{false};std::string reason{"unavailable"};
  GridAssociationResult association;
  std::vector<cv::Point2f> pixels;
  std::vector<LocalGridNode> nodes;
  cv::Mat image_to_grid,rectified,visible,debug;
};
// Uses the measured horizontal floor and cell pitch to remove perspective.
// A visible missing arm is negative evidence; an occluded arm is never tested.
// No complete outer rectangle is needed, and repeated interior nodes stay ambiguous.
class LocalGridDetector {
 public:
  explicit LocalGridDetector(GridMapperConfig config={}):config_(std::move(config)){}
  LocalGridDetection process(const cv::Mat& image,const MetricGridCalibration& c,
      const PlaneSample& sample,const cv::Mat& valid_pixels=cv::Mat())const {
    LocalGridDetection out;
    if(!c.verified){out.reason="local metric calibration unverified";return out;}
    c.validate();
    if(image.empty()||image.type()!=CV_8UC3||image.cols!=c.width||image.rows!=c.height||
       sample.depth<0||sample.depth>=c.pool_depth_m){out.reason="local image or depth invalid";return out;}
    if(!valid_pixels.empty()&&(valid_pixels.type()!=CV_8UC1||valid_pixels.size()!=image.size())) {
      out.reason="local rectification mask mismatch";return out;
    }
    const cv::Mat input_visibility=valid_pixels.empty()?cv::Mat(image.size(),CV_8U,cv::Scalar(255)):valid_pixels;
    PlaneCalibration plane;plane.verified=true;plane.pool_depth=c.pool_depth_m;
    std::copy(c.camera_to_body.begin(),c.camera_to_body.end(),plane.camera_to_body.begin());
    std::copy(c.camera_offset_m.begin(),c.camera_offset_m.end(),plane.camera_offset.begin());
    std::copy(c.depth_offset_m.begin(),c.depth_offset_m.end(),plane.depth_offset.begin());
    const double scale=100/c.cell_size_m;
    std::vector<cv::Point2f> source{{0,0},{float(image.cols-1),0},
      {float(image.cols-1),float(image.rows-1)},{0,float(image.rows-1)}};
    std::vector<cv::Point2f> floor;
    for(const auto& p:source) {
      Vec3 q;
      if(!floor_point(plane,sample,(p.x-c.camera_matrix[2])/c.camera_matrix[0],
          (p.y-c.camera_matrix[5])/c.camera_matrix[4],q)) {
        out.reason="floor projection outside supported field of view";return out;
      }
      floor.emplace_back(float(q[0]*scale),float(q[1]*scale));
    }
    auto bounds=cv::boundingRect(floor);
    if(bounds.width<30||bounds.height<30||bounds.width>800||bounds.height>800) {
      out.reason="floor footprint outside resource bounds";return out;
    }
    for(auto& p:floor){p.x-=bounds.x;p.y-=bounds.y;}
    cv::Mat h=cv::getPerspectiveTransform(source,floor),bird,coverage;
    cv::warpPerspective(image,bird,h,bounds.size());
    cv::warpPerspective(input_visibility,coverage,h,bounds.size(),cv::INTER_NEAREST);
    cv::erode(coverage,coverage,cv::getStructuringElement(cv::MORPH_RECT,{7,7}));
    cv::Mat hsv,white,yellow,mask,occluder1,occluder2,occluders;
    cv::cvtColor(bird,hsv,cv::COLOR_BGR2HSV);
    cv::inRange(hsv,cv::Scalar(0,0,config_.white_v_min),cv::Scalar(179,config_.white_s_max,255),white);
    cv::inRange(hsv,config_.hsv_lower,config_.hsv_upper,yellow);
    cv::inRange(hsv,cv::Scalar(0,80,50),cv::Scalar(14,255,255),occluder1);
    cv::inRange(hsv,cv::Scalar(165,80,50),cv::Scalar(179,255,255),occluder2);
    cv::bitwise_or(occluder1,occluder2,occluders);
    cv::bitwise_or(white,yellow,mask);cv::bitwise_and(mask,coverage,mask);
    std::vector<cv::Vec4i> lines;
    const auto centres=config_.local_pipe_centres?pipe_centerlines(mask):mask;
    cv::HoughLinesP(centres,lines,1,CV_PI/180,35,50,6);
    if(lines.size()<3||lines.size()>100){out.reason="insufficient or excessive pipe segments";return out;}
    std::array<double,90> votes{};
    for(const auto& l:lines) {
      double a=std::atan2(l[3]-l[1],l[2]-l[0])*180/CV_PI;
      while(a<0)a+=90;while(a>=90)a-=90;
      votes[std::min(89,int(a))]+=std::hypot(l[2]-l[0],l[3]-l[1]);
    }
    int best=0;double support=0;
    for(int i=0;i<90;++i){double v=0;for(int d=-3;d<=3;++d)v+=votes[(i+d+90)%90];if(v>support){support=v;best=i;}}
    // Rotate the dominant pipe axis onto the local column axis. The actual
    // canonical rotation is chosen only by topology and observed yellow arms.
    // The peak window selects support, not its centre: using the first
    // winning bin biases an axis by up to three degrees. Average supported
    // measured angles modulo 90 degrees instead.
    double axis_cos=0,axis_sin=0;
    for(const auto& l:lines) {
      const double angle=std::atan2(l[3]-l[1],l[2]-l[0]);
      double degrees=angle*180/CV_PI;
      while(degrees<0)degrees+=90;while(degrees>=90)degrees-=90;
      const double distance=std::abs(degrees-best);
      if(std::min(distance,90-distance)>4)continue;
      const double weight=std::hypot(l[2]-l[0],l[3]-l[1]);
      axis_cos+=weight*std::cos(4*angle);axis_sin+=weight*std::sin(4*angle);
    }
    const double a=.25*std::atan2(axis_sin,axis_cos),ca=std::cos(a),sa=std::sin(a);
    std::vector<float> xs,ys;
    for(const auto& l:lines) {
      const double dx=l[2]-l[0],dy=l[3]-l[1];
      const double u=ca*dx+sa*dy,v=-sa*dx+ca*dy;
      if(std::abs(v)<.08*std::abs(u))ys.push_back(float((-sa*(l[0]+l[2])+ca*(l[1]+l[3]))/2));
      if(std::abs(u)<.08*std::abs(v))xs.push_back(float((ca*(l[0]+l[2])+sa*(l[1]+l[3]))/2));
    }
    xs=cluster(xs);ys=cluster(ys);
    xs=lattice_consensus(xs);ys=lattice_consensus(ys);
    if(xs.size()<2||ys.size()<2||xs.size()>4||ys.size()>4||!pitch_valid(xs)||!pitch_valid(ys)) {
      out.reason="pipe pitch does not match measured cell spacing";return out;
    }
    const auto inv=h.inv();
    for(float y:ys)for(float x:xs) {
      const cv::Point2f p(float(ca*x-sa*y),float(sa*x+ca*y));
      if(!fraction(mask,p,{0,0},4,.8))continue;
      LocalGridNode node;node.col=int(std::lround((x-xs[0])/100));node.row=int(std::lround((y-ys[0])/100));
      const std::array<cv::Point2f,4> directions{{{float(sa),float(-ca)},{float(ca),float(sa)},
        {float(-sa),float(ca)},{float(-ca),float(-sa)}}};
      for(int d=0;d<4;++d) {
        const auto q=p+directions[d]*25;
        if(!fraction(coverage,q,{0,0},6,.99))continue;
        if(fraction(occluders,q,directions[d],8,.1))continue;
        node.tested_directions|=1U<<d;
        if(fraction(mask,q,directions[d],8,.65)) {
          node.arms|=1U<<d;
          if(fraction(yellow,q,directions[d],8,.65))node.yellow_arms|=1U<<d;
        }
      }
      if(!node.arms)continue;
      node.confidence=.9;out.nodes.push_back(node);
      std::vector<cv::Point2f> p1{p},p2;cv::perspectiveTransform(p1,p2,inv);out.pixels.push_back(p2[0]);
    }
    out.association=associate_local_grid(out.nodes);
    if(out.association.unique) {
      out=resolve(std::move(out),image,valid_pixels,0);
      if(out.valid)remember_placement(out,sample.stamp);
      return out;
    }
    // Repeated interior nodes stay ambiguous without an outer edge or yellow
    // arm. A bounded, same-stage temporal prior disambiguates them against the
    // last accepted placement instead of enlarging the topology gate.
    if(out.association.hypotheses.size()>1) {
      const std::size_t selected=temporal_select(out.association,sample.stamp);
      if(selected<out.association.hypotheses.size()) {
        out=resolve(std::move(out),image,valid_pixels,selected);
        if(out.valid){out.reason="continuous local association";remember_placement(out,sample.stamp);}
        return out;
      }
    }
    out.reason=out.association.reason;return out;
  }
  // Resolves an already measured candidate; caller must establish uniqueness
  // from topology or a fresh, same-stage bounded metric prediction.
  LocalGridDetection resolve(LocalGridDetection out,const cv::Mat& image,
      const cv::Mat& valid_pixels,std::size_t selected)const {
    if(selected>=out.association.hypotheses.size()) {out.reason="invalid local association selection";return out;}
    if(out.nodes.size()<4){out.reason="not enough noncollinear local landmarks";return out;}
    std::vector<cv::Point2f> canonical;
    for(const auto& n:out.association.hypotheses[selected].nodes)canonical.emplace_back(float(n[1]*200),float(n[0]*200));
    out.image_to_grid=cv::findHomography(out.pixels,canonical,cv::RANSAC,3);
    if(out.image_to_grid.empty()){out.reason="local landmarks degenerate";return out;}
    std::vector<cv::Point2f> projected;cv::perspectiveTransform(out.pixels,projected,out.image_to_grid);
    for(std::size_t i=0;i<projected.size();++i)if(cv::norm(projected[i]-canonical[i])>3) {
      out.reason="local lattice residual inconsistent";return out;
    }
    cv::warpPerspective(image,out.rectified,out.image_to_grid,{600,600});
    cv::warpPerspective(valid_pixels.empty()?cv::Mat(image.size(),CV_8U,cv::Scalar(255)):valid_pixels,out.visible,out.image_to_grid,{600,600},cv::INTER_NEAREST);
    cv::erode(out.visible,out.visible,cv::getStructuringElement(cv::MORPH_RECT,{9,9}));
    out.debug=image.clone();for(auto p:out.pixels)cv::circle(out.debug,p,4,{0,0,255},1);
    auto chosen=out.association.hypotheses[selected];
    out.association.hypotheses={chosen};out.association.unique=true;
    out.valid=true;out.reason="associated local pipe topology";return out;
  }
 private:
  GridMapperConfig config_;
  // Bounded temporal prior for continuous association of repeated interior
  // nodes. Stores the canonical placement of the last accepted frame so an
  // ambiguous lattice can be pinned to the same rotation and a nearby centroid
  // instead of enlarging the topology or pixel-residual gates.
  mutable bool prior_valid_{false};
  mutable double prior_stamp_{-1};
  mutable int prior_quarter_turns_{0};
  mutable double prior_centroid_row_{0},prior_centroid_col_{0};
  std::size_t temporal_select(const GridAssociationResult& association,double stamp)const {
    if(!prior_valid_||!std::isfinite(stamp)||stamp<prior_stamp_||stamp-prior_stamp_>1.0)
      return association.hypotheses.size();
    std::size_t best=association.hypotheses.size();double best_distance=std::numeric_limits<double>::infinity();bool unique=false;
    for(std::size_t i=0;i<association.hypotheses.size();++i) {
      const auto& h=association.hypotheses[i];
      if(h.quarter_turns!=prior_quarter_turns_)continue;
      double r=0,c=0;
      for(const auto& n:h.nodes){r+=n[0];c+=n[1];}
      r/=h.nodes.size();c/=h.nodes.size();
      const double d=std::hypot(r-prior_centroid_row_,c-prior_centroid_col_);
      if(d>1.0)continue;
      if(d<best_distance-1e-9){best=i;best_distance=d;unique=true;}
      else if(std::abs(d-best_distance)<=1e-9)unique=false;
    }
    return unique?best:association.hypotheses.size();
  }
  void remember_placement(const LocalGridDetection& out,double stamp)const {
    if(!out.valid||out.association.hypotheses.empty())return;
    const auto& h=out.association.hypotheses[0];
    prior_valid_=true;prior_stamp_=stamp;prior_quarter_turns_=h.quarter_turns;
    double r=0,c=0;
    for(const auto& n:h.nodes){r+=n[0];c+=n[1];}
    prior_centroid_row_=r/h.nodes.size();prior_centroid_col_=c/h.nodes.size();
  }
  static std::vector<float> lattice_consensus(std::vector<float> v) {
    // Keeps the largest subset of clustered line positions that lies on the
    // measured 100 px cell lattice (a missing line yields an integer multiple
    // of the pitch). A clipped boundary segment near the bird's-eye edge or a
    // spurious Hough fragment must not veto an otherwise consistent lattice;
    // the surviving subset is still re-checked by pitch_valid at 8 px.
    if(v.size()<=2)return v;
    std::sort(v.begin(),v.end());
    std::size_t best_count=0;std::vector<float> best;
    for(std::size_t base=0;base<v.size();++base) {
      std::vector<float> kept;
      for(double p:v){const double n=(p-v[base])/100.0;if(std::abs(n-std::round(n))*100.0<=8.0)kept.push_back(float(p));}
      if(kept.size()>best_count){best_count=kept.size();best=std::move(kept);}
    }
    return best_count>=2?best:v;
  }
  static std::vector<float> cluster(std::vector<float> input) {
    std::sort(input.begin(),input.end());std::vector<float> out;
    for(std::size_t i=0;i<input.size();) {
      std::size_t end=i+1;double sum=input[i];
      while(end<input.size()&&input[end]-input[i]<12){sum+=input[end];++end;}
      out.push_back(float(sum/(end-i)));i=end;
    }return out;
  }
  static bool pitch_valid(const std::vector<float>& v) {
    for(auto p:v){double n=(p-v[0])/100;if(std::abs(n-std::round(n))>.08||n>3.08)return false;}return true;
  }
  static bool fraction(const cv::Mat& mask,cv::Point2f p,cv::Point2f axis,int radius,double threshold) {
    int good=0,total=0;
    for(int i=-radius;i<=radius;++i) {
      const cv::Point2f q=p+axis*float(i);int x=int(std::lround(q.x)),y=int(std::lround(q.y));
      if(x<2||y<2||x>=mask.cols-2||y>=mask.rows-2)return false;
      ++total;if(cv::countNonZero(mask(cv::Rect(x-2,y-2,5,5)))>=(threshold>.98?25:15))++good;
    }return good>=threshold*total;
  }
};
}
