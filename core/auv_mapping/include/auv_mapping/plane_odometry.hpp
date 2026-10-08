#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace auv_mapping {
using Vec3=std::array<double,3>;
using Mat3=std::array<double,9>;
inline Vec3 multiply(const Mat3& m,const Vec3& v){return {m[0]*v[0]+m[1]*v[1]+m[2]*v[2],m[3]*v[0]+m[4]*v[1]+m[5]*v[2],m[6]*v[0]+m[7]*v[1]+m[8]*v[2]};}
inline Mat3 attitude(double roll,double pitch,double yaw){
 const double cr=std::cos(roll),sr=std::sin(roll),cp=std::cos(pitch),sp=std::sin(pitch),cy=std::cos(yaw),sy=std::sin(yaw);
 return {cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr};
}
struct PlaneCalibration {
 Mat3 camera_to_body{}; Vec3 camera_offset{},depth_offset{};
 double pool_depth=0,depth_zero=0; bool verified=false;
};
struct PlaneSample {double stamp=0,roll=0,pitch=0,yaw=0,depth=0;};
struct PlaneEstimate {bool valid=false;double dx=0,dy=0,residual=0;std::size_t inliers=0;std::string reason;};
inline bool finite(const Vec3& v){return std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});}
inline bool calibration_valid(const PlaneCalibration& c){
 if(!c.verified||!std::isfinite(c.pool_depth)||c.pool_depth<=0||!std::isfinite(c.depth_zero)||!finite(c.camera_offset)||!finite(c.depth_offset))return false;
 for(auto v:c.camera_to_body)if(!std::isfinite(v))return false;
 for(int i=0;i<3;i++)for(int j=0;j<3;j++){double dot=0;for(int k=0;k<3;k++)dot+=c.camera_to_body[i*3+k]*c.camera_to_body[j*3+k];if(std::abs(dot-(i==j?1:0))>1e-4)return false;}
 const auto& m=c.camera_to_body;
 return std::abs(m[0]*(m[4]*m[8]-m[5]*m[7])-m[1]*(m[3]*m[8]-m[5]*m[6])+m[2]*(m[3]*m[7]-m[4]*m[6])-1)<1e-4;
}
// Coordinates relative to body origin; floor is horizontal, body X forward/Y left/Z up.
inline bool floor_point(const PlaneCalibration& c,const PlaneSample& s,double nx,double ny,Vec3& result){
 if(!calibration_valid(c)||!std::isfinite(s.stamp)||!std::isfinite(s.roll)||!std::isfinite(s.pitch)||!std::isfinite(s.yaw)||!std::isfinite(s.depth)||!std::isfinite(nx)||!std::isfinite(ny))return false;
 const auto r=attitude(s.roll,s.pitch,s.yaw);const auto camera=multiply(r,c.camera_offset),sensor=multiply(r,c.depth_offset);
 const double body_height=c.pool_depth-(s.depth-c.depth_zero)-sensor[2];
 const double height=body_height+camera[2];
 const auto ray=multiply(r,multiply(c.camera_to_body,{nx,ny,1}));
 if(height<.15||height>c.pool_depth+.5||ray[2]>=-.15)return false;
 const double scale=-height/ray[2];result={camera[0]+scale*ray[0],camera[1]+scale*ray[1],-body_height};return finite(result);
}
inline double median(std::vector<double> v){std::sort(v.begin(),v.end());return v[v.size()/2];}
inline PlaneEstimate estimate_translation(const PlaneCalibration& c,const PlaneSample& before,const PlaneSample& after,const std::vector<std::array<double,4>>& matches,double residual_limit=.04,double max_speed=1.0){
 PlaneEstimate out;const double dt=after.stamp-before.stamp;
 if(!(dt>0&&dt<=.3)){out.reason="frame_time_gap";return out;}
 std::vector<double> xs,ys;
 for(const auto& p:matches){Vec3 a,b;if(floor_point(c,before,p[0],p[1],a)&&floor_point(c,after,p[2],p[3],b)){xs.push_back(a[0]-b[0]);ys.push_back(a[1]-b[1]);}}
 if(xs.size()<20){out.reason="insufficient_floor_points";return out;}
 const double mx=median(xs),my=median(ys);std::vector<double> goodx,goody,errors;
 for(std::size_t i=0;i<xs.size();++i){const double e=std::hypot(xs[i]-mx,ys[i]-my);if(e<=residual_limit){goodx.push_back(xs[i]);goody.push_back(ys[i]);errors.push_back(e);}}
 out.inliers=goodx.size();
 if(out.inliers<20||out.inliers<.6*xs.size()){out.reason="translation_inconsistent";return out;}
 out.dx=median(goodx);out.dy=median(goody);out.residual=median(errors);
 if(std::hypot(out.dx,out.dy)>max_speed*dt+.01){out.reason="implausible_speed";return out;}
 out.valid=true;out.reason="tracking";return out;
}
}
