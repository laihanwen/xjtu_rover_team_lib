#undef NDEBUG
#include "auv_mapping/plane_odometry.hpp"
#include <cassert>
#include <iostream>
using namespace auv_mapping;
int main(){
 PlaneCalibration c;c.verified=true;c.pool_depth=2;c.camera_to_body={0,-1,0,-1,0,0,0,0,-1};
 assert(calibration_valid(c));PlaneSample a{1,0,0,0,1},b=a;b.stamp=1.1;
 std::vector<std::array<double,4>> matches;
 for(int i=0;i<40;i++){double x=(i%8-4)*.03,y=(i/8-2)*.03;matches.push_back({x,y,x,y+.04});}
 auto e=estimate_translation(c,a,b,matches);assert(e.valid&&std::abs(e.dx-.04)<1e-8&&std::abs(e.dy)<1e-8);
 b.yaw=.2;matches.clear();
 for(int i=0;i<40;i++){double x=(i%8-4)*.03,y=(i/8-2)*.03;Vec3 p;assert(floor_point(c,a,x,y,p));auto r=attitude(0,0,-b.yaw);auto q=multiply(r,p);matches.push_back({x,y,-q[1],-q[0]});}
 e=estimate_translation(c,a,b,matches);assert(e.valid&&std::hypot(e.dx,e.dy)<1e-8);
 b=a;b.stamp=1.1;b.depth=.8;matches.clear();
 for(int i=0;i<40;i++){double x=(i%8-4)*.03,y=(i/8-2)*.03;matches.push_back({x,y,x/1.2,y/1.2});}
 e=estimate_translation(c,a,b,matches);assert(e.valid&&std::hypot(e.dx,e.dy)<1e-8);
 // Sparse matches and grossly inconsistent motion must fail.
 auto small=matches;small.resize(10);assert(!estimate_translation(c,a,b,small).valid);
 auto bad=matches;for(std::size_t i=0;i<bad.size();i++)bad[i][2]+=static_cast<double>(i)*.1;
 assert(!estimate_translation(c,a,b,bad).valid);
 // Tilt with a displaced camera must not look like translation.
 c.camera_offset={.12,.03,-.05};c.depth_offset={-.08,0,.02};a={2,0,0,0,1};b=a;b.stamp=2.1;b.pitch=.12;b.roll=-.1;
 const auto rotation=attitude(b.roll,b.pitch,b.yaw);auto sensor=multiply(rotation,c.depth_offset);
 // Hold body vertical position fixed while pressure sensor rotates around body center.
 b.depth=a.depth+c.depth_offset[2]-sensor[2];
 matches.clear();
 for(int i=0;i<40;i++){
  double nx=(i%8-4)*.03,ny=(i/8-2)*.03;Vec3 world;assert(floor_point(c,a,nx,ny,world));
  // Camera optical coordinates: inverse body/world and inverse camera/body rotation.
  Vec3 delta={world[0],world[1],world[2]};Vec3 body={rotation[0]*delta[0]+rotation[3]*delta[1]+rotation[6]*delta[2],rotation[1]*delta[0]+rotation[4]*delta[1]+rotation[7]*delta[2],rotation[2]*delta[0]+rotation[5]*delta[1]+rotation[8]*delta[2]};
  for(int k=0;k<3;k++)body[k]-=c.camera_offset[k];
  const auto& m=c.camera_to_body;Vec3 optical={m[0]*body[0]+m[3]*body[1]+m[6]*body[2],m[1]*body[0]+m[4]*body[1]+m[7]*body[2],m[2]*body[0]+m[5]*body[1]+m[8]*body[2]};
  matches.push_back({nx,ny,optical[0]/optical[2],optical[1]/optical[2]});
 }
 e=estimate_translation(c,a,b,matches);assert(e.valid&&std::hypot(e.dx,e.dy)<1e-8);
 b.stamp=a.stamp;assert(!estimate_translation(c,a,b,matches).valid);
 c.verified=false;Vec3 p;assert(!floor_point(c,a,0,0,p));c.verified=true;c.camera_to_body={1,0,0,0,1,0,0,0,1};assert(!floor_point(c,a,0,0,p));
 std::cout<<"plane geometry: translation, yaw, height, gap and calibration passed\n";
}
