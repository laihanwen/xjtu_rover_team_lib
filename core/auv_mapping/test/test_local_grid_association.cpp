#include "auv_mapping/local_grid_association.hpp"
#include <stdexcept>
#include <limits>
using namespace auv_mapping;
static void check(bool value){if(!value)throw std::runtime_error("local grid association regression");}
int main(){
  auto repeated=associate_local_grid({{0,0,15,15,0,.9}});
  check(!repeated.unique && repeated.hypotheses.size()==16);
  // A cropped T cannot be promoted to a boundary: the fourth arm is untested.
  auto occluded=associate_local_grid({{0,0,11,11,0,.9}});
  check(!occluded.unique && occluded.hypotheses.size()>16);
  std::vector<LocalGridNode> boundary={{0,0,11,15,10,.9},{0,2,9,15,8,.9}};
  auto anchored=associate_local_grid(boundary);
  check(anchored.unique && anchored.hypotheses[0].nodes[0]==std::array<int,2>{3,1});
  check(anchored.hypotheses[0].nodes[1]==std::array<int,2>{3,3});
  // Match every physical rotation without pretending it is field yaw.
  for(int turn=1;turn<4;++turn){
    auto rotated=boundary;
    for(auto& node:rotated){auto p=rotate_grid_node(node.row,node.col,turn);node.row=p[0];node.col=p[1];
      node.arms=rotate_grid_arms(node.arms,turn);node.tested_directions=rotate_grid_arms(node.tested_directions,turn);
      node.yellow_arms=rotate_grid_arms(node.yellow_arms,turn);}
    auto result=associate_local_grid(rotated);
    check(result.unique && result.hypotheses[0].quarter_turns==(4-turn)%4);
    check(result.hypotheses[0].nodes==anchored.hypotheses[0].nodes);
  }
  auto conflict=boundary;conflict[1].arms=15;
  check(associate_local_grid(conflict).hypotheses.empty());
  auto duplicate=boundary;duplicate.push_back(boundary[0]);
  check(associate_local_grid(duplicate).hypotheses.empty());
  auto invalid=boundary;invalid[0].confidence=std::numeric_limits<double>::quiet_NaN();
  check(associate_local_grid(invalid).hypotheses.empty());
  check(associate_local_grid({}).hypotheses.empty());
}
