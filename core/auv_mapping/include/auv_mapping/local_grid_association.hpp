#pragma once
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace auv_mapping {
// Local coordinates are integer PIPE-CENTRE node indices, not image pixels.
// Producer must validate spacing/topology before using this matcher. Directions
// are N/E/S/W (bits 0/1/2/3). An untested/occluded arm is NOT an absent arm.
struct LocalGridNode {
  int row{0},col{0};
  unsigned arms{0},tested_directions{0},yellow_arms{0};
  double confidence{0};
};
struct GridAssociationHypothesis {
  int quarter_turns{0},row_offset{0},col_offset{0};
  std::vector<std::array<int,2>> nodes;
};
struct GridAssociationResult {
  bool unique{false};
  std::string reason;
  std::vector<GridAssociationHypothesis> hypotheses;
};
inline std::array<int,2> rotate_grid_node(int row,int col,int turns) {
  if(turns==1)return {col,-row};
  if(turns==2)return {-row,-col};
  if(turns==3)return {-col,row};
  return {row,col};
}
inline unsigned rotate_grid_arms(unsigned mask,int turns) {
  return ((mask<<turns)|(mask>>(4-turns)))&15U;
}
inline unsigned grid_node_arms(int row,int col) {
  return (row>0?1U:0U)|(col<3?2U:0U)|(row<3?4U:0U)|(col>0?8U:0U);
}
// Enumerates translations and rotations of a measured local lattice onto the
// 4x4 node lattice. Canonical yellow edge is bottom; this is NOT field-heading
// calibration, PnP, node detection or odometry. No reflection is guessed.
inline GridAssociationResult associate_local_grid(const std::vector<LocalGridNode>& input) {
  GridAssociationResult result;
  if(input.empty()||input.size()>16){result.reason="invalid node count";return result;}
  for(std::size_t i=0;i<input.size();++i) {
    const auto& node=input[i];
    if(node.row < -3 || node.row > 3 || node.col < -3 || node.col > 3 ||
       node.arms>15 || node.tested_directions>15 || node.yellow_arms>15 ||
       (node.arms&~node.tested_directions) || (node.yellow_arms&~node.arms) ||
       !std::isfinite(node.confidence)||node.confidence<.8||node.confidence>1) {
      result.reason="invalid or unverified node evidence";return result;
    }
    for(std::size_t j=0;j<i;++j)if(input[j].row==node.row&&input[j].col==node.col) {
      result.reason="duplicate local node";return result;
    }
  }
  for(int turn=0;turn<4;++turn) {
    const auto anchor=rotate_grid_node(input[0].row,input[0].col,turn);
    for(int row=0;row<4;++row)for(int col=0;col<4;++col) {
      GridAssociationHypothesis candidate;candidate.quarter_turns=turn;
      candidate.row_offset=row-anchor[0];candidate.col_offset=col-anchor[1];
      bool accepted=true;
      for(const auto& node:input) {
        auto index=rotate_grid_node(node.row,node.col,turn);
        index[0]+=candidate.row_offset;index[1]+=candidate.col_offset;
        if(index[0]<0||index[0]>3||index[1]<0||index[1]>3){accepted=false;break;}
        const auto expected=grid_node_arms(index[0],index[1]);
        const auto observed=rotate_grid_arms(node.arms,turn);
        const auto tested=rotate_grid_arms(node.tested_directions,turn);
        const auto yellow=rotate_grid_arms(node.yellow_arms,turn);
        if((expected&tested)!=observed || (yellow && (index[0]!=3 || (yellow&5U)))) {
          accepted=false;break;
        }
        candidate.nodes.push_back(index);
      }
      if(accepted)result.hypotheses.push_back(candidate);
    }
  }
  result.unique=result.hypotheses.size()==1;
  result.reason=result.unique?"unique topology association":
    result.hypotheses.empty()?"topology or yellow-edge conflict":"ambiguous repeated grid";
  return result;
}
}
