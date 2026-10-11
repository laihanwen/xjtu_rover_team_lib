#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace auv_mapping {
enum class LocalCellClass { unknown, empty, circle, square };
struct LocalCellEvidence {
  int row{-1},col{-1}; LocalCellClass type{LocalCellClass::unknown};
  bool full_cell_visible{false}; double confidence{0};
  // Measured silhouette evidence; coordinate frame must be stated by producer.
  std::vector<std::array<double,2>> contour;
  std::string contour_frame;
};
struct LocalMapObservation {
  std::uint64_t epoch{0},sequence{0}; int camera{0}; double stamp{0};
  std::string grid_frame; bool association_unique{false};
  std::vector<LocalCellEvidence> cells;
};
struct AccumulatedCell {
  LocalCellClass type{LocalCellClass::unknown};
  std::array<unsigned,3> votes{}; bool conflict{false},observed{false};
  double stamp{0}; int camera{0}; std::uint64_t sequence{0};
  std::vector<std::array<double,2>> contour;
  std::string contour_frame;
};
// Consumes already associated observations. It does NOT infer global grid IDs
// from line candidates or treat commanded motion as measured position.
class LocalMapAccumulator {
 public:
  void reset(std::uint64_t epoch,const std::string& frame) {
    epoch_=epoch;frame_=frame;cells_={};sequence_={};stamp_={};has_sample_={};boundary_=0;
  }
  // A stage transition retains semantic evidence in the same map frame only;
  // the next observation must have new-stage association and fresh timestamp.
  bool begin_stage(std::uint64_t epoch,double boundary,const std::string& frame) {
    if(epoch<=epoch_||!std::isfinite(boundary)||boundary<boundary_||frame!=frame_||frame.empty())return false;
    epoch_=epoch;boundary_=boundary;sequence_={};stamp_={};has_sample_={};return true;
  }
  bool update(const LocalMapObservation& input,double now,std::string& reason) {
    if(!std::isfinite(now)||!std::isfinite(input.stamp)||input.stamp<boundary_||
       input.stamp>now||now-input.stamp>.3||input.epoch!=epoch_) {
      reason="stale or wrong-stage observation";return false;
    }
    if(input.camera<0||input.camera>1||!input.sequence||!input.association_unique||
       input.grid_frame.empty()||input.grid_frame!=frame_) {
      reason="grid association not unique or frame mismatch";return false;
    }
    const auto camera=static_cast<std::size_t>(input.camera);
    if(has_sample_[camera]&&(input.sequence<=sequence_[camera]||input.stamp<=stamp_[camera])) {
      reason="duplicate or reordered frame";return false;
    }
    std::array<bool,9> occupied{};
    for(const auto& evidence:input.cells) {
      if(evidence.row<0||evidence.row>2||evidence.col<0||evidence.col>2||
         !std::isfinite(evidence.confidence)||evidence.confidence<0||evidence.confidence>1||
         (evidence.type!=LocalCellClass::unknown&&evidence.type!=LocalCellClass::empty&&
          evidence.type!=LocalCellClass::circle&&evidence.type!=LocalCellClass::square)) {
        reason="invalid cell evidence";return false;
      }
      const auto index=static_cast<std::size_t>(3*evidence.row+evidence.col);
      if(occupied[index]){reason="duplicate cell in frame";return false;}
      occupied[index]=true;
      if(!evidence.contour.empty()&&evidence.contour_frame.empty()) {
        reason="contour coordinate frame missing";return false;
      }
      for(const auto& point:evidence.contour)if(!std::isfinite(point[0])||!std::isfinite(point[1])) {
        reason="invalid contour";return false;
      }
    }
    sequence_[camera]=input.sequence;stamp_[camera]=input.stamp;has_sample_[camera]=true;
    for(const auto& evidence:input.cells) {
      if(evidence.type==LocalCellClass::unknown||evidence.confidence<.8)continue;
      if(evidence.type==LocalCellClass::empty&&!evidence.full_cell_visible)continue;
      auto& cell=cells_[static_cast<std::size_t>(3*evidence.row+evidence.col)];
      const int label=evidence.type==LocalCellClass::empty?0:evidence.type==LocalCellClass::circle?1:2;
      // Only threshold support is needed; saturate to avoid long-run overflow.
      if(cell.votes[label]<3)++cell.votes[label];
      cell.observed=true;cell.stamp=input.stamp;cell.camera=input.camera;cell.sequence=input.sequence;
      int supported=0,winner=-1;
      for(int i=0;i<3;++i)if(cell.votes[i]>=3){++supported;winner=i;}
      // Conflicting strong evidence never becomes a majority-vote certainty.
      cell.conflict=supported>1;cell.type=LocalCellClass::unknown;
      if(supported==1)cell.type=winner==0?LocalCellClass::empty:winner==1?LocalCellClass::circle:LocalCellClass::square;
      // Never attach the losing or unresolved class's silhouette to a winner.
      cell.contour.clear();cell.contour_frame.clear();
      if(cell.type==evidence.type) {
        cell.contour=evidence.contour;cell.contour_frame=evidence.contour_frame;
      }
    }
    reason="local semantic evidence accumulated";return true;
  }
  bool complete(int expected=4) const {
    int cones=0;for(const auto& cell:cells_) {
      if(cell.conflict||cell.type==LocalCellClass::unknown)return false;
      if(cell.type==LocalCellClass::circle||cell.type==LocalCellClass::square)++cones;
    }
    return cones==expected;
  }
  const std::array<AccumulatedCell,9>& cells()const{return cells_;}
 private:
  std::array<AccumulatedCell,9> cells_{};
  std::array<std::uint64_t,2> sequence_{};std::array<double,2> stamp_{};
  std::array<bool,2> has_sample_{};std::uint64_t epoch_{0};double boundary_{0};std::string frame_;
};
}
