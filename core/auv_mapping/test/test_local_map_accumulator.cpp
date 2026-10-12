#include "auv_mapping/local_map_accumulator.hpp"
#include <stdexcept>
#include <limits>
using namespace auv_mapping;
static void check_impl(bool value,int line){if(!value)throw std::runtime_error("local map regression failed at line "+std::to_string(line));}
#define check(value) check_impl((value),__LINE__)
static void coverage_regression() {
  LocalMapAccumulator map;map.reset(1,"grid");std::string reason;
  LocalMapObservation frame;frame.epoch=1;frame.grid_frame="grid";frame.association_unique=true;
  frame.cells.resize(1);auto& cell=frame.cells[0];cell.row=cell.col=0;
  auto send=[&](unsigned sequence){frame.sequence=sequence;frame.stamp=sequence*.1;return map.update(frame,frame.stamp,reason);};
  for(int i=0;i<kCoverageTiles/2;++i)cell.coverage[i]=kVisible|kEmptyFloor;
  for(unsigned i=1;i<=3;++i)check(send(i));
  check(map.cells()[0].type==LocalCellClass::unknown&&map.cells()[0].coverage_fraction()==.5);
  check(!send(3)); // Cached image cannot supply another coverage vote.
  cell.coverage={};for(int i=kCoverageTiles/2;i<kCoverageTiles;++i)cell.coverage[i]=kVisible|kEmptyFloor;
  frame.association_unique=false;check(!send(4));
  check(map.cells()[0].coverage_fraction()==.5);frame.association_unique=true;
  check(send(4)&&send(5));check(map.cells()[0].type==LocalCellClass::unknown);
  check(send(6));check(map.cells()[0].type==LocalCellClass::empty);
  check(map.cells()[0].coverage_fraction()==1&&map.cells()[0].empty_support_fraction()==1);
  check(!map.complete()); // Full coverage of one cell says nothing about others.
  cell.coverage={};cell.coverage[0]=kOccupied;check(send(7));
  check(map.cells()[0].type==LocalCellClass::unknown); // Unclassified target cannot disappear into floor votes.
  cell.coverage.fill(kVisible|kEmptyFloor);
  for(unsigned i=8;i<=10;++i)check(send(i));
  check(map.cells()[0].type==LocalCellClass::unknown&&map.cells()[0].empty_support_fraction()<1);
  map.reset(2,"grid");frame.epoch=2;cell.coverage.fill(kVisible);
  for(unsigned i=1;i<=3;++i)check(send(i));
  check(map.cells()[0].coverage_fraction()==1&&map.cells()[0].type==LocalCellClass::unknown);
  cell.coverage[0]=kEmptyFloor;check(!send(4)); // Empty needs an actually visible tile.
  cell.coverage[0]=8;check(!send(4));
  bool rejected=false;try{LocalMapAccumulator invalid({2});}catch(const std::invalid_argument&){rejected=true;}check(rejected);
  // Complete silhouette classification does not require seeing the whole cell.
  map.reset(3,"grid");frame.epoch=3;cell.coverage={};cell.type=LocalCellClass::circle;
  cell.confidence=.9;cell.full_cell_visible=false;cell.contour={{30,30},{40,30},{40,40}};cell.contour_frame="grid";
  for(unsigned i=1;i<=3;++i)check(send(i));check(map.cells()[0].type==LocalCellClass::circle);
}
int main(){
  coverage_regression();
  LocalMapAccumulator map;map.reset(1,"grid");std::string reason;
  LocalMapObservation observation;observation.epoch=1;observation.grid_frame="grid";observation.association_unique=true;
  observation.cells={{0,0,LocalCellClass::empty,false,.9,{}}};
  for(int i=1;i<=3;++i){observation.sequence=i;observation.stamp=i*.1;check(map.update(observation,i*.1,reason));}
  check(map.cells()[0].type==LocalCellClass::unknown); // cropped empty cannot clear a cell
  check(!map.update(observation,.3,reason)); // repeated cached frame cannot vote
  observation.sequence=4;observation.stamp=.4;observation.association_unique=false;
  check(!map.update(observation,.4,reason));observation.association_unique=true;
  check(!map.update(observation,1.,reason)); // stale absolute association
  observation.cells[0].full_cell_visible=true;
  for(int i=4;i<=6;++i){observation.sequence=i;observation.stamp=i*.1;check(map.update(observation,i*.1,reason));}
  check(map.cells()[0].type==LocalCellClass::empty);
  check(!map.complete()); // unobserved cells remain unknown
  check(map.begin_stage(2,1.,"grid"));check(!map.update(observation,1.,reason));
  observation.epoch=2;observation.sequence=1;observation.stamp=.9;check(!map.update(observation,1.,reason));
  observation.stamp=1.1;check(map.update(observation,1.1,reason));
  check(!map.begin_stage(3,2.,"other-grid")); // cannot silently mix maps
  observation.cells[0].type=LocalCellClass::square;
  for(int i=2;i<=4;++i){observation.sequence=i;observation.stamp=1.+i*.1;check(map.update(observation,observation.stamp,reason));}
  check(map.cells()[0].conflict&&map.cells()[0].type==LocalCellClass::unknown);
  map.reset(3,"grid");observation.epoch=3;observation.sequence=1;observation.stamp=2.;
  observation.cells.push_back(observation.cells[0]);check(!map.update(observation,2.,reason));
  observation.cells.pop_back();observation.stamp=.1;check(map.update(observation,.1,reason)); // reset clears boundary
  map.reset(4,"grid");observation.epoch=4;observation.cells.clear();
  for(int row=0;row<3;++row)for(int col=0;col<3;++col)
    observation.cells.push_back({row,col,row==2|| (row==1&&col==0)?LocalCellClass::circle:LocalCellClass::empty,true,.9,{}});
  for(int i=1;i<=3;++i){observation.sequence=i;observation.stamp=i*.1;check(map.update(observation,i*.1,reason));}
  check(map.complete(4));check(!map.complete(3));
  check(map.begin_stage(5,2.,"grid"));
  check(!map.begin_stage(6,1.,"grid")); // stage boundary cannot move backwards
  observation.epoch=5;observation.sequence=1;observation.stamp=2.1;
  observation.cells.resize(1);observation.cells[0].contour={{1.,2.}};
  check(!map.update(observation,2.1,reason)); // silhouette coordinates must be explicit
  observation.cells[0].contour_frame="rectified_front_pixels";
  check(map.update(observation,2.1,reason));
  check(map.cells()[0].contour_frame=="rectified_front_pixels");
  observation.sequence=2;observation.stamp=2.2;observation.cells[0].type=LocalCellClass::square;
  check(map.update(observation,2.2,reason));
  check(map.cells()[0].type==LocalCellClass::empty && map.cells()[0].contour.empty());
  return 0;
}
