#include "auv_mapping/local_map_accumulator.hpp"
#include <stdexcept>
#include <limits>
using namespace auv_mapping;
static void check(bool value){if(!value)throw std::runtime_error("local map regression failed");}
int main(){
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
