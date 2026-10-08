#pragma once
#include <array>
#include "auv_mapping/grid_mapper.hpp"
#include "auv_planning/grid_planner.hpp"
#include "auv_vision/cone_detector.hpp"

namespace auv_core {
struct SemanticMap {
  auv_planning::PlanningGrid grid;
  std::array<bool, 9> visited{};
  bool complete{};
  int cone_count{};
};
SemanticMap fuse_semantic_map(const auv_mapping::GridResult& geometry,
  const std::vector<auv_vision::ConeObservation>& cones, bool cones_stable,
  const std::array<bool, 9>& visited, int expected_cones);
}
