#include "auv_core/semantic_map.hpp"
#include <algorithm>

namespace auv_core {
SemanticMap fuse_semantic_map(const auv_mapping::GridResult& geometry,
    const std::vector<auv_vision::ConeObservation>& cones, bool cones_stable,
    const std::array<bool, 9>& visited, int expected_cones) {
  SemanticMap result;
  result.grid.rows = result.grid.cols = 3;
  result.visited = visited;
  std::array<const auv_vision::ConeObservation*, 9> best{};
  for (const auto& cone : cones) {
    if (cone.row < 0 || cone.row >= 3 || cone.col < 0 || cone.col >= 3) continue;
    auto& slot = best[static_cast<std::size_t>(cone.row * 3 + cone.col)];
    if (!slot || slot->confidence < cone.confidence) slot = &cone;
  }
  result.cone_count = std::count_if(best.begin(), best.end(), [](auto* p) { return p != nullptr; });
  const bool classified = std::all_of(best.begin(),best.end(),[](const auto* cone) {
    return !cone || cone->shape == auv_vision::ConeShape::kCircle || cone->shape == auv_vision::ConeShape::kSquare;
  });
  result.complete = geometry.stable && cones_stable && classified && result.cone_count == expected_cones;
  result.grid.complete = result.complete;
  for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) {
    const auto* cone = best[static_cast<std::size_t>(r * 3 + c)];
    const char* type = "unknown";
    if (geometry.stable && cone) {
      if (cone->shape == auv_vision::ConeShape::kCircle) type = "circle_cone";
      if (cone->shape == auv_vision::ConeShape::kSquare) type = "square_cone";
    }
    result.grid.cells.push_back({{static_cast<std::int8_t>(r), static_cast<std::int8_t>(c), type},
      visited[static_cast<std::size_t>(r * 3 + c)]});
  }
  return result;
}
}
