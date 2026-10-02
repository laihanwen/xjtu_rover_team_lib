// Copyright 2026 hanwen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUV_PLANNING__GRID_PLANNER_HPP_
#define AUV_PLANNING__GRID_PLANNER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace auv_planning
{

struct GridCell
{
  std::int8_t row{0};
  std::int8_t col{0};
  std::string object_type{"unknown"};

  bool operator==(const GridCell & other) const;
  bool operator<(const GridCell & other) const;
};

struct PlanningCell
{
  GridCell cell;
  bool visited{false};
};

struct PlanningGrid
{
  std::uint8_t rows{0};
  std::uint8_t cols{0};
  bool complete{false};
  std::vector<PlanningCell> cells;
};

struct GridPlannerConfig
{
  std::vector<std::string> target_object_types{"circle_cone", "square_cone"};
  std::vector<std::string> blocked_object_types{"obstacle", "forbidden"};
  std::size_t maximum_targets{8U};
};

struct PlanResult
{
  bool valid{false};
  std::string reason;
  GridCell start;
  std::vector<GridCell> targets;
  std::vector<GridCell> path;
  float total_cost{0.0F};
};

class GridPlanner
{
public:
  explicit GridPlanner(GridPlannerConfig config = {});

  PlanResult plan(const PlanningGrid & grid, const GridCell & start) const;

private:
  std::vector<GridCell> shortest_path(
    const PlanningGrid & grid, const GridCell & start, const GridCell & goal) const;
  bool has_type(const std::vector<std::string> & types, const std::string & value) const;
  bool is_blocked(const PlanningGrid & grid, int row, int col) const;

  GridPlannerConfig config_;
};

}  // namespace auv_planning

#endif  // AUV_PLANNING__GRID_PLANNER_HPP_
