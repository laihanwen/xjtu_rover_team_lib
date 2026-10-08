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

#include "auv_planning/grid_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace auv_planning
{

bool GridCell::operator==(const GridCell & other) const
{
  return row == other.row && col == other.col && object_type == other.object_type;
}

bool GridCell::operator<(const GridCell & other) const
{
  return std::tie(row, col, object_type) < std::tie(other.row, other.col, other.object_type);
}

GridPlanner::GridPlanner(GridPlannerConfig config)
: config_(std::move(config))
{
  if (config_.target_object_types.empty()) {
    throw std::invalid_argument("target_object_types must not be empty");
  }
  if (config_.maximum_targets == 0U) {
    throw std::invalid_argument("maximum_targets must be positive");
  }
  for (const auto & type : config_.target_object_types) {
    if (type.empty()) {
      throw std::invalid_argument("target object types must not be empty");
    }
    if (has_type(config_.blocked_object_types, type)) {
      throw std::invalid_argument("an object type cannot be both target and blocked");
    }
  }
}

bool GridPlanner::has_type(
  const std::vector<std::string> & types, const std::string & value) const
{
  return std::find(types.begin(), types.end(), value) != types.end();
}

bool GridPlanner::is_blocked(const PlanningGrid & grid, int row, int col) const
{
  const auto index = static_cast<std::size_t>(row * grid.cols + col);
  return has_type(config_.blocked_object_types, grid.cells[index].cell.object_type);
}

std::vector<GridCell> GridPlanner::shortest_path(
  const PlanningGrid & grid, const GridCell & start, const GridCell & goal) const
{
  const int rows = grid.rows;
  const int cols = grid.cols;
  const int cell_count = rows * cols;
  const auto id = [cols](int row, int col) {return row * cols + col;};
  const int start_id = id(start.row, start.col);
  const int goal_id = id(goal.row, goal.col);
  std::vector<int> distance(static_cast<std::size_t>(cell_count), std::numeric_limits<int>::max());
  std::vector<int> parent(static_cast<std::size_t>(cell_count), -1);
  using QueueEntry = std::tuple<int, int, int, int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
  distance[static_cast<std::size_t>(start_id)] = 0;
  open.emplace(
    std::abs(start.row - goal.row) + std::abs(start.col - goal.col), 0,
    start.row, start.col);
  constexpr std::array<std::array<int, 2>, 4> offsets{{
    {{-1, 0}}, {{0, -1}}, {{0, 1}}, {{1, 0}}
  }};

  while (!open.empty()) {
    const auto [unused_score, current_distance, row, col] = open.top();
    (void)unused_score;
    open.pop();
    const int current_id = id(row, col);
    if (current_distance != distance[static_cast<std::size_t>(current_id)]) {
      continue;
    }
    if (current_id == goal_id) {
      break;
    }
    for (const auto & offset : offsets) {
      const int next_row = row + offset[0];
      const int next_col = col + offset[1];
      if (next_row < 0 || next_row >= rows || next_col < 0 || next_col >= cols ||
        is_blocked(grid, next_row, next_col))
      {
        continue;
      }
      const int next_id = id(next_row, next_col);
      const int tentative = current_distance + 1;
      if (tentative >= distance[static_cast<std::size_t>(next_id)]) {
        continue;
      }
      distance[static_cast<std::size_t>(next_id)] = tentative;
      parent[static_cast<std::size_t>(next_id)] = current_id;
      const int heuristic = std::abs(next_row - goal.row) + std::abs(next_col - goal.col);
      open.emplace(tentative + heuristic, tentative, next_row, next_col);
    }
  }

  if (distance[static_cast<std::size_t>(goal_id)] == std::numeric_limits<int>::max()) {
    return {};
  }
  std::vector<GridCell> reversed;
  for (int current = goal_id; current >= 0; current = parent[static_cast<std::size_t>(current)]) {
    reversed.push_back(grid.cells[static_cast<std::size_t>(current)].cell);
    if (current == start_id) {
      break;
    }
  }
  std::reverse(reversed.begin(), reversed.end());
  return reversed;
}

PlanResult GridPlanner::plan(const PlanningGrid & grid, const GridCell & start) const
{
  PlanResult result;
  result.start = start;
  if (!grid.complete) {
    result.reason = "semantic map is incomplete";
    return result;
  }
  if (grid.rows == 0U || grid.cols == 0U ||
    grid.cells.size() != static_cast<std::size_t>(grid.rows) * grid.cols)
  {
    result.reason = "semantic map dimensions are invalid";
    return result;
  }
  std::vector<bool> occupied(grid.cells.size(), false);
  for (std::size_t position = 0U; position < grid.cells.size(); ++position) {
    const auto & planning_cell = grid.cells[position];
    const int row = planning_cell.cell.row;
    const int col = planning_cell.cell.col;
    if (row < 0 || row >= grid.rows || col < 0 || col >= grid.cols) {
      result.reason = "semantic map contains an out-of-range cell";
      return result;
    }
    const auto index = static_cast<std::size_t>(row * grid.cols + col);
    if (index != position) {
      result.reason = "semantic map cells are not in row-major order";
      return result;
    }
    if (occupied[index]) {
      result.reason = "semantic map contains duplicate cells";
      return result;
    }
    occupied[index] = true;
  }
  if (start.row < 0 || start.row >= grid.rows || start.col < 0 || start.col >= grid.cols) {
    result.reason = "start cell is not configured or out of range";
    return result;
  }
  if (is_blocked(grid, start.row, start.col)) {
    result.reason = "start cell is blocked";
    return result;
  }
  result.start = grid.cells[static_cast<std::size_t>(start.row * grid.cols + start.col)].cell;

  std::vector<GridCell> targets;
  for (const auto & planning_cell : grid.cells) {
    if (!planning_cell.visited &&
      has_type(config_.target_object_types, planning_cell.cell.object_type))
    {
      targets.push_back(planning_cell.cell);
    }
  }
  std::sort(targets.begin(), targets.end());
  if (targets.size() > config_.maximum_targets) {
    result.reason = "target count exceeds maximum_targets";
    return result;
  }
  if (targets.empty()) {
    result.valid = true;
    result.reason = "no unvisited targets";
    result.path.push_back(grid.cells[static_cast<std::size_t>(start.row * grid.cols +
      start.col)].cell);
    return result;
  }

  if (config_.forbid_target_reentry) {
    // On a 3x3 field, BFS over (cell, visited-target mask) finds the shortest
    // whole path. Entering an unvisited target counts immediately; entering
    // one already present in the mask is forbidden, including transit cells.
    if (grid.rows != 3 || grid.cols != 3 || targets.size() > 4) {
      result.reason = "strict traversal requires 3x3 and at most four targets";
      return result;
    }
    std::array<int,9> bit{}; bit.fill(-1);
    for (std::size_t i=0;i<targets.size();++i) bit[targets[i].row*3+targets[i].col]=static_cast<int>(i);
    const int masks=1<<targets.size(), full=masks-1, sid=start.row*3+start.col;
    if(grid.cells[sid].visited && has_type(config_.target_object_types,grid.cells[sid].cell.object_type)) {
      result.reason="strict start is an already visited target";return result;
    }
    const int smask=bit[sid]<0 ? 0 : 1<<bit[sid];
    const int initial=smask*9+sid;
    std::vector<int> parent(static_cast<std::size_t>(masks*9),-1);
    parent[initial]=initial;
    std::queue<int> open;open.push(initial);int goal=-1;
    while(!open.empty()) {
      const int state=open.front();open.pop();const int mask=state/9,id=state%9;
      if(mask==full){goal=state;break;}
      for(const auto& delta : std::array<std::array<int,2>,4>{{{{-1,0}},{{0,-1}},{{0,1}},{{1,0}}}}) {
        const int r=id/3+delta[0],c=id%3+delta[1];
        if(r<0||r>=3||c<0||c>=3||is_blocked(grid,r,c))continue;
        const int next=r*3+c;
        if(grid.cells[next].visited && has_type(config_.target_object_types,grid.cells[next].cell.object_type))continue;
        if(bit[next]>=0 && (mask & (1<<bit[next])))continue;
        const int nm=bit[next]<0 ? mask : mask | (1<<bit[next]);
        const int ns=nm*9+next;if(parent[ns]>=0)continue;
        parent[ns]=state;open.push(ns);
      }
    }
    if(goal<0){result.reason="no route without target reentry";return result;}
    for(int state=goal;;state=parent[state]) {
      result.path.push_back(grid.cells[state%9].cell);if(state==initial)break;
    }
    std::reverse(result.path.begin(),result.path.end());
    for(const auto& cell:result.path) if(bit[cell.row*3+cell.col]>=0)result.targets.push_back(cell);
    result.valid=true;result.reason="whole path forbids target reentry";
    result.total_cost=static_cast<float>(result.path.size()-1);return result;
  }

  std::vector<GridCell> best_targets;
  std::vector<GridCell> best_path;
  std::size_t best_cost = std::numeric_limits<std::size_t>::max();
  do {
    GridCell current = start;
    std::vector<GridCell> candidate_path;
    bool reachable = true;
    for (const auto & target : targets) {
      auto segment = shortest_path(grid, current, target);
      if (segment.empty()) {
        reachable = false;
        break;
      }
      if (!candidate_path.empty()) {
        segment.erase(segment.begin());
      }
      candidate_path.insert(candidate_path.end(), segment.begin(), segment.end());
      current = target;
    }
    const std::size_t candidate_cost = candidate_path.empty() ? 0U : candidate_path.size() - 1U;
    if (reachable && candidate_cost < best_cost) {
      best_cost = candidate_cost;
      best_targets = targets;
      best_path = std::move(candidate_path);
    }
  } while (std::next_permutation(targets.begin(), targets.end()));

  if (best_path.empty()) {
    result.reason = "one or more targets are unreachable";
    return result;
  }
  result.valid = true;
  result.reason = "route planned";
  result.targets = std::move(best_targets);
  result.path = std::move(best_path);
  result.total_cost = static_cast<float>(best_cost);
  return result;
}

}  // namespace auv_planning
