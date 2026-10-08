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

#include <stdexcept>
#include <string>

#include "auv_planning/grid_planner.hpp"
#include "gtest/gtest.h"

namespace
{

auv_planning::PlanningGrid make_grid()
{
  auv_planning::PlanningGrid grid;
  grid.rows = 3U;
  grid.cols = 3U;
  grid.complete = true;
  for (std::int8_t row = 0; row < 3; ++row) {
    for (std::int8_t col = 0; col < 3; ++col) {
      grid.cells.push_back({{row, col, "unknown"}, false});
    }
  }
  return grid;
}

void set_cell(
  auv_planning::PlanningGrid & grid, int row, int col, const std::string & object,
  bool visited = false)
{
  auto & cell = grid.cells[static_cast<std::size_t>(row * grid.cols + col)];
  cell.cell.object_type = object;
  cell.visited = visited;
}

TEST(GridPlanner, RejectsIncompleteInvalidAndUnconfiguredMaps)
{
  auv_planning::GridPlanner planner;
  auto grid = make_grid();
  grid.complete = false;
  EXPECT_FALSE(planner.plan(grid, {0, 0, "unknown"}).valid);
  grid.complete = true;
  EXPECT_FALSE(planner.plan(grid, {-1, -1, "unknown"}).valid);
  grid.cells.pop_back();
  EXPECT_FALSE(planner.plan(grid, {0, 0, "unknown"}).valid);
}

TEST(GridPlanner, FindsSingleTargetAndObstacleDetour)
{
  auv_planning::GridPlanner planner;
  auto grid = make_grid();
  set_cell(grid, 0, 0, "circle_cone");
  set_cell(grid, 1, 0, "obstacle");
  const auto result = planner.plan(grid, {2, 0, "unknown"});
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_FLOAT_EQ(result.total_cost, 4.0F);
  ASSERT_EQ(result.path.size(), 5U);
  EXPECT_EQ(result.path.front().row, 2);
  EXPECT_EQ(result.path.front().col, 0);
  EXPECT_EQ(result.path.back().row, 0);
  EXPECT_EQ(result.path.back().col, 0);
  for (std::size_t index = 1U; index < result.path.size(); ++index) {
    const int distance =
      std::abs(result.path[index].row - result.path[index - 1U].row) +
      std::abs(result.path[index].col - result.path[index - 1U].col);
    EXPECT_EQ(distance, 1);
  }
}

TEST(GridPlanner, ChoosesMinimumCostTargetPermutation)
{
  auv_planning::GridPlanner planner;
  auto grid = make_grid();
  set_cell(grid, 0, 0, "circle_cone");
  set_cell(grid, 2, 2, "square_cone");
  const auto result = planner.plan(grid, {2, 1, "unknown"});
  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(result.targets.size(), 2U);
  EXPECT_EQ(result.targets[0].row, 2);
  EXPECT_EQ(result.targets[0].col, 2);
  EXPECT_EQ(result.targets[1].row, 0);
  EXPECT_EQ(result.targets[1].col, 0);
  EXPECT_FLOAT_EQ(result.total_cost, 5.0F);
}

TEST(GridPlanner, UsesDeterministicLexicographicTieBreak)
{
  auv_planning::GridPlanner planner;
  auto grid = make_grid();
  set_cell(grid, 1, 0, "square_cone");
  set_cell(grid, 1, 2, "circle_cone");
  const auto first = planner.plan(grid, {1, 1, "unknown"});
  const auto second = planner.plan(grid, {1, 1, "unknown"});
  ASSERT_TRUE(first.valid);
  ASSERT_EQ(first.targets.size(), 2U);
  EXPECT_EQ(first.targets[0].col, 0);
  EXPECT_EQ(first.path, second.path);
}

TEST(GridPlanner, ExcludesVisitedTargetsAndHandlesNoTargets)
{
  auv_planning::GridPlanner planner;
  auto grid = make_grid();
  set_cell(grid, 0, 0, "circle_cone", true);
  const auto result = planner.plan(grid, {2, 2, "unknown"});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.reason, "no unvisited targets");
  EXPECT_TRUE(result.targets.empty());
  ASSERT_EQ(result.path.size(), 1U);
  EXPECT_FLOAT_EQ(result.total_cost, 0.0F);
}

TEST(GridPlanner, HandlesTargetAtStart)
{
  auv_planning::GridPlanner planner;
  auto grid = make_grid();
  set_cell(grid, 2, 2, "circle_cone");
  const auto result = planner.plan(grid, {2, 2, "unknown"});
  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(result.targets.size(), 1U);
  ASSERT_EQ(result.path.size(), 1U);
  EXPECT_FLOAT_EQ(result.total_cost, 0.0F);
}

TEST(GridPlanner, ReportsBlockedStartAndUnreachableTarget)
{
  auv_planning::GridPlanner planner;
  auto grid = make_grid();
  set_cell(grid, 2, 0, "obstacle");
  EXPECT_EQ(planner.plan(grid, {2, 0, "unknown"}).reason, "start cell is blocked");

  grid = make_grid();
  set_cell(grid, 0, 0, "circle_cone");
  set_cell(grid, 1, 0, "obstacle");
  set_cell(grid, 1, 1, "obstacle");
  set_cell(grid, 1, 2, "obstacle");
  const auto result = planner.plan(grid, {2, 0, "unknown"});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "one or more targets are unreachable");
}

TEST(GridPlanner, EnforcesTargetLimitAndRowMajorCells)
{
  auv_planning::GridPlannerConfig config;
  config.maximum_targets = 1U;
  auv_planning::GridPlanner planner(config);
  auto grid = make_grid();
  set_cell(grid, 0, 0, "circle_cone");
  set_cell(grid, 0, 1, "square_cone");
  EXPECT_EQ(
    planner.plan(grid, {2, 2, "unknown"}).reason,
    "target count exceeds maximum_targets");

  grid = make_grid();
  std::swap(grid.cells[0], grid.cells[1]);
  EXPECT_EQ(
    planner.plan(grid, {2, 2, "unknown"}).reason,
    "semantic map cells are not in row-major order");
}

TEST(GridPlanner, ValidatesConfiguration)
{
  auv_planning::GridPlannerConfig config;
  config.maximum_targets = 0U;
  EXPECT_THROW(auv_planning::GridPlanner planner(config), std::invalid_argument);
  config.maximum_targets = 8U;
  config.blocked_object_types.push_back("circle_cone");
  EXPECT_THROW(auv_planning::GridPlanner planner(config), std::invalid_argument);
}

}  // namespace
