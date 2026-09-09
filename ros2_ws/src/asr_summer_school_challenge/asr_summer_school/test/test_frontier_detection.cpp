// Regression tests for the frontier search.
//
// The first test here is the one that matters.  Karto sizes the occupancy grid
// to the bounding box of the scan *endpoints* - which excludes the no-return
// rays - while rastering those same rays as free space out to the range
// threshold.  Free space is therefore routinely clipped at the grid boundary
// with no unknown margin beyond it, and a search that skips out-of-bounds
// neighbours sees no frontier there at all.
//
// Observed in a real run: 87% of a 4 x 3.5 m map known, the other 396 m² of the
// arena never visited, and the mission reporting "exploration complete" and
// driving home after two goals.  Nothing errored.

#include <gtest/gtest.h>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "asr_summer_school/frontier_detection.h"

namespace
{
constexpr double kRes = 0.05;
constexpr int8_t kFree = 0;
constexpr int8_t kUnknown = -1;
constexpr int8_t kOccupied = 100;

// A grid whose origin is placed so that world (0, 0) is the centre cell.
nav_msgs::msg::OccupancyGrid MakeGrid(int width, int height, int8_t fill)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = kRes;
  grid.info.origin.position.x = -(width / 2) * kRes;
  grid.info.origin.position.y = -(height / 2) * kRes;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<size_t>(width) * height, fill);
  return grid;
}

int CountFrontiers(const nav_msgs::msg::OccupancyGrid & marked)
{
  int total = 0;
  for (const auto value : marked.data) {
    if (value == 100) {++total;}
  }
  return total;
}

int8_t & At(nav_msgs::msg::OccupancyGrid & grid, int x, int y)
{
  return grid.data[static_cast<size_t>(y) * grid.info.width + x];
}
}  // namespace

// The regression.  Every cell of this grid is free, so there is no unknown
// cell anywhere - but the map simply stops at its edge, and what lies past the
// edge has certainly not been observed.
TEST(PreprocessFrontierCells, FreeSpaceAtTheGridEdgeIsAFrontier)
{
  auto grid = MakeGrid(11, 11, kFree);

  const auto marked = frontier_detection::preprocess_frontier_cells(grid, 0.0, 0.0, 30.0);

  // The border ring: 11 * 4 - 4 corners counted twice = 40 cells.
  EXPECT_EQ(CountFrontiers(marked), 40)
    << "free cells against the map boundary must be frontiers, or a robot in "
       "open space reports the arena explored and drives home";
}

TEST(PreprocessFrontierCells, InteriorFreeSpaceIsNotAFrontier)
{
  auto grid = MakeGrid(11, 11, kFree);

  const auto marked = frontier_detection::preprocess_frontier_cells(grid, 0.0, 0.0, 30.0);

  // Centre cell, well away from the border.
  EXPECT_EQ(marked.data[5 * 11 + 5], -1);
}

TEST(PreprocessFrontierCells, FreeSpaceBesideUnknownIsAFrontier)
{
  auto grid = MakeGrid(11, 11, kUnknown);
  // A 3x3 island of free space in the middle of an unknown map.  Every one of
  // the nine is either beside unknown or beside another free cell that is.
  for (int y = 4; y <= 6; ++y) {
    for (int x = 4; x <= 6; ++x) {
      At(grid, x, y) = kFree;
    }
  }

  const auto marked = frontier_detection::preprocess_frontier_cells(grid, 0.0, 0.0, 30.0);

  EXPECT_EQ(CountFrontiers(marked), 8) << "the eight ring cells, not the centre";
  EXPECT_EQ(marked.data[5 * 11 + 5], -1) << "the centre is enclosed by free cells";
}

// The search walks through free space only; a frontier behind a wall is not
// reachable and must not be proposed.
TEST(PreprocessFrontierCells, DoesNotCrossOccupiedCells)
{
  auto grid = MakeGrid(11, 11, kFree);
  for (int y = 0; y < 11; ++y) {
    At(grid, 7, y) = kOccupied;   // a wall down the middle-right
  }
  // Beyond the wall, make it unknown so it would otherwise be a rich frontier.
  for (int y = 0; y < 11; ++y) {
    for (int x = 8; x < 11; ++x) {
      At(grid, x, y) = kUnknown;
    }
  }

  const auto marked = frontier_detection::preprocess_frontier_cells(grid, 0.0, 0.0, 30.0);

  for (int y = 0; y < 11; ++y) {
    for (int x = 8; x < 11; ++x) {
      EXPECT_NE(marked.data[static_cast<size_t>(y) * 11 + x], 100)
        << "cell (" << x << ", " << y << ") is behind a wall";
    }
  }
  // The free column against the wall is still explorable via the map edge.
  EXPECT_GT(CountFrontiers(marked), 0);
}

// The radius caps how far the search may walk, and that cap is exactly why it
// must not be pinned to the sensor horizon.  Inside a fully-explored active
// area there is nothing to find, so the detector goes silent and the mission
// concludes the arena is finished - however much unexplored space lies just
// past the radius.  The same grid with a radius that reaches the map edge
// yields the whole border.
TEST(PreprocessFrontierCells, TheActiveAreaRadiusBoundsTheSearch)
{
  auto grid = MakeGrid(41, 41, kFree);

  const auto near = frontier_detection::preprocess_frontier_cells(grid, 0.0, 0.0, 0.15);
  EXPECT_EQ(CountFrontiers(near), 0)
    << "everything within 0.15 m is free and known, so there is nothing to "
       "explore *within the radius* - which is why a radius set to the sensor "
       "horizon makes the robot stop as soon as its own pocket is mapped";

  const auto far = frontier_detection::preprocess_frontier_cells(grid, 0.0, 0.0, 30.0);
  EXPECT_EQ(CountFrontiers(far), 41 * 4 - 4) << "the whole border ring";
}

TEST(PreprocessFrontierCells, RobotOutsideTheGridYieldsNothing)
{
  auto grid = MakeGrid(11, 11, kFree);

  const auto marked = frontier_detection::preprocess_frontier_cells(grid, 100.0, 100.0, 30.0);

  EXPECT_EQ(CountFrontiers(marked), 0);
}
