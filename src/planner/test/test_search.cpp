#include "planner/search.hpp"

#include <gtest/gtest.h>

#include <cmath>

using planner::GridMap;
using planner::Point2d;
using planner::SearchOptions;

namespace {

GridMap makeGrid(int width, int height, double resolution = 0.1) {
  GridMap grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  const size_t total = static_cast<size_t>(width) * static_cast<size_t>(height);
  grid.cost.assign(total, 1.0f);
  grid.blocked.assign(total, 0);
  return grid;
}

void inflate(GridMap* grid, int radius) {
  const GridMap source = *grid;
  for (int y = 0; y < grid->height; ++y) {
    for (int x = 0; x < grid->width; ++x) {
      if (!source.blockedAt(x, y)) continue;
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          if (grid->inside(x + dx, y + dy)) grid->blocked[grid->index(x + dx, y + dy)] = 1;
        }
      }
    }
  }
}

void blockRect(GridMap* grid, int x0, int y0, int x1, int y1) {
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (grid->inside(x, y)) grid->blocked[grid->index(x, y)] = 1;
    }
  }
}

planner::FreeFn alwaysFree() {
  return [](double, double, double) { return true; };
}

double pathLength(const std::vector<Point2d>& path) {
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return length;
}

bool pathClear(const GridMap& grid, const std::vector<Point2d>& path) {
  for (size_t i = 1; i < path.size(); ++i) {
    if (!planner::lineOfSightCells(grid, path[i - 1], path[i])) return false;
  }
  return true;
}

}

TEST(SearchTest, octileHeuristicMatchesMoves) {
  EXPECT_NEAR(planner::octile(0.0, 0.0), 0.0, 1e-12);
  EXPECT_NEAR(planner::octile(3.0, 0.0), 3.0, 1e-12);
  EXPECT_NEAR(planner::octile(0.0, -4.0), 4.0, 1e-12);
  EXPECT_NEAR(planner::octile(2.0, 1.0), 1.0 + std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(planner::octile(1.0, 2.0), 1.0 + std::sqrt(2.0), 1e-12);
}

TEST(SearchTest, bresenhamDetectsBlockedCell) {
  GridMap grid = makeGrid(10, 10);
  Point2d a{0.05, 0.05};
  Point2d b{0.95, 0.05};
  EXPECT_TRUE(planner::lineOfSightCells(grid, a, b));

  blockRect(&grid, 5, 0, 5, 9);
  EXPECT_FALSE(planner::lineOfSightCells(grid, a, b));
  EXPECT_TRUE(planner::lineOfSightCells(grid, a, {0.05, 0.95}));
}

TEST(SearchTest, malformedGridDataIsTreatedAsBlocked) {
  GridMap grid = makeGrid(4, 4);
  grid.blocked.clear();
  EXPECT_TRUE(grid.blockedAt(0, 0));
  EXPECT_FLOAT_EQ(grid.costAt(0, 0), 1.0f);

  planner::EsdfField field;
  field.width = 4;
  field.height = 4;
  field.resolution = 0.1;
  field.max_distance = 3.0f;
  EXPECT_FLOAT_EQ(field.at(0.0, 0.0), field.max_distance);
  field.data.assign(4, 1.0f);
  EXPECT_FLOAT_EQ(field.at(0.0, 0.0), 1.0f);
  EXPECT_FLOAT_EQ(field.at(0.25, 0.25), field.max_distance);
}

TEST(SearchTest, lineOfSightAcceptsEmptyFreeCallback) {
  GridMap grid = makeGrid(10, 10);
  planner::FreeFn no_checker;
  EXPECT_TRUE(planner::lineOfSight(grid, {0.05, 0.05}, {0.95, 0.05}, no_checker, 0.05));
  blockRect(&grid, 5, 0, 5, 9);
  EXPECT_FALSE(planner::lineOfSight(grid, {0.05, 0.05}, {0.95, 0.05}, no_checker, 0.05));
}

TEST(SearchTest, astarFindsPathAroundWall) {
  GridMap grid = makeGrid(30, 30);
  blockRect(&grid, 15, 0, 15, 20);

  SearchOptions options;
  options.angle_search = false;
  const Point2d start{0.55, 0.55};
  const Point2d goal{2.55, 0.55};
  const planner::SearchResult result =
      planner::planAStar(grid, start, goal, options, alwaysFree());

  ASSERT_TRUE(result.success);
  EXPECT_GT(result.expanded, 0u);
  EXPECT_GT(result.path.size(), 2u);
  EXPECT_TRUE(pathClear(grid, result.path));
  EXPECT_NEAR(result.path.front().x, 0.55, 0.06);
  EXPECT_NEAR(result.path.front().y, 0.55, 0.06);
  EXPECT_LT(std::hypot(result.path.back().x - goal.x, result.path.back().y - goal.y), 0.08);

  const double straight = std::hypot(goal.x - start.x, goal.y - start.y);
  EXPECT_GT(pathLength(result.path), straight);
}

TEST(SearchTest, astarFailsWhenGoalEnclosed) {
  GridMap grid = makeGrid(20, 20);
  blockRect(&grid, 10, 10, 10, 14);
  blockRect(&grid, 14, 10, 14, 14);
  blockRect(&grid, 10, 10, 14, 10);
  blockRect(&grid, 10, 14, 14, 14);

  SearchOptions options;
  options.angle_search = false;
  const planner::SearchResult result =
      planner::planAStar(grid, {0.55, 0.55}, {1.25, 1.25}, options, alwaysFree());
  EXPECT_FALSE(result.success);
}

TEST(SearchTest, thetaStarProducesStraightLineInOpenSpace) {
  GridMap grid = makeGrid(40, 40);
  SearchOptions options;
  options.angle_search = true;
  const Point2d start{0.55, 0.55};
  const Point2d goal{3.55, 2.55};
  const planner::SearchResult result =
      planner::planThetaStar(grid, start, goal, options, alwaysFree());

  ASSERT_TRUE(result.success);
  EXPECT_LE(result.path.size(), 3u);
  EXPECT_NEAR(pathLength(result.path),
              std::hypot(goal.x - start.x, goal.y - start.y), 0.2);
}

TEST(SearchTest, thetaStarKeepsClearanceAroundObstacle) {
  GridMap grid = makeGrid(40, 40);
  blockRect(&grid, 18, 10, 22, 30);
  inflate(&grid, 1);

  auto is_free = [&grid](double x, double y, double) {
    int cx = 0;
    int cy = 0;
    return grid.worldToCell(x, y, &cx, &cy) && !grid.blockedAt(cx, cy);
  };

  SearchOptions options;
  options.angle_search = true;
  const Point2d start{0.55, 1.55};
  const Point2d goal{3.55, 1.55};
  const planner::SearchResult result = planner::planThetaStar(grid, start, goal, options, is_free);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(pathClear(grid, result.path));
  for (const Point2d& point : result.path) {
    EXPECT_TRUE(is_free(point.x, point.y, 0.0));
  }
}

TEST(SearchTest, planFallsBackToAStarWhenThetaStarFails) {
  GridMap grid = makeGrid(30, 30);
  blockRect(&grid, 15, 0, 15, 22);
  SearchOptions options;
  options.angle_search = true;
  const planner::SearchResult result =
      planner::plan(grid, {0.55, 0.55}, {2.55, 0.55}, options, alwaysFree());
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(pathClear(grid, result.path));
}
