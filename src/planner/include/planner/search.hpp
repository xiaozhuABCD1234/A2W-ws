#pragma once

#include "planner/grid.hpp"

#include <cstddef>
#include <functional>
#include <vector>

namespace planner {

struct SearchOptions {
  bool angle_search = true;
  double cost_weight = 1.0;
  int max_expansions = 300000;
  double sample_step = 0.0;
  double time_budget_ms = 500.0;
  double region_margin = 15.0;
};

struct SearchResult {
  bool success = false;
  bool timed_out = false;
  double cost = 0.0;
  size_t expanded = 0;
  std::vector<Point2d> path;
};

using FreeFn = std::function<bool(double, double, double)>;

double octile(double dx, double dy);

bool lineOfSightCells(const GridMap& grid, const Point2d& from, const Point2d& to);

bool lineOfSight(const GridMap& grid, const Point2d& from, const Point2d& to, const FreeFn& is_free,
                 double step);

SearchResult planAStar(const GridMap& grid, const Point2d& start, const Point2d& goal,
                       const SearchOptions& options, const FreeFn& is_free);

SearchResult planThetaStar(const GridMap& grid, const Point2d& start, const Point2d& goal,
                           const SearchOptions& options, const FreeFn& is_free);

SearchResult plan(const GridMap& grid, const Point2d& start, const Point2d& goal,
                  const SearchOptions& options, const FreeFn& is_free);

}
