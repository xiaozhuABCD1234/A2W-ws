#include "planner/search.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>

namespace planner {
namespace {

constexpr double kDiagonalCost = 1.4142135623730951;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct Cell {
  int x = 0;
  int y = 0;
};

struct NodeRecord {
  double g = kInfinity;
  double f = kInfinity;
  int parent = -1;
  bool closed = false;
};

struct QueueEntry {
  double f = 0.0;
  double g = 0.0;
  int index = -1;
};

struct QueueCompare {
  bool operator()(const QueueEntry& a, const QueueEntry& b) const { return a.f > b.f; }
};

int neighbourDx(int i) {
  static const int table[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  return table[i];
}

int neighbourDy(int i) {
  static const int table[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  return table[i];
}

double stepCost(int dx, int dy) {
  return (dx != 0 && dy != 0) ? kDiagonalCost : 1.0;
}

double traversalCost(const GridMap& grid, int cx, int cy) {
  const float value = grid.costAt(cx, cy);
  return value > 0.0f ? static_cast<double>(value) : 1.0;
}

bool cornerBlocked(const GridMap& grid, int cx, int cy, int dx, int dy) {
  return grid.blockedAt(cx + dx, cy) && grid.blockedAt(cx, cy + dy);
}

std::vector<Point2d> tracePath(const GridMap& grid, const std::vector<NodeRecord>& records,
                               const std::vector<Cell>& cells, int start_index, int goal_index) {
  std::vector<Point2d> path;
  int current = goal_index;
  int guard = 0;
  while (current >= 0 && guard++ < 100000) {
    Point2d point;
    grid.cellToWorld(cells[current].x, cells[current].y, &point.x, &point.y);
    path.push_back(point);
    if (current == start_index) break;
    current = records[current].parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

bool segmentFree(const GridMap& grid, const Point2d& from, const Point2d& to, const FreeFn& is_free,
                 double step) {
  // An empty callback means "only use the grid"; calling it would throw.
  if (!is_free) return true;
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double length = std::hypot(dx, dy);
  const double spacing = step > 0.0 ? step : 0.5 * grid.resolution;
  const double yaw = std::atan2(dy, dx);
  if (!is_free(from.x, from.y, yaw)) return false;
  const int samples = static_cast<int>(std::ceil(length / spacing));
  for (int i = 1; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    if (!is_free(from.x + t * dx, from.y + t * dy, yaw)) return false;
  }
  return true;
}

}

double octile(double dx, double dy) {
  const double ax = std::abs(dx);
  const double ay = std::abs(dy);
  return (ax > ay) ? (ax - ay) + kDiagonalCost * ay : (ay - ax) + kDiagonalCost * ax;
}

bool lineOfSightCells(const GridMap& grid, const Point2d& from, const Point2d& to) {
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  if (!grid.worldToCell(from.x, from.y, &x0, &y0)) return false;
  if (!grid.worldToCell(to.x, to.y, &x1, &y1)) return false;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx - dy;
  while (true) {
    if (grid.blockedAt(x0, y0)) return false;
    if (x0 == x1 && y0 == y1) return true;
    const int doubled = 2 * error;
    if (doubled > -dy) {
      error -= dy;
      x0 += sx;
    }
    if (doubled < dx) {
      error += dx;
      y0 += sy;
    }
  }
}

bool lineOfSight(const GridMap& grid, const Point2d& from, const Point2d& to, const FreeFn& is_free,
                 double step) {
  return lineOfSightCells(grid, from, to) && segmentFree(grid, from, to, is_free, step);
}

namespace {

SearchResult search(const GridMap& grid, const Point2d& start, const Point2d& goal,
                    const SearchOptions& options, const FreeFn& is_free, bool angle_search) {
  SearchResult result;
  if (grid.empty()) return result;

  int sx = 0;
  int sy = 0;
  int gx = 0;
  int gy = 0;
  if (!grid.worldToCell(start.x, start.y, &sx, &sy)) return result;
  if (!grid.worldToCell(goal.x, goal.y, &gx, &gy)) return result;

  const size_t total = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
  std::vector<Cell> cells(total);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      cells[grid.index(x, y)] = {x, y};
    }
  }

  const size_t start_index = grid.index(sx, sy);
  const size_t goal_index = grid.index(gx, gy);
  std::vector<NodeRecord> records(total);
  records[start_index].g = 0.0;
  records[start_index].f = octile(gx - sx, gy - sy);

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open;
  open.push({records[start_index].f, 0.0, static_cast<int>(start_index)});

  const double spacing = options.sample_step > 0.0 ? options.sample_step : grid.resolution;
  const int region = options.region_margin > 0.0
                         ? static_cast<int>(std::ceil(options.region_margin / grid.resolution))
                         : std::max(grid.width, grid.height);
  const int min_x = std::max(0, std::min(sx, gx) - region);
  const int min_y = std::max(0, std::min(sy, gy) - region);
  const int max_x = std::min(grid.width - 1, std::max(sx, gx) + region);
  const int max_y = std::min(grid.height - 1, std::max(sy, gy) + region);
  const auto outside = [&](int cx, int cy) {
    return cx < min_x || cy < min_y || cx > max_x || cy > max_y;
  };
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(static_cast<int>(options.time_budget_ms));
  bool found = false;

  while (!open.empty()) {
    const QueueEntry entry = open.top();
    open.pop();
    const size_t current_index = static_cast<size_t>(entry.index);
    NodeRecord& current = records[current_index];
    if (current.closed) continue;
    if (entry.g > current.g + 1e-9) continue;
    current.closed = true;
    ++result.expanded;
    if (current_index == goal_index) {
      found = true;
      break;
    }
    if (static_cast<int>(result.expanded) >= options.max_expansions) break;
    if ((result.expanded & 0x3FF) == 0 && std::chrono::steady_clock::now() > deadline) {
      result.timed_out = true;
      break;
    }

    const int cx = cells[current_index].x;
    const int cy = cells[current_index].y;
    for (int i = 0; i < 8; ++i) {
      const int dx = neighbourDx(i);
      const int dy = neighbourDy(i);
      const int nx = cx + dx;
      const int ny = cy + dy;
      if (outside(nx, ny)) continue;
      if (grid.blockedAt(nx, ny)) continue;
      if (dx != 0 && dy != 0 && cornerBlocked(grid, cx, cy, dx, dy)) continue;

      const size_t next_index = grid.index(nx, ny);
      if (records[next_index].closed) continue;

      double candidate = current.g;
      int new_parent = static_cast<int>(current_index);
      if (angle_search && current.parent >= 0) {
        const Cell& parent_cell = cells[static_cast<size_t>(current.parent)];
        Point2d parent_point;
        Point2d next_point;
        grid.cellToWorld(parent_cell.x, parent_cell.y, &parent_point.x, &parent_point.y);
        grid.cellToWorld(nx, ny, &next_point.x, &next_point.y);
        if (lineOfSight(grid, parent_point, next_point, is_free, spacing)) {
          candidate = records[static_cast<size_t>(current.parent)].g +
                      octile(next_point.x - parent_point.x, next_point.y - parent_point.y) /
                          grid.resolution;
          new_parent = current.parent;
        } else {
          candidate += stepCost(dx, dy) * traversalCost(grid, nx, ny);
        }
      } else {
        candidate += stepCost(dx, dy) * traversalCost(grid, nx, ny);
      }

      NodeRecord& next = records[next_index];
      if (candidate >= next.g - 1e-9) continue;
      next.g = candidate;
      next.parent = new_parent;
      next.f = candidate * options.cost_weight + octile(gx - nx, gy - ny);
      open.push({next.f, next.g, static_cast<int>(next_index)});
    }
  }

  if (!found) return result;
  result.success = true;
  result.cost = records[goal_index].g * grid.resolution;
  result.path = tracePath(grid, records, cells, static_cast<int>(start_index),
                          static_cast<int>(goal_index));
  return result;
}

}

SearchResult planAStar(const GridMap& grid, const Point2d& start, const Point2d& goal,
                       const SearchOptions& options, const FreeFn& is_free) {
  return search(grid, start, goal, options, is_free, false);
}

SearchResult planThetaStar(const GridMap& grid, const Point2d& start, const Point2d& goal,
                           const SearchOptions& options, const FreeFn& is_free) {
  return search(grid, start, goal, options, is_free, true);
}

SearchResult plan(const GridMap& grid, const Point2d& start, const Point2d& goal,
                  const SearchOptions& options, const FreeFn& is_free) {
  if (!options.angle_search) return planAStar(grid, start, goal, options, is_free);
  SearchResult result = planThetaStar(grid, start, goal, options, is_free);
  if (!result.success) result = planAStar(grid, start, goal, options, is_free);
  return result;
}

}
