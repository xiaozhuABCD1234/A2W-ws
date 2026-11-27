#include "controller/path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace controller {

void resamplePath(const std::vector<Point>& input, double step, std::vector<Point>* output) {
  if (output == nullptr) return;
  output->clear();
  if (input.empty()) return;
  if (input.size() == 1 || step <= 1e-6) {
    *output = input;
    return;
  }

  std::vector<double> arc(input.size(), 0.0);
  for (std::size_t i = 1; i < input.size(); ++i) {
    arc[i] = arc[i - 1] + pointDistance(input[i - 1], input[i]);
  }
  const double total = arc.back();
  output->push_back(input.front());
  if (total <= 1e-9) return;

  std::size_t segment = 1;
  for (double s = step; s < total; s += step) {
    while (segment + 1 < input.size() && arc[segment] < s) ++segment;
    const double span = arc[segment] - arc[segment - 1];
    const double ratio = span > 1e-9 ? (s - arc[segment - 1]) / span : 0.0;
    Point point;
    point.x = input[segment - 1].x + ratio * (input[segment].x - input[segment - 1].x);
    point.y = input[segment - 1].y + ratio * (input[segment].y - input[segment - 1].y);
    if (pointDistance(point, output->back()) > 1e-6) output->push_back(point);
  }
  if (pointDistance(input.back(), output->back()) > 1e-6) output->push_back(input.back());
}

Path2D makePath(const std::vector<Point>& points) {
  Path2D path;
  path.points = points;
  const std::size_t count = points.size();
  path.yaw.assign(count, 0.0);
  path.arc.assign(count, 0.0);
  if (count < 2) return path;

  for (std::size_t i = 1; i < count; ++i) {
    path.arc[i] = path.arc[i - 1] + pointDistance(points[i - 1], points[i]);
  }
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t low = i == 0 ? 0 : i - 1;
    const std::size_t high = i + 1 < count ? i + 1 : count - 1;
    path.yaw[i] = std::atan2(points[high].y - points[low].y, points[high].x - points[low].x);
  }
  return path;
}

std::size_t nearestIndex(const Path2D& path, const Point& query, std::size_t hint,
                         std::size_t window) {
  const std::size_t count = path.points.size();
  if (count == 0) return 0;
  std::size_t begin = 0;
  std::size_t end = count;
  if (window > 0) {
    // Clamp the hint so a stale index can never produce an empty/invalid range.
    const std::size_t center = std::min(hint, count - 1);
    begin = center > window ? center - window : 0;
    end = std::min(count, center + window + 1);
  }
  std::size_t best = begin;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = begin; i < end; ++i) {
    const double d = pointDistance(path.points[i], query);
    if (d < best_distance) {
      best_distance = d;
      best = i;
    }
  }
  return best;
}

std::size_t advanceIndex(const Path2D& path, const Point& query, std::size_t start,
                         std::size_t window) {
  const std::size_t count = path.points.size();
  if (count == 0) return 0;
  if (start >= count) start = count - 1;
  const std::size_t end = std::min(count, start + window + 1);
  std::size_t best = start;
  double best_distance = pointDistance(path.points[start], query);
  for (std::size_t i = start + 1; i < end; ++i) {
    const double d = pointDistance(path.points[i], query);
    if (d < best_distance) {
      best_distance = d;
      best = i;
    }
  }
  return best;
}

std::size_t indexAtDistance(const Path2D& path, double distance) {
  const std::size_t count = path.points.size();
  if (count == 0 || path.arc.size() != count) return 0;
  if (distance <= 0.0) return 0;
  if (distance >= path.arc.back()) return count - 1;
  const auto iterator = std::lower_bound(path.arc.begin(), path.arc.end(), distance);
  const std::size_t index = static_cast<std::size_t>(iterator - path.arc.begin());
  return index < count ? index : count - 1;
}

double remainingDistance(const Path2D& path, std::size_t index) {
  if (path.points.size() < 2 || path.arc.size() != path.points.size()) return 0.0;
  if (index >= path.points.size()) index = path.points.size() - 1;
  return path.arc.back() - path.arc[index];
}

}
