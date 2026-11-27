#pragma once

#include <cstddef>
#include <vector>

#include "controller/types.hpp"

namespace controller {

struct Path2D {
  std::vector<Point> points;
  std::vector<double> yaw;
  std::vector<double> arc;
};

void resamplePath(const std::vector<Point>& input, double step, std::vector<Point>* output);

Path2D makePath(const std::vector<Point>& points);

std::size_t nearestIndex(const Path2D& path, const Point& query, std::size_t hint,
                         std::size_t window);

std::size_t advanceIndex(const Path2D& path, const Point& query, std::size_t start,
                         std::size_t window);

std::size_t indexAtDistance(const Path2D& path, double distance);

double remainingDistance(const Path2D& path, std::size_t index);

}
