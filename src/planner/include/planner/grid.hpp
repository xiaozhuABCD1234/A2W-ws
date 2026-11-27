#pragma once

#include <array>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <vector>

namespace planner {

struct Point2d {
  double x = 0.0;
  double y = 0.0;
};

struct GridMap {
  int width = 0;
  int height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<float> cost;
  std::vector<uint8_t> blocked;

  bool empty() const { return width <= 0 || height <= 0; }

  bool inside(int cx, int cy) const {
    return cx >= 0 && cy >= 0 && cx < width && cy < height;
  }

  size_t index(int cx, int cy) const {
    return static_cast<size_t>(cy) * static_cast<size_t>(width) + static_cast<size_t>(cx);
  }

  bool blockedAt(int cx, int cy) const {
    return !inside(cx, cy) || blocked[index(cx, cy)] != 0;
  }

  float costAt(int cx, int cy) const { return cost[index(cx, cy)]; }

  bool worldToCell(double x, double y, int* cx, int* cy) const {
    *cx = static_cast<int>(std::floor((x - origin_x) / resolution));
    *cy = static_cast<int>(std::floor((y - origin_y) / resolution));
    return inside(*cx, *cy);
  }

  void cellToWorld(int cx, int cy, double* x, double* y) const {
    *x = origin_x + (static_cast<double>(cx) + 0.5) * resolution;
    *y = origin_y + (static_cast<double>(cy) + 0.5) * resolution;
  }
};

struct EsdfField {
  int width = 0;
  int height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  float max_distance = 0.0f;
  std::vector<float> data;

  bool empty() const { return width <= 0 || height <= 0 || data.empty(); }

  float at(double x, double y) const {
    if (empty()) return max_distance;
    const int cx = static_cast<int>(std::floor((x - origin_x) / resolution));
    const int cy = static_cast<int>(std::floor((y - origin_y) / resolution));
    if (cx < 0 || cy < 0 || cx >= width || cy >= height) return max_distance;
    return data[static_cast<size_t>(cy) * static_cast<size_t>(width) + static_cast<size_t>(cx)];
  }
};

}
