#include "esdf/edt.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace esdf {

namespace {

constexpr double kInf = 1e20;

void transform1D(const double* f, int n, double* d, int* v, double* z) {
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;
  for (int q = 1; q < n; ++q) {
    double s = (f[q] + static_cast<double>(q) * q - f[v[k]] -
                static_cast<double>(v[k]) * v[k]) /
               (2.0 * static_cast<double>(q) - 2.0 * static_cast<double>(v[k]));
    while (s <= z[k]) {
      --k;
      s = (f[q] + static_cast<double>(q) * q - f[v[k]] - static_cast<double>(v[k]) * v[k]) /
          (2.0 * static_cast<double>(q) - 2.0 * static_cast<double>(v[k]));
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<double>(q)) ++k;
    const double diff = static_cast<double>(q) - static_cast<double>(v[k]);
    d[q] = diff * diff + f[v[k]];
  }
}

}

void distanceTransform(const std::vector<int8_t>& cells, int width, int height,
                       const EdtParams& params, std::vector<float>* distance) {
  if (distance == nullptr) return;
  if (width <= 0 || height <= 0) {
    distance->clear();
    return;
  }
  const double resolution = std::max(1e-6, params.resolution);
  const double max_distance = std::max(0.0, params.max_distance);
  const size_t size = static_cast<size_t>(width) * static_cast<size_t>(height);
  distance->assign(size, static_cast<float>(max_distance));

  const int threshold = params.occupied_threshold;
  int min_x = width;
  int min_y = height;
  int max_x = -1;
  int max_y = -1;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t p = static_cast<size_t>(y) * static_cast<size_t>(width) +
                       static_cast<size_t>(x);
      const int value = p < cells.size() ? cells[p] : 0;
      const bool obstacle = value >= threshold || (value < 0 && params.unknown_is_obstacle);
      if (!obstacle) continue;
      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
    }
  }
  if (max_x < 0) return;

  const int margin = static_cast<int>(std::ceil(max_distance / resolution)) + 1;
  const int x0 = std::max(0, min_x - margin);
  const int y0 = std::max(0, min_y - margin);
  const int x1 = std::min(width - 1, max_x + margin);
  const int y1 = std::min(height - 1, max_y + margin);
  const int sub_width = x1 - x0 + 1;
  const int sub_height = y1 - y0 + 1;
  const size_t sub_size = static_cast<size_t>(sub_width) * static_cast<size_t>(sub_height);

  std::vector<double> grid(sub_size, kInf);
  for (int y = 0; y < sub_height; ++y) {
    for (int x = 0; x < sub_width; ++x) {
      const size_t source = static_cast<size_t>(y0 + y) * static_cast<size_t>(width) +
                            static_cast<size_t>(x0 + x);
      const int value = source < cells.size() ? cells[source] : 0;
      if (value >= threshold || (value < 0 && params.unknown_is_obstacle)) {
        grid[static_cast<size_t>(y) * static_cast<size_t>(sub_width) + static_cast<size_t>(x)] =
            0.0;
      }
    }
  }

  const int longest = std::max(sub_width, sub_height);
  std::vector<double> f(static_cast<size_t>(longest), 0.0);
  std::vector<double> d(static_cast<size_t>(longest), 0.0);
  std::vector<int> v(static_cast<size_t>(longest) + 1, 0);
  std::vector<double> z(static_cast<size_t>(longest) + 1, 0.0);

  for (int x = 0; x < sub_width; ++x) {
    for (int y = 0; y < sub_height; ++y) {
      f[static_cast<size_t>(y)] =
          grid[static_cast<size_t>(y) * static_cast<size_t>(sub_width) + static_cast<size_t>(x)];
    }
    transform1D(f.data(), sub_height, d.data(), v.data(), z.data());
    for (int y = 0; y < sub_height; ++y) {
      grid[static_cast<size_t>(y) * static_cast<size_t>(sub_width) + static_cast<size_t>(x)] =
          d[static_cast<size_t>(y)];
    }
  }

  for (int y = 0; y < sub_height; ++y) {
    const size_t row = static_cast<size_t>(y) * static_cast<size_t>(sub_width);
    for (int x = 0; x < sub_width; ++x) {
      f[static_cast<size_t>(x)] = grid[row + static_cast<size_t>(x)];
    }
    transform1D(f.data(), sub_width, d.data(), v.data(), z.data());
    for (int x = 0; x < sub_width; ++x) {
      const double squared = d[static_cast<size_t>(x)];
      const double metres =
          squared >= kInf * 0.5 ? max_distance : std::sqrt(squared) * resolution;
      (*distance)[static_cast<size_t>(y0 + y) * static_cast<size_t>(width) +
                  static_cast<size_t>(x0 + x)] =
          static_cast<float>(std::min(max_distance, metres));
    }
  }
}

void gradient(const std::vector<float>& distance, int width, int height, double resolution,
              double max_distance, std::vector<float>* gradient_x,
              std::vector<float>* gradient_y) {
  if (gradient_x == nullptr || gradient_y == nullptr) return;
  if (width <= 0 || height <= 0 || distance.size() <
      static_cast<size_t>(width) * static_cast<size_t>(height)) {
    gradient_x->clear();
    gradient_y->clear();
    return;
  }
  const size_t size = static_cast<size_t>(width) * static_cast<size_t>(height);
  gradient_x->assign(size, 0.0f);
  gradient_y->assign(size, 0.0f);
  if (width <= 2 || height <= 2 || !(resolution > 0.0)) return;
  const float scale = static_cast<float>(0.5 / resolution);
  for (int y = 1; y + 1 < height; ++y) {
    for (int x = 1; x + 1 < width; ++x) {
      const size_t p = static_cast<size_t>(y) * static_cast<size_t>(width) +
                       static_cast<size_t>(x);
      const float left = distance[p - 1];
      const float right = distance[p + 1];
      const float down = distance[p - static_cast<size_t>(width)];
      const float up = distance[p + static_cast<size_t>(width)];
      if (left >= max_distance && right >= max_distance) continue;
      if (down >= max_distance && up >= max_distance) continue;
      (*gradient_x)[p] = (right - left) * scale;
      (*gradient_y)[p] = (up - down) * scale;
    }
  }
}

float bilinear(const std::vector<float>& values, int width, int height, double x, double y,
               float fallback) {
  if (width <= 0 || height <= 0 ||
      values.size() < static_cast<size_t>(width) * static_cast<size_t>(height)) {
    return fallback;
  }
  const double clamped_x = std::min(std::max(x, 0.0), static_cast<double>(width - 1));
  const double clamped_y = std::min(std::max(y, 0.0), static_cast<double>(height - 1));
  const int x0 = static_cast<int>(std::floor(clamped_x));
  const int y0 = static_cast<int>(std::floor(clamped_y));
  const int x1 = std::min(x0 + 1, width - 1);
  const int y1 = std::min(y0 + 1, height - 1);
  const double fx = clamped_x - static_cast<double>(x0);
  const double fy = clamped_y - static_cast<double>(y0);
  const auto at = [&](int cx, int cy) {
    return static_cast<double>(values[static_cast<size_t>(cy) * static_cast<size_t>(width) +
                                      static_cast<size_t>(cx)]);
  };
  const double top = at(x0, y0) * (1.0 - fx) + at(x1, y0) * fx;
  const double bottom = at(x0, y1) * (1.0 - fx) + at(x1, y1) * fx;
  return static_cast<float>(top * (1.0 - fy) + bottom * fy);
}

}
