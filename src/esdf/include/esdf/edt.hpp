#pragma once

#include <cstdint>
#include <vector>

namespace esdf {

struct EdtParams {
  double resolution = 0.05;
  double max_distance = 5.0;
  int occupied_threshold = 65;
  bool unknown_is_obstacle = false;
};

void distanceTransform(const std::vector<int8_t>& cells, int width, int height,
                       const EdtParams& params, std::vector<float>* distance);

void gradient(const std::vector<float>& distance, int width, int height, double resolution,
              double max_distance, std::vector<float>* gradient_x,
              std::vector<float>* gradient_y);

float bilinear(const std::vector<float>& values, int width, int height, double x, double y,
               float fallback);

}
