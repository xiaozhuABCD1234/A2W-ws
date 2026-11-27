#pragma once

#include <vector>

#include "controller/grid.hpp"
#include "controller/types.hpp"

namespace controller {

struct SmootherParams {
  int iterations = 600;
  double tolerance = 5.0e-4;
  double w_data = 0.25;
  double w_smooth = 0.40;
  double w_obstacle = 0.60;
  double alpha = 0.60;
  double max_step = 0.05;
  double min_clearance = 0.22;
  double obstacle_margin = 0.15;
  double time_budget_ms = 100.0;
  int refinement = 1;
};

struct SmootherStats {
  int iterations = 0;
  double max_deviation = 0.0;
  double min_clearance = 0.0;
  bool limited_by_time = false;
  bool limited_by_collision = false;
  bool used_field = false;
};

bool smoothPath(const std::vector<Point>& input, const DistanceField& field,
                const SmootherParams& params, std::vector<Point>* output, SmootherStats* stats);

}
