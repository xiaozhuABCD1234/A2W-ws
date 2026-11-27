#pragma once

#include <cstddef>

#include "controller/path.hpp"
#include "controller/types.hpp"

namespace controller {

struct PurePursuitParams {
  double desired_linear = 0.60;
  double lookahead_time = 1.10;
  double min_lookahead = 0.35;
  double max_lookahead = 1.10;
  double min_radius = 0.35;
  double approach_distance = 0.70;
  double min_approach_speed = 0.06;
  double rotate_angle = 1.20;
  double rotate_angular = 0.90;
  double goal_tolerance = 0.20;
  double min_turning_linear = 0.05;
};

struct PurePursuitOutput {
  Command command;
  Point carrot;
  std::size_t carrot_index = 0;
  double lookahead = 0.0;
  double curvature = 0.0;
  double path_distance = 0.0;
  bool rotating = false;
  bool reached = false;
};

PurePursuitOutput purePursuit(const Path2D& path, const Pose2D& pose, double speed_reference,
                              double speed_scale, const PurePursuitParams& params);

}
