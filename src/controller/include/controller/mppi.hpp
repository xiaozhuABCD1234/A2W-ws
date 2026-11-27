#pragma once

#include <random>
#include <vector>

#include "controller/grid.hpp"
#include "controller/path.hpp"
#include "controller/types.hpp"

namespace controller {

struct MppiParams {
  int batch = 400;
  int time_steps = 40;
  double model_dt = 0.05;
  int iterations = 1;
  double temperature = 0.30;
  double gamma = 0.015;
  double sigma_linear = 0.20;
  double sigma_angular = 0.35;
  int shift_steps = 1;
  double w_path = 1.0;
  double w_goal = 1.6;
  double w_obstacle = 0.9;
  double w_heading = 0.35;
  double w_prefer_forward = 0.05;
  double w_constraint = 3.0;
  double collision_penalty = 1.0e5;
  double safe_distance = 0.40;
  double critical_clearance = 0.25;
  double near_goal_distance = 0.60;
  int path_step = 3;
  int path_window = 40;
  double max_linear = 0.80;
  double min_linear = 0.0;
  double max_angular = 1.20;
};

struct MppiOutput {
  Command command;
  std::vector<Point> best_trajectory;
  double min_clearance = 0.0;
  bool feasible = false;
  int colliding_candidates = 0;
  bool have_field = false;
};

std::vector<double> softmaxWeights(const std::vector<double>& costs, double temperature);

class Mppi {
 public:
  void configure(const MppiParams& params);

  void reset();

  void setPath(const Path2D& path);

  void warmStart(const Command& command);

  const Path2D& path() const;

  MppiOutput optimize(const Pose2D& pose, const DistanceField& field, const Point& goal);

  const std::vector<double>& controlLinear() const;

  const std::vector<double>& controlAngular() const;

 private:
  void shiftControl();

  MppiParams params_;
  Path2D path_;
  std::vector<double> control_linear_;
  std::vector<double> control_angular_;
  std::vector<double> noise_linear_;
  std::vector<double> noise_angular_;
  std::vector<double> costs_;
  std::mt19937 generator_{std::mt19937::default_seed};
};

}
