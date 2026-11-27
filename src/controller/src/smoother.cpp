#include "controller/smoother.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace controller {

namespace {

double clampStep(double value, double limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

}

bool smoothPath(const std::vector<Point>& input, const DistanceField& field,
                const SmootherParams& params, std::vector<Point>* output,
                SmootherStats* stats) {
  if (output == nullptr) return false;
  SmootherStats local_stats;
  local_stats.used_field = !field.empty();
  const std::size_t count = input.size();
  if (count < 3) {
    *output = input;
    if (stats != nullptr) *stats = local_stats;
    return false;
  }

  const auto start = std::chrono::steady_clock::now();
  const double step_limit = std::max(1e-4, params.max_step);
  const double budget = std::max(0.0, params.time_budget_ms);
  const double projection_margin = field.empty() ? 0.0 : 0.5 * field.resolution();
  const double repulsion_distance =
      params.min_clearance + std::max(0.0, params.obstacle_margin);

  std::vector<Point> work = input;
  const bool use_field = !field.empty();
  const int refinement = std::max(1, params.refinement);

  for (int pass = 0; pass < refinement; ++pass) {
    bool converged = false;
    for (int iteration = 0; iteration < std::max(1, params.iterations); ++iteration) {
      double change = 0.0;
      for (std::size_t i = 1; i + 1 < count; ++i) {
        double gradient_x = 0.0;
        double gradient_y = 0.0;
        double push_x = 0.0;
        double push_y = 0.0;
        if (use_field) {
          const double clearance = field.distance(work[i].x, work[i].y);
          if (clearance < repulsion_distance &&
              field.gradient(work[i].x, work[i].y, &gradient_x, &gradient_y)) {
            const double norm = std::hypot(gradient_x, gradient_y);
            if (norm > 1e-6) {
              const double push = repulsion_distance - clearance;
              push_x = push * gradient_x / norm;
              push_y = push * gradient_y / norm;
            }
          }
        }

        const double data_x = input[i].x - work[i].x;
        const double data_y = input[i].y - work[i].y;
        const double smooth_x = work[i - 1].x + work[i + 1].x - 2.0 * work[i].x;
        const double smooth_y = work[i - 1].y + work[i + 1].y - 2.0 * work[i].y;

        double delta_x = params.alpha * (params.w_data * data_x + params.w_smooth * smooth_x +
                                         params.w_obstacle * push_x);
        double delta_y = params.alpha * (params.w_data * data_y + params.w_smooth * smooth_y +
                                         params.w_obstacle * push_y);
        delta_x = clampStep(delta_x, step_limit);
        delta_y = clampStep(delta_y, step_limit);

        double candidate_x = work[i].x + delta_x;
        double candidate_y = work[i].y + delta_y;

        if (use_field) {
          double clearance = field.distance(candidate_x, candidate_y);
          if (clearance < params.min_clearance &&
              field.gradient(candidate_x, candidate_y, &gradient_x, &gradient_y)) {
            const double norm = std::hypot(gradient_x, gradient_y);
            if (norm > 1e-6) {
              const double push = params.min_clearance - clearance + projection_margin;
              candidate_x += push * gradient_x / norm;
              candidate_y += push * gradient_y / norm;
            }
          }
          clearance = field.distance(candidate_x, candidate_y);
          if (clearance < 0.5 * params.min_clearance) {
            candidate_x = work[i].x;
            candidate_y = work[i].y;
            local_stats.limited_by_collision = true;
          }
        }

        change += std::hypot(candidate_x - work[i].x, candidate_y - work[i].y);
        work[i].x = candidate_x;
        work[i].y = candidate_y;
      }

      ++local_stats.iterations;
      if (change < params.tolerance) {
        converged = true;
        break;
      }
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - start)
                                    .count();
      if (elapsed_ms >= budget) {
        local_stats.limited_by_time = true;
        break;
      }
    }
    if (local_stats.limited_by_time) break;
    if (converged) continue;
  }

  double max_deviation = 0.0;
  double min_clearance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < count; ++i) {
    max_deviation = std::max(max_deviation, pointDistance(work[i], input[i]));
    if (use_field && i > 0 && i + 1 < count) {
      min_clearance = std::min(min_clearance, field.distance(work[i].x, work[i].y));
    }
  }
  local_stats.max_deviation = max_deviation;
  local_stats.min_clearance = use_field ? min_clearance : 0.0;

  *output = std::move(work);
  if (stats != nullptr) *stats = local_stats;
  return true;
}

}
