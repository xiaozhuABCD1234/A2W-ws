#include "controller/mppi.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace controller {

std::vector<double> softmaxWeights(const std::vector<double>& costs, double temperature) {
  std::vector<double> weights(costs.size(), 0.0);
  if (costs.empty()) return weights;
  const double temp = std::max(1e-6, temperature);
  const double minimum = *std::min_element(costs.begin(), costs.end());
  double sum = 0.0;
  for (std::size_t i = 0; i < costs.size(); ++i) {
    const double scaled = -(costs[i] - minimum) / temp;
    weights[i] = scaled < -700.0 ? 0.0 : std::exp(scaled);
    sum += weights[i];
  }
  if (!(sum > 0.0)) {
    const double uniform = 1.0 / static_cast<double>(costs.size());
    std::fill(weights.begin(), weights.end(), uniform);
    return weights;
  }
  for (double& weight : weights) weight /= sum;
  return weights;
}

void Mppi::configure(const MppiParams& params) {
  params_ = params;
  params_.batch = std::max(1, params_.batch);
  params_.time_steps = std::max(2, params_.time_steps);
  params_.iterations = std::max(1, params_.iterations);
  params_.model_dt = std::max(1e-3, params_.model_dt);
  params_.temperature = std::max(1e-4, params_.temperature);
  params_.sigma_linear = std::max(1e-3, params_.sigma_linear);
  params_.sigma_angular = std::max(1e-3, params_.sigma_angular);
  params_.path_step = std::max(1, params_.path_step);
  params_.path_window = std::max(1, params_.path_window);
  params_.shift_steps = std::max(0, params_.shift_steps);
  params_.max_linear = std::max(0.0, params_.max_linear);
  params_.min_linear = std::clamp(params_.min_linear, -params_.max_linear, params_.max_linear);
  params_.max_angular = std::max(1e-3, params_.max_angular);
  params_.critical_clearance = std::max(0.0, params_.critical_clearance);
  params_.safe_distance = std::max(params_.critical_clearance, params_.safe_distance);

  const std::size_t steps = static_cast<std::size_t>(params_.time_steps);
  const std::size_t samples = static_cast<std::size_t>(params_.batch) * steps;
  control_linear_.assign(steps, 0.0);
  control_angular_.assign(steps, 0.0);
  noise_linear_.assign(samples, 0.0);
  noise_angular_.assign(samples, 0.0);
  costs_.assign(static_cast<std::size_t>(params_.batch), 0.0);
  reset();
}

void Mppi::reset() {
  std::fill(control_linear_.begin(), control_linear_.end(), 0.0);
  std::fill(control_angular_.begin(), control_angular_.end(), 0.0);
}

void Mppi::setPath(const Path2D& path) {
  // Accept both fully built Path2D objects and bare point lists.
  if (path.points.size() >= 2 &&
      (path.yaw.size() != path.points.size() || path.arc.size() != path.points.size())) {
    path_ = makePath(path.points);
  } else {
    path_ = path;
  }
}

const Path2D& Mppi::path() const { return path_; }

void Mppi::warmStart(const Command& command) {
  const double linear = std::clamp(command.linear, params_.min_linear, params_.max_linear);
  const double angular = std::clamp(command.angular, -params_.max_angular, params_.max_angular);
  std::fill(control_linear_.begin(), control_linear_.end(), linear);
  std::fill(control_angular_.begin(), control_angular_.end(), angular);
}

const std::vector<double>& Mppi::controlLinear() const { return control_linear_; }

const std::vector<double>& Mppi::controlAngular() const { return control_angular_; }

void Mppi::shiftControl() {
  const int steps = params_.time_steps;
  const int shifts = std::min(params_.shift_steps, std::max(0, steps - 1));
  for (int s = 0; s < shifts; ++s) {
    for (int t = 0; t + 1 < steps; ++t) {
      control_linear_[t] = control_linear_[t + 1];
      control_angular_[t] = control_angular_[t + 1];
    }
    control_linear_[steps - 1] = control_linear_[steps - 2];
    control_angular_[steps - 1] = control_angular_[steps - 2];
  }
}

MppiOutput Mppi::optimize(const Pose2D& pose, const DistanceField& field, const Point& goal) {
  MppiOutput output;
  const int batch = params_.batch;
  const int steps = params_.time_steps;
  const double dt = params_.model_dt;
  if (path_.points.size() < 2) return output;

  const bool have_field = !field.empty();
  output.have_field = have_field;
  const Path2D& path = path_;
  const std::size_t stride = static_cast<std::size_t>(params_.path_step);
  const std::size_t window = static_cast<std::size_t>(params_.path_window);
  const double goal_distance = std::hypot(pose.x - goal.x, pose.y - goal.y);
  const bool near_goal = goal_distance < params_.near_goal_distance;
  const double path_weight = near_goal ? 0.0 : params_.w_path;
  const double heading_weight = near_goal ? 0.0 : params_.w_heading;
  const double inverse_sigma_linear = 1.0 / (params_.sigma_linear * params_.sigma_linear);
  const double inverse_sigma_angular = 1.0 / (params_.sigma_angular * params_.sigma_angular);

  std::normal_distribution<double> sample_linear(0.0, params_.sigma_linear);
  std::normal_distribution<double> sample_angular(0.0, params_.sigma_angular);

  const std::size_t start_index = nearestIndex(path, {pose.x, pose.y}, 0, 0);
  int colliding = 0;

  for (int iteration = 0; iteration < params_.iterations; ++iteration) {
    for (std::size_t i = 0; i < noise_linear_.size(); ++i) {
      noise_linear_[i] = sample_linear(generator_);
      noise_angular_[i] = sample_angular(generator_);
    }

    std::fill(costs_.begin(), costs_.end(), 0.0);
    colliding = 0;

    for (int k = 0; k < batch; ++k) {
      const std::size_t offset = static_cast<std::size_t>(k) * static_cast<std::size_t>(steps);
      double x = pose.x;
      double y = pose.y;
      double yaw = pose.yaw;
      double path_cost = 0.0;
      double heading_cost = 0.0;
      double goal_cost = 0.0;
      double obstacle_cost = 0.0;
      double constraint_cost = 0.0;
      double forward_cost = 0.0;
      std::size_t path_index = start_index;
      std::size_t evaluated = 0;
      bool collide = false;

      for (int t = 0; t < steps; ++t) {
        const std::size_t sample = offset + static_cast<std::size_t>(t);
        const double linear = control_linear_[t] + noise_linear_[sample];
        const double angular = control_angular_[t] + noise_angular_[sample];

        x += linear * std::cos(yaw) * dt;
        y += linear * std::sin(yaw) * dt;
        yaw += angular * dt;

        const std::size_t step_index = static_cast<std::size_t>(t);
        if (step_index % stride == 0 || t == steps - 1) {
          path_index = advanceIndex(path, {x, y}, path_index, window);
          const Point& reference = path.points[path_index];
          path_cost += std::hypot(x - reference.x, y - reference.y);
          heading_cost += std::fabs(normalizeAngle(yaw - path.yaw[path_index]));
          ++evaluated;
        }

        goal_cost += std::hypot(x - goal.x, y - goal.y);
        forward_cost += std::max(0.0, params_.max_linear - std::fabs(linear)) * dt;
        constraint_cost += (std::max(0.0, linear - params_.max_linear) +
                            std::max(0.0, params_.min_linear - linear) +
                            std::max(0.0, std::fabs(angular) - params_.max_angular)) *
                           dt;

        if (have_field) {
          const double clearance = field.distance(x, y);
          if (clearance <= params_.critical_clearance) {
            collide = true;
            break;
          }
          const double penetration = params_.safe_distance - clearance;
          if (penetration > 0.0) obstacle_cost += penetration * penetration;
        }
      }

      const double samples = static_cast<double>(std::max<std::size_t>(1, evaluated));
      double cost = path_weight * path_cost / samples;
      cost += heading_weight * heading_cost / samples;
      cost += params_.w_goal * goal_cost / static_cast<double>(steps);
      cost += params_.w_obstacle * obstacle_cost / static_cast<double>(steps);
      cost += params_.w_constraint * constraint_cost;
      cost += params_.w_prefer_forward * forward_cost;

      double correction = 0.0;
      for (int t = 0; t < steps; ++t) {
        const std::size_t index = offset + static_cast<std::size_t>(t);
        correction += control_linear_[t] * noise_linear_[index] * inverse_sigma_linear;
        correction += control_angular_[t] * noise_angular_[index] * inverse_sigma_angular;
      }
      cost += params_.gamma * correction;

      if (collide) {
        cost += params_.collision_penalty;
        ++colliding;
      }
      costs_[static_cast<std::size_t>(k)] = cost;
    }

    const std::vector<double> weights = softmaxWeights(costs_, params_.temperature);
    for (int t = 0; t < steps; ++t) {
      double linear = 0.0;
      double angular = 0.0;
      for (int k = 0; k < batch; ++k) {
        const std::size_t index = static_cast<std::size_t>(k) * static_cast<std::size_t>(steps) +
                                  static_cast<std::size_t>(t);
        const double weight = weights[static_cast<std::size_t>(k)];
        linear += weight * (control_linear_[t] + noise_linear_[index]);
        angular += weight * (control_angular_[t] + noise_angular_[index]);
      }
      control_linear_[t] = std::clamp(linear, params_.min_linear, params_.max_linear);
      control_angular_[t] = std::clamp(angular, -params_.max_angular, params_.max_angular);
    }
  }

  output.best_trajectory.clear();
  output.best_trajectory.reserve(static_cast<std::size_t>(steps));
  double x = pose.x;
  double y = pose.y;
  double yaw = pose.yaw;
  double min_clearance = std::numeric_limits<double>::infinity();
  bool collide = false;
  for (int t = 0; t < steps; ++t) {
    x += control_linear_[t] * std::cos(yaw) * dt;
    y += control_linear_[t] * std::sin(yaw) * dt;
    yaw += control_angular_[t] * dt;
    Point point;
    point.x = x;
    point.y = y;
    output.best_trajectory.push_back(point);
    if (have_field) {
      const double clearance = field.distance(x, y);
      min_clearance = std::min(min_clearance, clearance);
      if (clearance <= params_.critical_clearance) collide = true;
    }
  }

  output.min_clearance = have_field ? min_clearance : 0.0;
  output.colliding_candidates = colliding;
  const bool all_collide = have_field && colliding >= batch;
  output.feasible = !collide && !all_collide;
  output.command.linear = control_linear_.front();
  output.command.angular = control_angular_.front();

  shiftControl();
  return output;
}

}
