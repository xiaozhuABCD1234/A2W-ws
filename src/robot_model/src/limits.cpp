#include "robot_model/limits.hpp"

#include <algorithm>
#include <cmath>

namespace robot_model {
namespace {

double clamp(double value, double limit) {
  if (!std::isfinite(value)) return 0.0;
  const double bound = std::abs(limit);
  if (!std::isfinite(bound)) return value;
  return std::min(bound, std::max(-bound, value));
}

double rateLimit(double current, double target, double max_delta) {
  if (!std::isfinite(current)) current = 0.0;
  if (!std::isfinite(target)) target = current;
  const double delta = std::isfinite(max_delta) ? std::max(0.0, max_delta) : 0.0;
  return current + std::min(delta, std::max(-delta, target - current));
}

}  // namespace

Twist2D applyLimits(const Twist2D& current, const Twist2D& desired, double dt,
                    const VelocityLimits& limits) {
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return {std::isfinite(current.linear) ? current.linear : 0.0,
            std::isfinite(current.angular) ? current.angular : 0.0};
  }
  Twist2D result;
  const double target_linear = clamp(desired.linear, limits.max_linear_velocity);
  const double target_angular = clamp(desired.angular, limits.max_angular_velocity);
  result.linear = rateLimit(current.linear, target_linear, limits.max_linear_acceleration * dt);
  result.angular =
      rateLimit(current.angular, target_angular, limits.max_angular_acceleration * dt);
  // A stale/oversized current command must not bypass the velocity limits.
  result.linear = clamp(result.linear, limits.max_linear_velocity);
  result.angular = clamp(result.angular, limits.max_angular_velocity);
  return result;
}

}
