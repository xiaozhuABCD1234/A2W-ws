#pragma once

namespace robot_model {

struct VelocityLimits {
  double max_linear_velocity = 0.5;
  double max_angular_velocity = 1.0;
  double max_linear_acceleration = 1.0;
  double max_angular_acceleration = 2.0;
};

struct Twist2D {
  double linear = 0.0;
  double angular = 0.0;
};

Twist2D applyLimits(const Twist2D& current, const Twist2D& desired, double dt,
                    const VelocityLimits& limits);

}
