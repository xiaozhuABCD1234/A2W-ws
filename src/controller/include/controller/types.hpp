#pragma once

#include <cmath>

namespace controller {

struct Point {
  double x = 0.0;
  double y = 0.0;
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct Command {
  double linear = 0.0;
  double angular = 0.0;
};

inline double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

inline double pointDistance(const Point& a, const Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

}
