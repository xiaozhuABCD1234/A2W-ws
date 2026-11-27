#include "controller/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

namespace controller {

PurePursuitOutput purePursuit(const Path2D& path, const Pose2D& pose, double speed_reference,
                              double speed_scale, const PurePursuitParams& params) {
  PurePursuitOutput output;
  if (path.points.size() < 2) return output;

  const Point robot{pose.x, pose.y};
  const std::size_t closest = nearestIndex(path, robot, 0, 0);
  output.path_distance = pointDistance(robot, path.points[closest]);
  const double goal_distance = pointDistance(robot, path.points.back());
  output.carrot = path.points.back();
  output.carrot_index = path.points.size() - 1;

  if (goal_distance <= params.goal_tolerance) {
    output.reached = true;
    return output;
  }

  const double reference = std::max(0.0, speed_reference);
  const double lookahead = std::clamp(reference * params.lookahead_time, params.min_lookahead,
                                      std::max(params.min_lookahead, params.max_lookahead));
  output.lookahead = lookahead;

  const double target_distance = path.arc[closest] + lookahead;
  const std::size_t target_index = indexAtDistance(path, target_distance);
  output.carrot_index = target_index;
  output.carrot = path.points[target_index];

  const double dx = output.carrot.x - pose.x;
  const double dy = output.carrot.y - pose.y;
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  const double local_x = dx * cos_yaw + dy * sin_yaw;
  const double local_y = -dx * sin_yaw + dy * cos_yaw;
  const double angle = std::atan2(local_y, local_x);

  const double scale = std::clamp(speed_scale, 0.0, 1.0);
  const double base_speed = std::max(0.0, params.desired_linear);

  if (local_x <= params.min_turning_linear || std::fabs(angle) > params.rotate_angle) {
    output.rotating = true;
    output.curvature = 0.0;
    output.command.linear = 0.0;
    const double sign = angle >= 0.0 ? 1.0 : -1.0;
    double angular = sign * params.rotate_angular;
    if (std::fabs(angle) < 1.0) angular *= std::fabs(angle);
    output.command.angular = angular;
    return output;
  }

  const double squared = local_x * local_x + local_y * local_y;
  double curvature = squared > 1e-9 ? 2.0 * local_y / squared : 0.0;
  const double max_curvature = params.min_radius > 1e-3 ? 1.0 / params.min_radius : 0.0;
  if (max_curvature > 0.0) {
    curvature = std::clamp(curvature, -max_curvature, max_curvature);
  }
  output.curvature = curvature;

  double curvature_scale = 1.0;
  if (std::fabs(curvature) > 1e-6 && max_curvature > 0.0) {
    curvature_scale = std::min(1.0, max_curvature / std::fabs(curvature));
  }
  double approach_scale = 1.0;
  if (params.approach_distance > 1e-6) {
    approach_scale = std::clamp(goal_distance / params.approach_distance, 0.0, 1.0);
  }

  double speed = base_speed * scale * curvature_scale * approach_scale;
  speed = std::max(speed, params.min_approach_speed * scale);
  speed = std::min(speed, base_speed);
  output.command.linear = speed;
  output.command.angular = curvature * speed;
  return output;
}

}
