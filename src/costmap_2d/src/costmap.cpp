#include "costmap_2d/costmap.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace costmap_2d {

namespace {
constexpr double kTwoPi = 2.0 * M_PI;

float logOddsOfProbability(double probability) {
  const double clamped = std::min(1.0 - 1e-6, std::max(1e-6, probability));
  return static_cast<float>(std::log(clamped / (1.0 - clamped)));
}
}

RollingCostmap::RollingCostmap(const CostmapParams& params) : params_(params) {
  params_.resolution = std::max(0.01, params_.resolution);
  const int width = std::max(4, static_cast<int>(std::lround(params_.width / params_.resolution)));
  const int height = std::max(4, static_cast<int>(std::lround(params_.height / params_.resolution)));
  params_.width = width * params_.resolution;
  params_.height = height * params_.resolution;
  params_.recenter_margin = std::max(0.0, params_.recenter_margin);
  params_.max_range = std::max(0.1, params_.max_range);
  params_.blind_hold_sec = std::max(0.0, params_.blind_hold_sec);
  params_.raycast_dedupe_deg = std::max(0.0, params_.raycast_dedupe_deg);
  params_.min_component_cells = std::max(1, params_.min_component_cells);

  occupied_threshold_ = logOddsOfProbability(std::min(0.99, params_.ray_block_probability * 0.7));
  ray_block_threshold_ = logOddsOfProbability(params_.ray_block_probability);
  free_ray_occupied_delta_ = static_cast<float>(params_.free_ray_occupied_delta);
  min_log_odds_ = static_cast<float>(params_.min_log_odds);
  max_log_odds_ = static_cast<float>(params_.max_log_odds);
  occupied_delta_ = static_cast<float>(params_.occupied_log_odds);
  free_delta_ = -static_cast<float>(params_.free_log_odds);
  occupied_decay_ = static_cast<float>(params_.occupied_decay);
  free_decay_ = static_cast<float>(params_.free_decay);
  max_range2_ = params_.max_range * params_.max_range;

  allocate(width, height, -0.5 * params_.width, -0.5 * params_.height);
}

void RollingCostmap::allocate(int width, int height, double origin_x, double origin_y) {
  width_ = width;
  height_ = height;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  const size_t size = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  log_odds_.assign(size, 0.0f);
  last_seen_.assign(size, -std::numeric_limits<float>::max());
  occupied_cells_ = 0;
  ++version_;
}

void RollingCostmap::reset(double robot_x, double robot_y) {
  allocate(width_, height_, robot_x - 0.5 * params_.width, robot_y - 0.5 * params_.height);
}

bool RollingCostmap::index(double x, double y, int* cell_x, int* cell_y) const {
  const int cx = static_cast<int>(std::floor((x - origin_x_) / params_.resolution));
  const int cy = static_cast<int>(std::floor((y - origin_y_) / params_.resolution));
  if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) return false;
  *cell_x = cx;
  *cell_y = cy;
  return true;
}

bool RollingCostmap::needsRecenter(double robot_x, double robot_y) const {
  const double center_x = origin_x_ + 0.5 * params_.width;
  const double center_y = origin_y_ + 0.5 * params_.height;
  const double dx = std::abs(robot_x - center_x);
  const double dy = std::abs(robot_y - center_y);
  const double limit = 0.5 * params_.recenter_margin * params_.width;
  return dx > limit || dy > limit;
}

void RollingCostmap::recenter(double robot_x, double robot_y) {
  const double target_x = robot_x - 0.5 * params_.width;
  const double target_y = robot_y - 0.5 * params_.height;
  const int shift_x = static_cast<int>(
      std::lround((target_x - origin_x_) / params_.resolution));
  const int shift_y = static_cast<int>(
      std::lround((target_y - origin_y_) / params_.resolution));
  if (shift_x == 0 && shift_y == 0) return;

  std::vector<float> log_odds(static_cast<size_t>(width_) * static_cast<size_t>(height_), 0.0f);
  std::vector<float> last_seen(log_odds.size(), -std::numeric_limits<float>::max());

  for (int y = 0; y < height_; ++y) {
    const int source_y = y + shift_y;
    if (source_y < 0 || source_y >= height_) continue;
    for (int x = 0; x < width_; ++x) {
      const int source_x = x + shift_x;
      if (source_x < 0 || source_x >= width_) continue;
      const size_t target = static_cast<size_t>(y) * static_cast<size_t>(width_) +
                            static_cast<size_t>(x);
      const size_t source = static_cast<size_t>(source_y) * static_cast<size_t>(width_) +
                            static_cast<size_t>(source_x);
      log_odds[target] = log_odds_[source];
      last_seen[target] = last_seen_[source];
    }
  }

  log_odds_ = std::move(log_odds);
  last_seen_ = std::move(last_seen);
  origin_x_ += shift_x * params_.resolution;
  origin_y_ += shift_y * params_.resolution;
  occupied_cells_ = countOccupied();
  ++version_;
}

void RollingCostmap::addFree(int x, int y, double now_sec) {
  const size_t p = position(x, y);
  const float previous = log_odds_[p];
  if (previous >= occupied_threshold_) {
    log_odds_[p] = std::max(0.0f, previous - free_ray_occupied_delta_);
  } else {
    log_odds_[p] = std::max(min_log_odds_, previous - free_delta_);
  }
  if (previous >= 0.0f && log_odds_[p] < 0.0f) {
    if (occupied_cells_ > 0) --occupied_cells_;
  }
  last_seen_[p] = static_cast<float>(now_sec);
}

void RollingCostmap::addOccupied(int x, int y, double now_sec) {
  const size_t p = position(x, y);
  const float previous = log_odds_[p];
  log_odds_[p] = std::min(max_log_odds_, previous + occupied_delta_);
  if (previous < 0.0f && log_odds_[p] >= 0.0f) ++occupied_cells_;
  last_seen_[p] = static_cast<float>(now_sec);
}

void RollingCostmap::castRay(int start_x, int start_y, int end_x, int end_y, double z_start,
                             double z_end, double now_sec) {
  const int delta_x = std::abs(end_x - start_x);
  const int delta_y = std::abs(end_y - start_y);
  if (delta_x == 0 && delta_y == 0) return;

  const int step_x = end_x > start_x ? 1 : (end_x < start_x ? -1 : 0);
  const int step_y = end_y > start_y ? 1 : (end_y < start_y ? -1 : 0);
  const double span = std::max(1e-9, std::hypot(static_cast<double>(delta_x),
                                                static_cast<double>(delta_y)));
  const double t_delta_x = step_x != 0 ? span / std::abs(end_x - start_x) : 0.0;
  const double t_delta_y = step_y != 0 ? span / std::abs(end_y - start_y) : 0.0;
  double t_max_x = step_x != 0 ? 0.5 * t_delta_x : std::numeric_limits<double>::max();
  double t_max_y = step_y != 0 ? 0.5 * t_delta_y : std::numeric_limits<double>::max();

  const double z_span = z_end - z_start;
  int x = start_x;
  int y = start_y;
  double t = 0.0;

  while (x != end_x || y != end_y) {
    if (t_max_x < t_max_y) {
      x += step_x;
      t = t_max_x;
      t_max_x += t_delta_x;
    } else {
      y += step_y;
      t = t_max_y;
      t_max_y += t_delta_y;
    }
    if (x < 0 || y < 0 || x >= width_ || y >= height_) return;
    if (x == end_x && y == end_y) return;

    const double z = z_start + (t / span) * z_span;
    if (z < params_.clear_min_z || z > params_.clear_max_z) continue;

    const size_t p = position(x, y);
    if (log_odds_[p] >= ray_block_threshold_) return;
    addFree(x, y, now_sec);
  }
}

void RollingCostmap::clearGround(const PointCloudT& ground, const Eigen::Isometry3d& sensor_to_map,
                                 double now_sec) {
  for (const auto& point : ground.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const Eigen::Vector3d world =
        sensor_to_map * Eigen::Vector3d(point.x, point.y, point.z);
    if (world.z() < params_.clear_min_z || world.z() > params_.clear_max_z) continue;
    int cx = 0;
    int cy = 0;
    if (!index(world.x(), world.y(), &cx, &cy)) continue;
    addFree(cx, cy, now_sec);
  }
}

void RollingCostmap::castObstacles(const PointCloudT& nonground,
                                   const Eigen::Isometry3d& sensor_to_map, double now_sec) {
  const Eigen::Vector3d origin = sensor_to_map.translation();
  const int origin_x = static_cast<int>(
      std::floor((origin.x() - origin_x_) / params_.resolution));
  const int origin_y = static_cast<int>(
      std::floor((origin.y() - origin_y_) / params_.resolution));

  const double dedupe_deg = params_.raycast_dedupe_deg;
  const int buckets = dedupe_deg > 0.0 ? static_cast<int>(std::lround(360.0 / dedupe_deg)) : 0;
  std::vector<float> bucket_dist2(buckets > 0 ? static_cast<size_t>(buckets) : 0,
                                  std::numeric_limits<float>::max());
  std::vector<size_t> bucket_point(buckets > 0 ? static_cast<size_t>(buckets) : 0, 0);
  std::vector<uint8_t> bucket_used(buckets > 0 ? static_cast<size_t>(buckets) : 0, 0);

  for (size_t i = 0; i < nonground.points.size(); ++i) {
    const auto& point = nonground.points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const Eigen::Vector3d world = sensor_to_map * Eigen::Vector3d(point.x, point.y, point.z);
    const double dx = world.x() - origin.x();
    const double dy = world.y() - origin.y();
    const double dz = world.z() - origin.z();
    const double distance2 = dx * dx + dy * dy + dz * dz;
    if (distance2 > max_range2_) continue;
    if (world.z() < params_.obstacle_min_z || world.z() > params_.obstacle_max_z) continue;

    if (buckets > 0) {
      double azimuth = std::atan2(dy, dx);
      if (azimuth < 0.0) azimuth += kTwoPi;
      const int bucket = std::min(buckets - 1,
                                  static_cast<int>(azimuth / kTwoPi * static_cast<double>(buckets)));
      const size_t slot = static_cast<size_t>(bucket);
      if (bucket_used[slot] == 0 || static_cast<float>(distance2) < bucket_dist2[slot]) {
        bucket_dist2[slot] = static_cast<float>(distance2);
        bucket_point[slot] = i;
        bucket_used[slot] = 1;
      }
    } else {
      bucket_used.push_back(1);
      bucket_point.push_back(i);
    }
  }

  for (size_t slot = 0; slot < bucket_used.size(); ++slot) {
    if (bucket_used[slot] == 0) continue;
    const auto& point = nonground.points[bucket_point[slot]];
    const Eigen::Vector3d world = sensor_to_map * Eigen::Vector3d(point.x, point.y, point.z);
    int end_x = 0;
    int end_y = 0;
    if (!index(world.x(), world.y(), &end_x, &end_y)) continue;
    if (origin_x >= 0 && origin_y >= 0 && origin_x < width_ && origin_y < height_) {
      castRay(origin_x, origin_y, end_x, end_y, origin.z(), world.z(), now_sec);
    }
    addOccupied(end_x, end_y, now_sec);
  }
}

void RollingCostmap::update(const PointCloudT& ground, const PointCloudT& nonground,
                            const Eigen::Isometry3d& sensor_to_map, double now_sec) {
  clearGround(ground, sensor_to_map, now_sec);
  castObstacles(nonground, sensor_to_map, now_sec);
  ++version_;
}

void RollingCostmap::decay(double now_sec) {
  const float hold = static_cast<float>(params_.blind_hold_sec);
  const int min_component = params_.min_component_cells;
  std::vector<uint8_t> stale(log_odds_.size(), 0);
  bool any = false;
  for (size_t p = 0; p < log_odds_.size(); ++p) {
    if (log_odds_[p] <= 0.0f) continue;
    if (static_cast<float>(now_sec) - last_seen_[p] <= hold) continue;
    stale[p] = 1;
    any = true;
  }
  if (any && min_component > 1) {
    std::vector<uint8_t> visited(stale.size(), 0);
    std::vector<size_t> stack;
    std::vector<size_t> component;
    for (size_t seed = 0; seed < stale.size(); ++seed) {
      if (stale[seed] == 0 || visited[seed] != 0) continue;
      stack.clear();
      component.clear();
      stack.push_back(seed);
      visited[seed] = 1;
      while (!stack.empty()) {
        const size_t current = stack.back();
        stack.pop_back();
        component.push_back(current);
        const int cx = static_cast<int>(current % static_cast<size_t>(width_));
        const int cy = static_cast<int>(current / static_cast<size_t>(width_));
        for (int ny = std::max(0, cy - 1); ny <= std::min(height_ - 1, cy + 1); ++ny) {
          for (int nx = std::max(0, cx - 1); nx <= std::min(width_ - 1, cx + 1); ++nx) {
            const size_t neighbour = position(nx, ny);
            if (stale[neighbour] == 0 || visited[neighbour] != 0) continue;
            visited[neighbour] = 1;
            stack.push_back(neighbour);
          }
        }
      }
      if (static_cast<int>(component.size()) < min_component) {
        for (const size_t cell : component) log_odds_[cell] = 0.0f;
      }
    }
  }
  for (size_t p = 0; p < log_odds_.size(); ++p) {
    if (static_cast<float>(now_sec) - last_seen_[p] <= hold) continue;
    if (log_odds_[p] > 0.0f) {
      log_odds_[p] = std::max(0.0f, log_odds_[p] - occupied_decay_);
    } else if (log_odds_[p] < 0.0f) {
      log_odds_[p] = std::min(0.0f, log_odds_[p] + free_decay_);
    }
    if (log_odds_[p] == 0.0f) {
      last_seen_[p] = -std::numeric_limits<float>::max();
    }
  }
  occupied_cells_ = countOccupied();
  ++version_;
}

size_t RollingCostmap::countOccupied() const {
  size_t count = 0;
  for (size_t p = 0; p < log_odds_.size(); ++p) {
    if (log_odds_[p] >= occupied_threshold_) ++count;
  }
  return count;
}

void RollingCostmap::takeSnapshot(CostmapSnapshot* snapshot) const {
  snapshot->resolution = params_.resolution;
  snapshot->origin_x = origin_x_;
  snapshot->origin_y = origin_y_;
  snapshot->width = width_;
  snapshot->height = height_;
  snapshot->cells.resize(log_odds_.size());
  for (size_t p = 0; p < log_odds_.size(); ++p) {
    if (last_seen_[p] < -1e30f) {
      snapshot->cells[p] = -1;
      continue;
    }
    const double probability = 1.0 / (1.0 + std::exp(-static_cast<double>(log_odds_[p])));
    const int value = static_cast<int>(std::lround(probability * 100.0));
    snapshot->cells[p] = static_cast<int8_t>(std::min(100, std::max(0, value)));
  }
}

nav_msgs::msg::OccupancyGrid RollingCostmap::toMessage(const CostmapSnapshot& snapshot,
                                                      const std::string& frame_id,
                                                      const rclcpp::Time& stamp) {
  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.info.resolution = static_cast<float>(snapshot.resolution);
  message.info.width = static_cast<uint32_t>(snapshot.width);
  message.info.height = static_cast<uint32_t>(snapshot.height);
  message.info.origin.position.x = snapshot.origin_x;
  message.info.origin.position.y = snapshot.origin_y;
  message.info.origin.position.z = 0.0;
  message.info.origin.orientation.w = 1.0;
  message.data = snapshot.cells;
  return message;
}

}
