#include "occupancy_map_2d/grid.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace occupancy_map_2d {

OccupancyGrid2D::OccupancyGrid2D(const GridParams& params) : params_(params) {
  params_.resolution = std::max(0.01, params_.resolution);
  width_ = std::max(1, static_cast<int>(std::ceil(params_.width / params_.resolution)));
  height_ = std::max(1, static_cast<int>(std::ceil(params_.height / params_.resolution)));
  log_odds_.resize(static_cast<size_t>(width_ * height_), 0.0f);
  observed_.resize(static_cast<size_t>(width_ * height_), 0);
}

void OccupancyGrid2D::clear() {
  std::fill(log_odds_.begin(), log_odds_.end(), 0.0f);
  std::fill(observed_.begin(), observed_.end(), 0);
  updated_cells_ = 0;
}

bool OccupancyGrid2D::index(double x, double y, int* cell_x, int* cell_y) const {
  const int ix = static_cast<int>(std::floor((x - params_.origin_x) / params_.resolution));
  const int iy = static_cast<int>(std::floor((y - params_.origin_y) / params_.resolution));
  if (ix < 0 || iy < 0 || ix >= width_ || iy >= height_) return false;
  *cell_x = ix;
  *cell_y = iy;
  return true;
}

void OccupancyGrid2D::addFree(int x, int y) {
  if (x < 0 || y < 0 || x >= width_ || y >= height_) return;
  const size_t position = static_cast<size_t>(y * width_ + x);
  if (!observed_[position]) {
    observed_[position] = 1;
    ++updated_cells_;
  }
  log_odds_[position] = std::clamp(log_odds_[position] + static_cast<float>(params_.free_log_odds),
                                   static_cast<float>(params_.min_log_odds),
                                   static_cast<float>(params_.max_log_odds));
}

void OccupancyGrid2D::addOccupied(int x, int y) {
  if (x < 0 || y < 0 || x >= width_ || y >= height_) return;
  const size_t position = static_cast<size_t>(y * width_ + x);
  if (!observed_[position]) {
    observed_[position] = 1;
    ++updated_cells_;
  }
  log_odds_[position] = std::clamp(
      log_odds_[position] + static_cast<float>(params_.occupied_log_odds),
      static_cast<float>(params_.min_log_odds), static_cast<float>(params_.max_log_odds));
}

void OccupancyGrid2D::raycast(int start_x, int start_y, int end_x, int end_y,
                              bool clear_endpoint) {
  const int delta_x = std::abs(end_x - start_x);
  const int step_x = start_x < end_x ? 1 : -1;
  const int delta_y = -std::abs(end_y - start_y);
  const int step_y = start_y < end_y ? 1 : -1;
  int error = delta_x + delta_y;
  int x = start_x;
  int y = start_y;
  while (true) {
    const bool endpoint = x == end_x && y == end_y;
    if (!endpoint || clear_endpoint) addFree(x, y);
    if (endpoint) break;
    const int doubled = 2 * error;
    if (doubled >= delta_y) {
      error += delta_y;
      x += step_x;
    }
    if (doubled <= delta_x) {
      error += delta_x;
      y += step_y;
    }
  }
}

void OccupancyGrid2D::update(const PointCloudT& ground, const PointCloudT& obstacles,
                             const Eigen::Isometry3d& sensor_to_map) {
  int sensor_x = 0;
  int sensor_y = 0;
  const auto& sensor = sensor_to_map.translation();
  if (!sensor.allFinite() || !index(sensor.x(), sensor.y(), &sensor_x, &sensor_y)) return;

  std::vector<uint8_t> ground_mask(log_odds_.size(), 0);
  std::vector<uint8_t> obstacle_mask(log_odds_.size(), 0);
  std::vector<int> ground_cells;
  std::vector<int> obstacle_cells;
  ground_cells.reserve(ground.size());
  obstacle_cells.reserve(obstacles.size());

  for (const auto& point : ground.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const Eigen::Vector3d p = sensor_to_map * Eigen::Vector3d(point.x, point.y, point.z);
    if (std::hypot(p.x() - sensor.x(), p.y() - sensor.y()) > params_.max_range) continue;
    int x = 0;
    int y = 0;
    if (!index(p.x(), p.y(), &x, &y)) continue;
    const int cell = y * width_ + x;
    if (!ground_mask[cell]) {
      ground_mask[cell] = 1;
      ground_cells.push_back(cell);
    }
  }
  for (const auto& point : obstacles.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const Eigen::Vector3d p = sensor_to_map * Eigen::Vector3d(point.x, point.y, point.z);
    const double height = p.z() - sensor.z();
    if (height < params_.obstacle_min_height || height > params_.obstacle_max_height ||
        std::hypot(p.x() - sensor.x(), p.y() - sensor.y()) > params_.max_range) {
      continue;
    }
    int x = 0;
    int y = 0;
    if (!index(p.x(), p.y(), &x, &y)) continue;
    const int cell = y * width_ + x;
    if (!obstacle_mask[cell]) {
      obstacle_mask[cell] = 1;
      obstacle_cells.push_back(cell);
    }
  }

  for (int cell : ground_cells) {
    raycast(sensor_x, sensor_y, cell % width_, cell / width_, true);
  }
  for (int cell : obstacle_cells) {
    raycast(sensor_x, sensor_y, cell % width_, cell / width_, false);
  }
  for (int cell : obstacle_cells) addOccupied(cell % width_, cell / width_);
}

double OccupancyGrid2D::probability(int position) const {
  return 1.0 / (1.0 + std::exp(-static_cast<double>(log_odds_[position])));
}

size_t OccupancyGrid2D::occupiedCells() const {
  size_t count = 0;
  for (size_t index = 0; index < log_odds_.size(); ++index) {
    if (observed_[index] && probability(static_cast<int>(index)) >= params_.occupied_probability) {
      ++count;
    }
  }
  return count;
}

nav_msgs::msg::OccupancyGrid OccupancyGrid2D::message(const std::string& frame_id,
                                                       const rclcpp::Time& stamp) const {
  cv::Mat binary(height_, width_, CV_8UC1, cv::Scalar(0));
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const int position = y * width_ + x;
      if (observed_[position] && probability(position) >= params_.occupied_probability) {
        binary.at<uint8_t>(y, x) = 255;
      }
    }
  }

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  std::vector<bool> removed(1, false);
  if (params_.min_component_cells > 1 && cv::countNonZero(binary) > 0) {
    const int components = cv::connectedComponentsWithStats(
        binary, labels, stats, centroids, 8, CV_32S);
    removed.resize(static_cast<size_t>(components), false);
    for (int component = 1; component < components; ++component) {
      removed[component] = stats.at<int>(component, cv::CC_STAT_AREA) < params_.min_component_cells;
    }
  }

  nav_msgs::msg::OccupancyGrid output;
  output.header.frame_id = frame_id;
  output.header.stamp = stamp;
  output.info.resolution = static_cast<float>(params_.resolution);
  output.info.width = static_cast<uint32_t>(width_);
  output.info.height = static_cast<uint32_t>(height_);
  output.info.origin.position.x = params_.origin_x;
  output.info.origin.position.y = params_.origin_y;
  output.info.origin.orientation.w = 1.0;
  output.data.resize(log_odds_.size(), -1);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const int position = y * width_ + x;
      if (!observed_[position]) continue;
      if (!labels.empty() && removed[labels.at<int>(y, x)]) continue;
      const int value = static_cast<int>(std::lround(probability(position) * 100.0));
      output.data[position] = static_cast<int8_t>(std::clamp(value, 0, 100));
    }
  }
  return output;
}

}
