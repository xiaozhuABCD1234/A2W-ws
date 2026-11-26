#include "occupancy_map_2d/grid.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace occupancy_map_2d {
namespace {

constexpr double kPi = 3.14159265358979323846;

float logit(double probability) {
  const double clamped = std::clamp(probability, 1e-6, 1.0 - 1e-6);
  return static_cast<float>(std::log(clamped / (1.0 - clamped)));
}

}

OccupancyGrid2D::OccupancyGrid2D(const GridParams& params) : params_(params) {
  params_.resolution = std::max(0.01, params_.resolution);
  width_ = std::max(1, static_cast<int>(std::ceil(params_.width / params_.resolution)));
  height_ = std::max(1, static_cast<int>(std::ceil(params_.height / params_.resolution)));
  const size_t cells = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  log_odds_.assign(cells, 0.0f);
  observed_.assign(cells, 0);
  ground_mask_.assign(cells, 0);
  obstacle_mask_.assign(cells, 0);
  occupied_threshold_ = logit(params_.occupied_probability);
  free_delta_ = static_cast<float>(params_.free_log_odds);
  occupied_delta_ = static_cast<float>(params_.occupied_log_odds);
  min_log_odds_ = static_cast<float>(params_.min_log_odds);
  max_log_odds_ = static_cast<float>(params_.max_log_odds);
  max_range2_ = params_.max_range * params_.max_range;
  observed_cells_.reserve(cells / 8);
  ground_cells_.reserve(8192);
  obstacle_cells_.reserve(8192);
  ground_ray_cells_.reserve(2048);
  ground_touched_.reserve(8192);
  obstacle_touched_.reserve(8192);
}

void OccupancyGrid2D::clear() {
  std::fill(log_odds_.begin(), log_odds_.end(), 0.0f);
  std::fill(observed_.begin(), observed_.end(), 0);

  for (int cell : ground_touched_) ground_mask_[cell] = 0;
  for (int cell : obstacle_touched_) obstacle_mask_[cell] = 0;
  ground_touched_.clear();
  obstacle_touched_.clear();
  ground_cells_.clear();
  obstacle_cells_.clear();
  ground_ray_cells_.clear();
  observed_cells_.clear();
  occupied_cells_ = 0;
  ++version_;
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
  const size_t position = static_cast<size_t>(y) * static_cast<size_t>(width_) + static_cast<size_t>(x);
  const float before = log_odds_[position];
  float after = before + free_delta_;
  after = after < min_log_odds_ ? min_log_odds_ : (after > max_log_odds_ ? max_log_odds_ : after);
  log_odds_[position] = after;
  if (before >= occupied_threshold_ && after < occupied_threshold_) --occupied_cells_;
  if (!observed_[position]) {
    observed_[position] = 1;
    observed_cells_.push_back(static_cast<int>(position));
  }
  ++version_;
}

void OccupancyGrid2D::addOccupied(int x, int y) {
  if (x < 0 || y < 0 || x >= width_ || y >= height_) return;
  const size_t position = static_cast<size_t>(y) * static_cast<size_t>(width_) + static_cast<size_t>(x);
  const float before = log_odds_[position];
  float after = before + occupied_delta_;
  after = after < min_log_odds_ ? min_log_odds_ : (after > max_log_odds_ ? max_log_odds_ : after);
  log_odds_[position] = after;
  if (before < occupied_threshold_ && after >= occupied_threshold_) ++occupied_cells_;
  if (!observed_[position]) {
    observed_[position] = 1;
    observed_cells_.push_back(static_cast<int>(position));
  }
  ++version_;
}

void OccupancyGrid2D::raycast(int start_x, int start_y, int end_x, int end_y,
                              bool clear_endpoint, const std::vector<uint8_t>* blocked) {
  const int delta_x = std::abs(end_x - start_x);
  const int step_x = start_x < end_x ? 1 : -1;
  const int delta_y = -std::abs(end_y - start_y);
  const int step_y = start_y < end_y ? 1 : -1;
  int error = delta_x + delta_y;
  int x = start_x;
  int y = start_y;
  while (true) {
    if (blocked != nullptr) {
      const size_t position = static_cast<size_t>(y) * static_cast<size_t>(width_) +
                              static_cast<size_t>(x);
      if ((*blocked)[position] || log_odds_[position] >= occupied_threshold_) break;
    }
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

const std::vector<int>& OccupancyGrid2D::dedupeGroundRays(int sensor_x, int sensor_y) {
  const double step = params_.raycast_dedupe_deg * kPi / 180.0;
  const int quadrant_buckets = std::max(1, static_cast<int>(std::ceil(2.0 * kPi / step / 4.0)));
  const int buckets = quadrant_buckets * 4;
  if (static_cast<int>(bucket_dist2_.size()) != buckets) {
    bucket_dist2_.assign(static_cast<size_t>(buckets), -1);
    bucket_cell_.assign(static_cast<size_t>(buckets), -1);
  } else {
    std::fill(bucket_dist2_.begin(), bucket_dist2_.end(), -1);
    std::fill(bucket_cell_.begin(), bucket_cell_.end(), -1);
  }

  for (int cell : ground_cells_) {
    const int dx = cell % width_ - sensor_x;
    const int dy = cell / width_ - sensor_y;
    const int abs_x = std::abs(dx);
    const int abs_y = std::abs(dy);
    const int quadrant = dx >= 0 ? (dy >= 0 ? 0 : 3) : (dy >= 0 ? 1 : 2);
    const int offset = static_cast<int>(
        static_cast<long long>(abs_y) * quadrant_buckets / (abs_x + abs_y));
    const int bucket = quadrant * quadrant_buckets +
                       std::min(offset, quadrant_buckets - 1);
    const int dist2 = dx * dx + dy * dy;
    if (dist2 > bucket_dist2_[static_cast<size_t>(bucket)]) {
      bucket_dist2_[static_cast<size_t>(bucket)] = dist2;
      bucket_cell_[static_cast<size_t>(bucket)] = cell;
    }
  }

  ground_ray_cells_.clear();
  for (int cell : bucket_cell_) {
    if (cell >= 0) ground_ray_cells_.push_back(cell);
  }
  return ground_ray_cells_;
}

void OccupancyGrid2D::update(const PointCloudT& ground, const PointCloudT& nonground,
                             const Eigen::Isometry3d& sensor_to_map) {
  int sensor_x = 0;
  int sensor_y = 0;
  const auto& sensor = sensor_to_map.translation();
  if (!sensor.allFinite() || !index(sensor.x(), sensor.y(), &sensor_x, &sensor_y)) return;

  for (int cell : ground_touched_) ground_mask_[cell] = 0;
  for (int cell : obstacle_touched_) obstacle_mask_[cell] = 0;
  ground_touched_.clear();
  obstacle_touched_.clear();
  ground_cells_.clear();
  obstacle_cells_.clear();

  auto project = [&](const PointCloudT& cloud, bool filter_height, std::vector<uint8_t>& mask,
                     std::vector<int>& touched, std::vector<int>& cells) {
    for (const auto& point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      const Eigen::Vector3d p = sensor_to_map * Eigen::Vector3d(point.x, point.y, point.z);
      const double dx = p.x() - sensor.x();
      const double dy = p.y() - sensor.y();
      if (dx * dx + dy * dy > max_range2_) continue;
      if (filter_height) {
        const double height = p.z() - sensor.z();
        if (height < params_.obstacle_min_height || height > params_.obstacle_max_height) continue;
      }
      int x = 0;
      int y = 0;
      if (!index(p.x(), p.y(), &x, &y)) continue;
      const size_t position = static_cast<size_t>(y) * static_cast<size_t>(width_) +
                              static_cast<size_t>(x);
      if (mask[position]) continue;
      mask[position] = 1;
      touched.push_back(static_cast<int>(position));
      cells.push_back(static_cast<int>(position));
    }
  };
  project(ground, false, ground_mask_, ground_touched_, ground_cells_);
  project(nonground, true, obstacle_mask_, obstacle_touched_, obstacle_cells_);

  const std::vector<int>* ground_rays = &ground_cells_;
  if (params_.raycast_dedupe_deg > 0.0 && ground_cells_.size() > 1) {
    ground_rays = &dedupeGroundRays(sensor_x, sensor_y);
  }
  for (int cell : *ground_rays) {
    raycast(sensor_x, sensor_y, cell % width_, cell / width_, true, &obstacle_mask_);
  }
  for (int cell : obstacle_cells_) {
    raycast(sensor_x, sensor_y, cell % width_, cell / width_, false, &obstacle_mask_);
  }
  for (int cell : obstacle_cells_) addOccupied(cell % width_, cell / width_);
}

void OccupancyGrid2D::takeSnapshot(GridSnapshot* snapshot) const {
  snapshot->resolution = params_.resolution;
  snapshot->origin_x = params_.origin_x;
  snapshot->origin_y = params_.origin_y;
  snapshot->occupied_probability = params_.occupied_probability;
  snapshot->min_component_cells = params_.min_component_cells;
  snapshot->width = width_;
  snapshot->height = height_;
  snapshot->cells.assign(observed_cells_.begin(), observed_cells_.end());
  snapshot->log_odds.resize(observed_cells_.size());
  for (size_t i = 0; i < observed_cells_.size(); ++i) {
    snapshot->log_odds[i] = log_odds_[observed_cells_[i]];
  }
}

nav_msgs::msg::OccupancyGrid OccupancyGrid2D::toMessage(const GridSnapshot& snapshot,
                                                        const std::string& frame_id,
                                                        const rclcpp::Time& stamp) {
  const int width = snapshot.width;
  const int height = snapshot.height;
  const size_t cell_count = static_cast<size_t>(width) * static_cast<size_t>(height);

  nav_msgs::msg::OccupancyGrid output;
  output.header.frame_id = frame_id;
  output.header.stamp = stamp;
  output.info.resolution = static_cast<float>(snapshot.resolution);
  output.info.width = static_cast<uint32_t>(width);
  output.info.height = static_cast<uint32_t>(height);
  output.info.origin.position.x = snapshot.origin_x;
  output.info.origin.position.y = snapshot.origin_y;
  output.info.origin.orientation.w = 1.0;
  output.data.assign(cell_count, -1);

  const size_t observed = snapshot.cells.size();
  if (observed == 0) return output;

  thread_local std::vector<int8_t> values_storage;
  thread_local std::vector<int32_t> occupied_storage;
  thread_local cv::Mat binary;
  thread_local cv::Mat labels;
  thread_local cv::Mat stats;
  thread_local cv::Mat centroids;
  thread_local std::vector<uint8_t> small_component_storage;

  std::vector<int8_t>& values = values_storage;
  std::vector<int32_t>& occupied = occupied_storage;
  std::vector<uint8_t>& small_component = small_component_storage;

  values.resize(observed);
  occupied.clear();
  small_component.clear();

  const double threshold = snapshot.occupied_probability;
  int min_cell_x = width;
  int max_cell_x = -1;
  int min_cell_y = height;
  int max_cell_y = -1;
  for (size_t i = 0; i < observed; ++i) {
    const double probability = 1.0 / (1.0 + std::exp(-static_cast<double>(snapshot.log_odds[i])));
    values[i] = static_cast<int8_t>(
        std::clamp(static_cast<int>(std::lround(probability * 100.0)), 0, 100));
    if (probability < threshold) continue;
    occupied.push_back(static_cast<int32_t>(i));
    const int position = snapshot.cells[i];
    const int x = position % width;
    const int y = position / width;
    min_cell_x = std::min(min_cell_x, x);
    max_cell_x = std::max(max_cell_x, x);
    min_cell_y = std::min(min_cell_y, y);
    max_cell_y = std::max(max_cell_y, y);
  }

  if (snapshot.min_component_cells > 1 && !occupied.empty()) {
    const int roi_width = max_cell_x - min_cell_x + 1;
    const int roi_height = max_cell_y - min_cell_y + 1;
    binary.create(roi_height, roi_width, CV_8UC1);
    binary.setTo(cv::Scalar(0));
    uint8_t* binary_data = binary.ptr<uint8_t>(0);
    for (int32_t k : occupied) {
      const int position = snapshot.cells[k];
      const int x = position % width - min_cell_x;
      const int y = position / width - min_cell_y;
      binary_data[static_cast<size_t>(y) * static_cast<size_t>(roi_width) + static_cast<size_t>(x)] = 255;
    }

    const int components =
        cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8, CV_32S);
    small_component.assign(static_cast<size_t>(components), 0);
    for (int component = 1; component < components; ++component) {
      small_component[static_cast<size_t>(component)] =
          stats.at<int>(component, cv::CC_STAT_AREA) < snapshot.min_component_cells ? 1 : 0;
    }
    const int32_t* label_data = labels.ptr<int32_t>(0);
    for (int32_t k : occupied) {
      const int position = snapshot.cells[k];
      const int x = position % width - min_cell_x;
      const int y = position / width - min_cell_y;
      if (small_component[static_cast<size_t>(
              label_data[static_cast<size_t>(y) * static_cast<size_t>(roi_width) + static_cast<size_t>(x)])]) {
        values[k] = -1;
      }
    }
  }

  for (size_t i = 0; i < observed; ++i) {
    const int8_t value = values[i];
    if (value < 0) continue;
    output.data[static_cast<size_t>(snapshot.cells[i])] = value;
  }
  return output;
}

nav_msgs::msg::OccupancyGrid OccupancyGrid2D::message(const std::string& frame_id,
                                                      const rclcpp::Time& stamp) const {
  GridSnapshot snapshot;
  takeSnapshot(&snapshot);
  return toMessage(snapshot, frame_id, stamp);
}

}
