#include "ground_segmentation/segmenter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace ground_segmentation {

PolarGroundSegmenter::PolarGroundSegmenter(const SegmentationParams& params) : params_(params) {
  params_.ring_size = std::max(0.05, params_.ring_size);
  params_.max_range = std::max(params_.min_range + params_.ring_size, params_.max_range);
  params_.sectors = std::max(4, params_.sectors);
  params_.min_cell_points = std::max(1, params_.min_cell_points);
  params_.ground_alpha = std::clamp(params_.ground_alpha, 0.0, 1.0);
  rings_ = static_cast<int>(std::ceil((params_.max_range - params_.min_range) /
                                      params_.ring_size));
}

SegmentationResult PolarGroundSegmenter::segment(
    const PointCloudT& cloud, const Eigen::Matrix3d& gravity_rotation) const {
  SegmentationResult result;
  if (cloud.empty()) return result;

  const double two_pi = 2.0 * M_PI;
  std::vector<Eigen::Vector3d> level_points(cloud.size(), Eigen::Vector3d::Zero());
  std::vector<std::vector<int>> cells(static_cast<size_t>(rings_ * params_.sectors));
  std::vector<uint8_t> labels(cloud.size(), 255);
  for (size_t index = 0; index < cloud.size(); ++index) {
    const auto& point = cloud.points[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const Eigen::Vector3d p =
        gravity_rotation * Eigen::Vector3d(point.x, point.y, point.z);
    const double radius = std::hypot(p.x(), p.y());
    if (radius < params_.min_range || radius >= params_.max_range) continue;
    const int ring = static_cast<int>((radius - params_.min_range) / params_.ring_size);
    if (ring < 0 || ring >= rings_) continue;
    double angle = std::atan2(p.y(), p.x());
    if (angle < 0.0) angle += two_pi;
    int sector = static_cast<int>(angle / two_pi * params_.sectors);
    sector = std::clamp(sector, 0, params_.sectors - 1);
    level_points[index] = p;
    cells[static_cast<size_t>(ring * params_.sectors + sector)].push_back(
        static_cast<int>(index));
    ++result.finite_points;
  }

  std::vector<double> ground_samples;
  for (int ring = 0; ring < rings_; ++ring) {
    const double radius = params_.min_range + (ring + 0.5) * params_.ring_size;
    if (radius > params_.ground_estimation_range) break;
    for (int sector = 0; sector < params_.sectors; ++sector) {
      const auto& cell = cells[static_cast<size_t>(ring * params_.sectors + sector)];
      if (static_cast<int>(cell.size()) < params_.min_cell_points) continue;
      double minimum = std::numeric_limits<double>::max();
      for (int index : cell) minimum = std::min(minimum, level_points[index].z());
      ground_samples.push_back(minimum);
    }
  }
  double global_ground = 0.0;
  if (!ground_samples.empty()) {
    const auto middle = ground_samples.begin() + ground_samples.size() / 2;
    std::nth_element(ground_samples.begin(), middle, ground_samples.end());
    global_ground = *middle;
  }

#pragma omp parallel for schedule(dynamic, 1) if(params_.sectors > 16)
  for (int sector = 0; sector < params_.sectors; ++sector) {
    bool has_ground = false;
    double tracked_ground = global_ground;
    int last_ground_ring = -1;
    for (int ring = 0; ring < rings_; ++ring) {
      const auto& cell = cells[static_cast<size_t>(ring * params_.sectors + sector)];
      if (static_cast<int>(cell.size()) < params_.min_cell_points) continue;
      double minimum = std::numeric_limits<double>::max();
      for (int index : cell) minimum = std::min(minimum, level_points[index].z());
      double seed_sum = 0.0;
      int seed_count = 0;
      for (int index : cell) {
        const double z = level_points[index].z();
        if (z - minimum <= params_.seed_height_band) {
          seed_sum += z;
          ++seed_count;
        }
      }
      const double seed = seed_count > 0 ? seed_sum / seed_count : minimum;
      const double radial_gap = last_ground_ring >= 0
                                    ? (ring - last_ground_ring) * params_.ring_size
                                    : params_.ring_size;
      const double allowed_step = params_.max_ground_step +
                                  std::tan(params_.max_slope_deg * M_PI / 180.0) * radial_gap;
      bool ground_cell = false;
      if (!has_ground) {
        ground_cell = std::abs(seed - global_ground) <= params_.initial_ground_tolerance;
      } else {
        ground_cell = std::abs(seed - tracked_ground) <= allowed_step;
      }
      if (ground_cell) {
        tracked_ground = has_ground
                             ? params_.ground_alpha * seed +
                                   (1.0 - params_.ground_alpha) * tracked_ground
                             : seed;
        has_ground = true;
        last_ground_ring = ring;
      }
      const double ground_z = tracked_ground;
      for (int index : cell) {
        const double dz = level_points[index].z() - ground_z;
        if (std::abs(dz) <= params_.ground_height_threshold) {
          labels[index] = 0;
        } else if (dz >= params_.elevated_height) {
          labels[index] = 2;
        } else {
          labels[index] = 1;
        }
      }
    }
  }

  result.ground->reserve(result.finite_points);
  result.obstacles->reserve(result.finite_points);
  result.elevated->reserve(result.finite_points);
  for (size_t index = 0; index < cloud.size(); ++index) {
    if (labels[index] == 0) {
      result.ground->push_back(cloud.points[index]);
    } else if (labels[index] == 1) {
      result.obstacles->push_back(cloud.points[index]);
    } else if (labels[index] == 2) {
      result.elevated->push_back(cloud.points[index]);
    }
  }
  for (auto* output : {result.ground.get(), result.obstacles.get(), result.elevated.get()}) {
    output->width = static_cast<uint32_t>(output->size());
    output->height = 1;
  }
  return result;
}

}
