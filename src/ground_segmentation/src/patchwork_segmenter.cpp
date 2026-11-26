#include "ground_segmentation/patchwork_segmenter.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace ground_segmentation {
namespace {

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0.0;
  const size_t kth = static_cast<size_t>(std::clamp(fraction, 0.0, 1.0) * (values.size() - 1));
  std::nth_element(values.begin(), values.begin() + kth, values.end());
  return values[kth];
}

}

PatchworkGroundSegmenter::PatchworkGroundSegmenter(const PatchworkParams& params)
    : params_(params) {
  params_.max_range = std::max(params_.min_range + 1.0, params_.max_range);
  params_.num_iter = std::max(1, params_.num_iter);
  params_.num_lpr = std::max(3, params_.num_lpr);
  params_.min_bucket_points = std::max(3, params_.min_bucket_points);
  params_.th_seeds = std::max(0.0, params_.th_seeds);
  params_.th_dist = std::max(0.01, params_.th_dist);
  params_.uprightness_thr = std::clamp(params_.uprightness_thr, 0.0, 1.0);
  params_.flatness_thr = std::max(0.0, params_.flatness_thr);
  params_.ground_bottom_percentile =
      std::clamp(params_.ground_bottom_percentile, 0.0, 1.0);
  params_.temporal_revert_ratio = std::clamp(params_.temporal_revert_ratio, 0.0, 1.0);

  auto& zones = params_.zone_ranges;
  if (zones.size() < 2) zones = {0.0, params_.max_range};
  zones.front() = std::max(0.0, zones.front());
  zones.back() = std::max(zones.back(), params_.max_range);
  for (size_t i = 1; i < zones.size(); ++i) zones[i] = std::max(zones[i], zones[i - 1] + 1e-3);

  int zone_count = static_cast<int>(zones.size()) - 1;
  if (static_cast<int>(params_.sectors_per_zone.size()) != zone_count ||
      static_cast<int>(params_.rings_per_zone.size()) != zone_count) {
    zones = {0.0, 4.0, 8.0, 14.0, std::max(params_.max_range, 30.0)};
    params_.sectors_per_zone = {16, 32, 54, 32};
    params_.rings_per_zone = {2, 4, 4, 4};
    zone_count = 4;
  }
  num_zones_ = zone_count;

  bucket_offset_.assign(static_cast<size_t>(num_zones_) + 1, 0);
  for (int zone = 0; zone < num_zones_; ++zone) {
    const int sectors = std::max(1, static_cast<int>(params_.sectors_per_zone[zone]));
    const int rings = std::max(1, static_cast<int>(params_.rings_per_zone[zone]));
    bucket_offset_[static_cast<size_t>(zone) + 1] =
        bucket_offset_[static_cast<size_t>(zone)] + sectors * rings;
  }
  previous_.assign(static_cast<size_t>(bucket_offset_.back()), Plane{});
}

bool PatchworkGroundSegmenter::fitPlane(const std::vector<int>& indices,
                                        const std::vector<Eigen::Vector3d>& points,
                                        Plane* plane) const {
  if (indices.size() < 3) return false;

  std::vector<int> sorted = indices;
  std::sort(sorted.begin(), sorted.end(), [&points](int lhs, int rhs) {
    return points[lhs].z() < points[rhs].z();
  });

  const size_t lpr_count = std::min<size_t>(static_cast<size_t>(params_.num_lpr), sorted.size());
  Eigen::Vector3d lpr = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < lpr_count; ++i) lpr += points[sorted[i]];
  lpr /= static_cast<double>(lpr_count);

  std::vector<int> seeds;
  seeds.reserve(sorted.size());
  for (int index : sorted) {
    if (points[index].z() <= lpr.z() + params_.th_seeds) seeds.push_back(index);
  }
  if (seeds.size() < 3) return false;

  Plane fitted;
  bool converged = false;
  for (int iter = 0; iter < params_.num_iter; ++iter) {
    if (seeds.size() < 3) return false;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int index : seeds) mean += points[index];
    mean /= static_cast<double>(seeds.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int index : seeds) {
      const Eigen::Vector3d delta = points[index] - mean;
      covariance += delta * delta.transpose();
    }
    covariance /= static_cast<double>(seeds.size());

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) return false;
    Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
    if (normal.z() < 0.0) normal = -normal;
    const Eigen::Vector3d eigenvalues = solver.eigenvalues();

    std::vector<int> inliers;
    inliers.reserve(sorted.size());
    for (int index : sorted) {
      if (std::abs(normal.dot(points[index]) - normal.dot(mean)) <= params_.th_dist) {
        inliers.push_back(index);
      }
    }
    if (inliers.size() < 3) return false;

    fitted.normal = normal;
    fitted.offset = -normal.dot(mean);
    fitted.flatness = eigenvalues(0) / std::max(1e-9, eigenvalues(2));
    fitted.valid = true;
    converged = true;
    seeds.swap(inliers);
  }
  if (!converged) return false;
  *plane = fitted;
  return true;
}

SegmentationResult PatchworkGroundSegmenter::segment(
    const PointCloudT& cloud, const Eigen::Matrix3d& gravity_rotation) {
  SegmentationResult result;
  if (cloud.empty()) return result;

  const double two_pi = 2.0 * M_PI;
  const auto& zones = params_.zone_ranges;
  std::vector<Eigen::Vector3d> level_points(cloud.size(), Eigen::Vector3d::Zero());
  std::vector<std::vector<int>> buckets(static_cast<size_t>(bucket_offset_.back()));
  std::vector<uint8_t> labels(cloud.size(), 255);

  for (size_t index = 0; index < cloud.size(); ++index) {
    const auto& point = cloud.points[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const Eigen::Vector3d p = gravity_rotation * Eigen::Vector3d(point.x, point.y, point.z);
    const double radius = std::hypot(p.x(), p.y());
    if (radius < params_.min_range || radius >= params_.max_range) continue;

    int zone = num_zones_ - 1;
    for (int z = 0; z < num_zones_; ++z) {
      if (radius < zones[static_cast<size_t>(z) + 1]) {
        zone = z;
        break;
      }
    }
    const double zone_min = zones[static_cast<size_t>(zone)];
    const double zone_max = zones[static_cast<size_t>(zone) + 1];
    const int sectors = std::max(1, static_cast<int>(params_.sectors_per_zone[zone]));
    const int rings = std::max(1, static_cast<int>(params_.rings_per_zone[zone]));
    const double ring_size = (zone_max - zone_min) / static_cast<double>(rings);

    int ring = static_cast<int>((radius - zone_min) / ring_size);
    ring = std::clamp(ring, 0, rings - 1);
    double angle = std::atan2(p.y(), p.x());
    if (angle < 0.0) angle += two_pi;
    int sector = static_cast<int>(angle / two_pi * sectors);
    sector = std::clamp(sector, 0, sectors - 1);

    const int bucket = bucket_offset_[static_cast<size_t>(zone)] + ring * sectors + sector;
    level_points[index] = p;
    buckets[static_cast<size_t>(bucket)].push_back(static_cast<int>(index));
    ++result.finite_points;
  }

  std::vector<double> bucket_minimum(buckets.size(), std::numeric_limits<double>::max());
  std::vector<double> minima;
  minima.reserve(buckets.size());
  for (size_t b = 0; b < buckets.size(); ++b) {
    double minimum = std::numeric_limits<double>::max();
    for (int index : buckets[b]) minimum = std::min(minimum, level_points[index].z());
    bucket_minimum[b] = minimum;
    if (!buckets[b].empty()) minima.push_back(minimum);
  }
  if (minima.empty()) return result;
  const double global_bottom = percentile(minima, params_.ground_bottom_percentile);
  const double max_ground_z = global_bottom + params_.max_ground_height;

  for (size_t b = 0; b < buckets.size(); ++b) {
    const auto& bucket = buckets[b];
    auto& previous = previous_[b];
    if (bucket.empty()) {
      previous = Plane{};
      continue;
    }
    Plane plane;
    bool ground = false;
    if (static_cast<int>(bucket.size()) >= params_.min_bucket_points &&
        fitPlane(bucket, level_points, &plane)) {
      const double elevation_cosine = std::clamp(plane.normal.z(), -1.0, 1.0);
      const bool upright = elevation_cosine >= params_.uprightness_thr;
      const bool planar = plane.flatness <= params_.flatness_thr;
      const bool low_enough = bucket_minimum[b] <= max_ground_z;
      ground = upright && planar && low_enough;
    }

    if (!ground && params_.enable_temporal_revert && previous.valid) {
      if (bucket_minimum[b] <= max_ground_z) {
        size_t hits = 0;
        for (int index : bucket) {
          if (std::abs(previous.normal.dot(level_points[index]) + previous.offset) <=
              params_.th_dist) {
            ++hits;
          }
        }
        if (static_cast<double>(hits) >=
            params_.temporal_revert_ratio * static_cast<double>(bucket.size())) {
          plane = previous;
          ground = true;
        }
      }
    }

    if (ground) {
      for (int index : bucket) {
        const double distance =
            std::abs(plane.normal.dot(level_points[index]) + plane.offset);
        labels[index] = distance <= params_.th_dist ? 0 : 1;
      }
      previous = plane;
      previous.valid = true;
    } else {
      for (int index : bucket) labels[index] = 1;
      previous = Plane{};
    }
  }

  result.ground->reserve(result.finite_points);
  result.nonground->reserve(result.finite_points);
  for (size_t index = 0; index < cloud.size(); ++index) {
    if (labels[index] == 0) {
      result.ground->push_back(cloud.points[index]);
    } else if (labels[index] == 1) {
      result.nonground->push_back(cloud.points[index]);
    }
  }
  for (auto* output : {result.ground.get(), result.nonground.get()}) {
    output->width = static_cast<uint32_t>(output->size());
    output->height = 1;
  }
  return result;
}

}
