#include "ground_segmentation/linefit_segmenter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace ground_segmentation {
namespace {

struct GroundLine {
  double slope = 0.0;
  double intercept = 0.0;
  bool valid = false;
};

GroundLine fitLine(const std::vector<double>& radius, const std::vector<double>& height) {
  GroundLine line;
  const size_t n = radius.size();
  if (n == 0) return line;
  double sum_r = 0.0, sum_z = 0.0, sum_rr = 0.0, sum_rz = 0.0;
  for (size_t i = 0; i < n; ++i) {
    sum_r += radius[i];
    sum_z += height[i];
    sum_rr += radius[i] * radius[i];
    sum_rz += radius[i] * height[i];
  }
  const double count = static_cast<double>(n);
  const double denominator = count * sum_rr - sum_r * sum_r;
  if (std::abs(denominator) < 1e-9) {
    line.intercept = sum_z / count;
    line.valid = true;
    return line;
  }
  line.slope = (count * sum_rz - sum_r * sum_z) / denominator;
  line.intercept = (sum_z - line.slope * sum_r) / count;
  line.valid = true;
  return line;
}

}

LineFitGroundSegmenter::LineFitGroundSegmenter(const LineFitParams& params) : params_(params) {
  params_.ring_size = std::max(0.05, params_.ring_size);
  params_.max_range = std::max(params_.min_range + params_.ring_size, params_.max_range);
  params_.sectors = std::max(8, params_.sectors);
  params_.num_lpr = std::max(1, params_.num_lpr);
  params_.num_iter = std::max(1, params_.num_iter);
  params_.min_seed_points = std::max(2, params_.min_seed_points);
  params_.th_seeds = std::max(0.0, params_.th_seeds);
  params_.th_dist = std::max(0.01, params_.th_dist);
  params_.uprightness_thr = std::clamp(params_.uprightness_thr, 0.0, 1.0);
  params_.ground_bottom_percentile = std::clamp(params_.ground_bottom_percentile, 0.0, 1.0);
  max_slope_ = std::tan(std::clamp(params_.max_slope_deg, 0.0, 80.0) * M_PI / 180.0);
  rings_ = static_cast<int>(std::ceil((params_.max_range - params_.min_range) /
                                      params_.ring_size));
}

SegmentationResult LineFitGroundSegmenter::segment(
    const PointCloudT& cloud, const Eigen::Matrix3d& gravity_rotation) {
  SegmentationResult result;
  if (cloud.empty()) return result;

  const double two_pi = 2.0 * M_PI;
  const size_t bin_count = static_cast<size_t>(rings_ * params_.sectors);
  std::vector<Eigen::Vector3d> level_points(cloud.size(), Eigen::Vector3d::Zero());
  std::vector<double> radii(cloud.size(), 0.0);
  std::vector<std::vector<int>> bins(bin_count);
  std::vector<uint8_t> labels(cloud.size(), 255);

  for (size_t index = 0; index < cloud.size(); ++index) {
    const auto& point = cloud.points[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const Eigen::Vector3d p = gravity_rotation * Eigen::Vector3d(point.x, point.y, point.z);
    const double radius = std::hypot(p.x(), p.y());
    if (radius < params_.min_range || radius >= params_.max_range) continue;
    const int ring = static_cast<int>((radius - params_.min_range) / params_.ring_size);
    if (ring < 0 || ring >= rings_) continue;
    double angle = std::atan2(p.y(), p.x());
    if (angle < 0.0) angle += two_pi;
    int sector = static_cast<int>(angle / two_pi * params_.sectors);
    sector = std::clamp(sector, 0, params_.sectors - 1);
    level_points[index] = p;
    radii[index] = radius;
    bins[static_cast<size_t>(ring * params_.sectors + sector)].push_back(
        static_cast<int>(index));
    ++result.finite_points;
  }

  std::vector<double> minima;
  minima.reserve(bin_count);
  for (size_t b = 0; b < bin_count; ++b) {
    auto& bin = bins[b];
    if (bin.empty()) continue;
    std::sort(bin.begin(), bin.end(), [&](int lhs, int rhs) {
      return level_points[lhs].z() < level_points[rhs].z();
    });
    minima.push_back(level_points[bin.front()].z());
  }
  if (minima.empty()) return result;

  const size_t kth = static_cast<size_t>(
      std::clamp(params_.ground_bottom_percentile, 0.0, 1.0) * (minima.size() - 1));
  std::nth_element(minima.begin(), minima.begin() + kth, minima.end());
  const double global_bottom = minima[kth];

  const auto reject = [&labels](const std::vector<int>& bin) {
    for (int index : bin) labels[index] = 1;
  };

#pragma omp parallel for schedule(dynamic, 1) if(params_.sectors > 16)
  for (int sector = 0; sector < params_.sectors; ++sector) {
    bool has_ground = false;
    double previous_ground_z = global_bottom;
    int previous_ring = -1;
    for (int ring = 0; ring < rings_; ++ring) {
      const auto& bin = bins[static_cast<size_t>(ring * params_.sectors + sector)];
      if (bin.size() < 2) {
        reject(bin);
        continue;
      }
      const double gap = previous_ring >= 0
                             ? (ring - previous_ring) * params_.ring_size
                             : params_.ring_size;
      const double allowed_step = params_.max_ground_step + max_slope_ * gap;
      const double bin_minimum_z = level_points[bin.front()].z();

      if (!has_ground) {
        if (bin_minimum_z > global_bottom + params_.initial_ground_tolerance) {
          reject(bin);
          continue;
        }
      } else if (std::abs(bin_minimum_z - previous_ground_z) > allowed_step) {
        reject(bin);
        continue;
      }

      const int lpr_count = std::min<int>(params_.num_lpr, static_cast<int>(bin.size()));
      double lpr = 0.0;
      for (int i = 0; i < lpr_count; ++i) lpr += level_points[bin[i]].z();
      lpr /= static_cast<double>(lpr_count);

      std::vector<int> seeds;
      seeds.reserve(bin.size());
      for (int index : bin) {
        if (level_points[index].z() <= lpr + params_.th_seeds) seeds.push_back(index);
      }

      GroundLine line;
      for (int iter = 0; iter < params_.num_iter; ++iter) {
        if (static_cast<int>(seeds.size()) < params_.min_seed_points) {
          line.valid = false;
          break;
        }
        std::vector<double> seed_radius(seeds.size());
        std::vector<double> seed_height(seeds.size());
        for (size_t i = 0; i < seeds.size(); ++i) {
          seed_radius[i] = radii[seeds[i]];
          seed_height[i] = level_points[seeds[i]].z();
        }
        line = fitLine(seed_radius, seed_height);
        if (!line.valid || std::abs(line.slope) > max_slope_) {
          line.valid = false;
          break;
        }
        std::vector<int> inliers;
        inliers.reserve(bin.size());
        for (int index : bin) {
          const double residual =
              level_points[index].z() - (line.slope * radii[index] + line.intercept);
          if (std::abs(residual) <= params_.th_dist) inliers.push_back(index);
        }
        if (static_cast<int>(inliers.size()) < params_.min_seed_points) {
          line.valid = false;
          break;
        }
        seeds.swap(inliers);
      }
      if (line.valid) {
        std::vector<double> seed_radius(seeds.size());
        std::vector<double> seed_height(seeds.size());
        for (size_t i = 0; i < seeds.size(); ++i) {
          seed_radius[i] = radii[seeds[i]];
          seed_height[i] = level_points[seeds[i]].z();
        }
        const GroundLine refined = fitLine(seed_radius, seed_height);
        if (refined.valid && std::abs(refined.slope) <= max_slope_) line = refined;
      }

      const double uprightness = 1.0 / std::sqrt(1.0 + line.slope * line.slope);
      if (!line.valid || uprightness < params_.uprightness_thr) {
        reject(bin);
        continue;
      }

      const double center_radius = params_.min_range + (ring + 0.5) * params_.ring_size;
      const double line_z = line.slope * center_radius + line.intercept;
      if (!has_ground) {
        if (std::abs(line_z - global_bottom) > params_.initial_ground_tolerance) {
          reject(bin);
          continue;
        }
      } else if (std::abs(line_z - previous_ground_z) > allowed_step) {
        reject(bin);
        continue;
      }

      for (int index : bin) {
        const double residual =
            level_points[index].z() - (line.slope * radii[index] + line.intercept);
        labels[index] = std::abs(residual) <= params_.th_dist ? 0 : 1;
      }
      has_ground = true;
      previous_ground_z = line_z;
      previous_ring = ring;
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
