#include "point_lio_backend/scan_context.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace point_lio_backend {

namespace {

constexpr double kTwoPi = 2.0 * M_PI;

}  // namespace

ScanContext::ScanContext(const ScanContextParams& params) : params_(params) {}

Eigen::MatrixXd ScanContext::describe(const PointCloudT& cloud) const {
  const int rings = std::max(1, params_.rings);
  const int sectors = std::max(1, params_.sectors);

  Eigen::MatrixXd descriptor =
      Eigen::MatrixXd::Constant(rings, sectors, std::numeric_limits<double>::lowest());
  if (!(params_.max_radius > 0.0)) {
    descriptor.setZero();
    return descriptor;
  }

  const double sector_rad = kTwoPi / static_cast<double>(sectors);
  const double inv_ring_width = static_cast<double>(rings) / params_.max_radius;

  for (const auto& p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const double radius = std::hypot(static_cast<double>(p.x), static_cast<double>(p.y));
    if (radius > params_.max_radius) {
      continue;
    }

    int ring = static_cast<int>(std::floor(radius * inv_ring_width));
    ring = std::min(std::max(ring, 0), rings - 1);

    double angle = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
    if (angle < 0.0) {
      angle += kTwoPi;
    }
    int sector = static_cast<int>(std::floor(angle / sector_rad));
    sector %= sectors;
    if (sector < 0) {
      sector += sectors;
    }

    double& cell = descriptor(ring, sector);
    cell = std::max(cell, static_cast<double>(p.z));
  }

  // 无点的网格填 0；有点的网格保留其 max z（允许负值，以区分地面点）。
  const double empty = std::numeric_limits<double>::lowest();
  for (int r = 0; r < rings; ++r) {
    for (int c = 0; c < sectors; ++c) {
      if (descriptor(r, c) == empty) {
        descriptor(r, c) = 0.0;
      }
    }
  }
  return descriptor;
}

Eigen::VectorXd ScanContext::ringKey(const Eigen::MatrixXd& descriptor) const {
  if (descriptor.rows() == 0) {
    return Eigen::VectorXd();
  }
  return descriptor.rowwise().mean();
}

double ScanContext::ringKeyDistance(const Eigen::VectorXd& a,
                                    const Eigen::VectorXd& b) const {
  if (a.size() == 0 || a.size() != b.size()) {
    return std::numeric_limits<double>::max();
  }
  const double distance = (a - b).cwiseAbs().sum() / static_cast<double>(a.size());
  if (!std::isfinite(distance)) {
    return std::numeric_limits<double>::max();
  }
  return distance;
}

double ScanContext::distance(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
                             int* best_shift) const {
  if (best_shift) {
    *best_shift = 0;
  }
  if (a.rows() == 0 || b.rows() == 0 || a.rows() != b.rows() || a.cols() != b.cols()) {
    return std::numeric_limits<double>::max();
  }

  const int rings = static_cast<int>(a.rows());
  const int sectors = static_cast<int>(a.cols());
  if (sectors == 0) {
    return std::numeric_limits<double>::max();
  }

  double best_distance = std::numeric_limits<double>::max();
  int best_shift_value = 0;
  bool found = false;

  for (int shift = 0; shift < sectors; ++shift) {
    double row_distance_sum = 0.0;
    int valid_rows = 0;

    for (int r = 0; r < rings; ++r) {
      double dot = 0.0;
      double norm_a = 0.0;
      double norm_b = 0.0;
      for (int c = 0; c < sectors; ++c) {
        const double av = a(r, c);
        const double bv = b(r, (c + shift) % sectors);
        dot += av * bv;
        norm_a += av * av;
        norm_b += bv * bv;
      }
      if (norm_a <= 0.0 || norm_b <= 0.0) {
        continue;  // 跳过空环
      }
      double cosine = dot / (std::sqrt(norm_a) * std::sqrt(norm_b));
      cosine = std::min(1.0, std::max(-1.0, cosine));
      row_distance_sum += 1.0 - cosine;
      ++valid_rows;
    }

    if (valid_rows == 0) {
      continue;
    }
    const double distance = row_distance_sum / static_cast<double>(valid_rows);
    if (distance < best_distance) {
      best_distance = distance;
      best_shift_value = shift;
      found = true;
    }
  }

  if (!found) {
    return std::numeric_limits<double>::max();
  }
  if (best_shift) {
    *best_shift = best_shift_value;
  }
  return best_distance;
}

}  // namespace point_lio_backend
