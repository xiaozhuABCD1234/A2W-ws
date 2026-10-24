#include "point_lio_backend/scan_context.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace point_lio_backend {

// TODO(M2): 按计划实现 20x60 Scan Context + ring key + 扇区移位距离。
ScanContext::ScanContext(const ScanContextParams& params) : params_(params) {}

Eigen::MatrixXd ScanContext::describe(const PointCloudT& cloud) const {
  (void)cloud;
  return Eigen::MatrixXd::Zero(params_.rings, params_.sectors);
}

Eigen::VectorXd ScanContext::ringKey(const Eigen::MatrixXd& descriptor) const {
  if (descriptor.rows() == 0) return Eigen::VectorXd();
  return descriptor.rowwise().mean();
}

double ScanContext::ringKeyDistance(const Eigen::VectorXd& a,
                                    const Eigen::VectorXd& b) const {
  if (a.size() == 0 || a.size() != b.size()) return std::numeric_limits<double>::max();
  return (a - b).cwiseAbs().mean();
}

double ScanContext::distance(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
                             int* best_shift) const {
  if (a.rows() != b.rows() || a.cols() != b.cols() || a.rows() == 0) {
    return std::numeric_limits<double>::max();
  }
  if (best_shift) *best_shift = 0;
  return std::numeric_limits<double>::max();
}

}  // namespace point_lio_backend
