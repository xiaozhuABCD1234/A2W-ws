#pragma once

#include <Eigen/Core>

#include "point_lio_backend/types.hpp"

namespace point_lio_backend {

// Scan Context 描述子（Kim & Kim 2018 的简化实现，按论文自研，不拷贝第三方代码）。
class ScanContext {
 public:
  explicit ScanContext(const ScanContextParams& params);

  // rings x sectors，每格取最大 z（空网格填 0）；输入为机体系点云。
  Eigen::MatrixXd describe(const PointCloudT& cloud) const;

  // 每环均值，长度 rings。
  Eigen::VectorXd ringKey(const Eigen::MatrixXd& descriptor) const;

  // 归一化环键距离（逐维相对差均值）。
  double ringKeyDistance(const Eigen::VectorXd& a, const Eigen::VectorXd& b) const;

  // 扇区循环移位后的最小余弦距离；best_shift 返回最优移位（列数）。
  double distance(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
                  int* best_shift = nullptr) const;

 private:
  ScanContextParams params_;
};

}  // namespace point_lio_backend
