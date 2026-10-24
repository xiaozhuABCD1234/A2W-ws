#include "point_lio_backend/loop_detector.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace point_lio_backend {

namespace {

constexpr double kTwoPi = 2.0 * M_PI;

double wrapAngle(double angle) {
  while (angle > M_PI) {
    angle -= kTwoPi;
  }
  while (angle < -M_PI) {
    angle += kTwoPi;
  }
  return angle;
}

double yawOf(const Eigen::Matrix3d& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

bool validCloud(const PointCloudT::Ptr& cloud) {
  return cloud && !cloud->points.empty();
}

Eigen::VectorXd effectiveRingKey(const ScanContext& scan_context, const Keyframe& keyframe,
                                 int expected_size) {
  if (keyframe.ring_key.size() == expected_size) {
    return keyframe.ring_key;
  }
  if (keyframe.descriptor.rows() > 0) {
    return scan_context.ringKey(keyframe.descriptor);
  }
  if (validCloud(keyframe.cloud)) {
    return scan_context.ringKey(scan_context.describe(*keyframe.cloud));
  }
  return Eigen::VectorXd();
}

// 回环匹配专用降采样：GICP 的耗时≈点数×候选数×迭代数，全分辨率子图太贵。
// 只影响匹配，不影响 keyframe.cloud（地图与输出仍是原分辨率）。
PointCloudT::Ptr downsampleForMatching(const PointCloudT::ConstPtr& cloud,
                                       const IcpParams& params) {
  if (!cloud || cloud->empty()) {
    return PointCloudT::Ptr(new PointCloudT());
  }
  PointCloudT::Ptr filtered(new PointCloudT());
  if (params.match_voxel > 0.0) {
    const float leaf = static_cast<float>(params.match_voxel);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(cloud);
    voxel.filter(*filtered);
  } else {
    *filtered = *cloud;
  }
  // 极端情况下体素没压下来（子图特别大），再按步长截断。
  const int max_points = std::max(1, params.max_match_points);
  if (static_cast<int>(filtered->size()) > max_points) {
    PointCloudT::Ptr thinned(new PointCloudT());
    thinned->reserve(static_cast<size_t>(max_points));
    const double stride =
        static_cast<double>(filtered->size()) / static_cast<double>(max_points);
    for (double idx = 0.0; idx < static_cast<double>(filtered->size()); idx += stride) {
      thinned->push_back(filtered->points[static_cast<size_t>(idx)]);
    }
    thinned->width = static_cast<uint32_t>(thinned->size());
    thinned->height = 1;
    return thinned;
  }
  filtered->width = static_cast<uint32_t>(filtered->size());
  filtered->height = 1;
  return filtered;
}

Eigen::MatrixXd effectiveDescriptor(const ScanContext& scan_context, const Keyframe& keyframe) {
  if (keyframe.descriptor.rows() > 0) {
    return keyframe.descriptor;
  }
  if (validCloud(keyframe.cloud)) {
    return scan_context.describe(*keyframe.cloud);
  }
  return Eigen::MatrixXd();
}

}  // namespace

LoopDetector::LoopDetector(const ScanContextParams& sc_params, const IcpParams& icp_params)
    : scan_context_(sc_params), sc_params_(sc_params), icp_params_(icp_params) {}

std::optional<LoopConstraint> LoopDetector::detect(
    const Keyframe& current, const std::vector<Keyframe>& history) const {
  if (!validCloud(current.cloud)) {
    return std::nullopt;
  }

  const Eigen::VectorXd current_ring_key =
      effectiveRingKey(scan_context_, current, sc_params_.rings);
  if (current_ring_key.size() == 0) {
    return std::nullopt;
  }
  const Eigen::MatrixXd current_descriptor = effectiveDescriptor(scan_context_, current);
  if (current_descriptor.rows() == 0) {
    return std::nullopt;
  }

  struct Candidate {
    size_t history_index = 0;
    double ring_key_distance = std::numeric_limits<double>::max();
    double sc_distance = std::numeric_limits<double>::max();
  };
  std::vector<Candidate> candidates;
  candidates.reserve(history.size());

  for (size_t i = 0; i < history.size(); ++i) {
    const Keyframe& keyframe = history[i];
    if (!validCloud(keyframe.cloud)) {
      continue;
    }

    // 索引差 / 时间差过滤，避免与相邻帧或刚看过的帧配对。
    if (current.id >= 0 && keyframe.id >= 0) {
      if (std::abs(current.id - keyframe.id) < std::max(0, sc_params_.min_index_gap)) {
        continue;
      }
    }
    if (std::abs(current.stamp - keyframe.stamp) < sc_params_.min_time_gap_sec) {
      continue;
    }

    const Eigen::VectorXd history_ring_key =
        effectiveRingKey(scan_context_, keyframe, sc_params_.rings);
    const double ring_distance =
        scan_context_.ringKeyDistance(current_ring_key, history_ring_key);
    if (ring_distance > sc_params_.ring_key_threshold) {
      continue;
    }
    candidates.push_back(Candidate{i, ring_distance});
  }

  const int top_candidates = std::max(1, sc_params_.top_candidates);
  if (static_cast<int>(candidates.size()) > top_candidates) {
    std::partial_sort(candidates.begin(), candidates.begin() + top_candidates, candidates.end(),
                      [](const Candidate& lhs, const Candidate& rhs) {
                        return lhs.ring_key_distance < rhs.ring_key_distance;
                      });
    candidates.resize(static_cast<size_t>(top_candidates));
  } else {
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& lhs, const Candidate& rhs) {
                return lhs.ring_key_distance < rhs.ring_key_distance;
              });
  }

  // ── 第二阶段：SC 精筛 ────────────────────────────────────────────────
  // 先算完所有候选的 SC 距离再排序：GICP 很贵，必须让最像的候选先试，
  // 配合 stop_at_first 大多数关键帧只跑 0~1 次 GICP。
  struct Verified {
    size_t history_index = 0;
    double sc_distance = std::numeric_limits<double>::max();
    int shift = 0;
  };
  std::vector<Verified> verified;
  verified.reserve(candidates.size());
  for (const Candidate& candidate : candidates) {
    const Eigen::MatrixXd history_descriptor =
        effectiveDescriptor(scan_context_, history[candidate.history_index]);
    if (history_descriptor.rows() == 0) {
      continue;
    }
    int best_shift = 0;
    const double sc_distance =
        scan_context_.distance(current_descriptor, history_descriptor, &best_shift);
    if (sc_distance > sc_params_.sc_distance_threshold) {
      continue;
    }
    verified.push_back(Verified{candidate.history_index, sc_distance, best_shift});
  }
  std::sort(verified.begin(), verified.end(),
            [](const Verified& lhs, const Verified& rhs) {
              return lhs.sc_distance < rhs.sc_distance;
            });

  std::optional<LoopConstraint> best;
  double best_fitness = std::numeric_limits<double>::max();
  if (verified.empty()) {
    return best;
  }

  // 匹配用点云只降采样一次（当前帧），历史帧按需降采样。
  const PointCloudT::Ptr current_match = downsampleForMatching(current.cloud, icp_params_);
  const auto detect_start = std::chrono::steady_clock::now();
  const double budget_ms = icp_params_.time_budget_ms;

  for (const Verified& candidate : verified) {
    // 预算控制：坏候选可能让 GICP 迭代满也不收敛，必须在循环里判超时，
    // 否则一次检测就能把 worker 卡住 1 s 以上。
    if (budget_ms > 0.0) {
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - detect_start).count();
      if (elapsed_ms > budget_ms) {
        break;
      }
    }
    const Keyframe& keyframe = history[candidate.history_index];
    const int best_shift = candidate.shift;

    // 初值：前端位姿推出的相对位姿 history^-1 * current。
    Eigen::Isometry3d initial = keyframe.pose_opt.inverse() * current.pose_opt;
    const int sectors = static_cast<int>(current_descriptor.cols());
    if (sectors > 0) {
      // a.col(j) 对齐 b.col(j+shift)，对应相对航向 shift * 360/sectors。
      const double sc_yaw =
          wrapAngle(static_cast<double>(best_shift) * kTwoPi / static_cast<double>(sectors));
      const double yaw_correction = wrapAngle(sc_yaw - yawOf(initial.linear()));
      const Eigen::Matrix3d z_rotation =
          Eigen::AngleAxisd(yaw_correction, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      initial.linear() = z_rotation * initial.linear();
    }

    const PointCloudT::Ptr target_match =
        downsampleForMatching(keyframe.cloud, icp_params_);
    if (!current_match || current_match->size() < 10 ||
        !target_match || target_match->size() < 10) {
      continue;
    }

    // 两级 GICP：先用较大的对应距离粗配准吸收累积漂移，再精细配准。
    const double fine_correspondence = icp_params_.max_correspondence;
    const double coarse_correspondence =
        std::max(icp_params_.coarse_correspondence, fine_correspondence);
    Eigen::Isometry3d fine_initial = initial;

    if (coarse_correspondence > fine_correspondence) {
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> coarse_gicp;
      coarse_gicp.setInputSource(current_match);
      coarse_gicp.setInputTarget(target_match);
      coarse_gicp.setMaximumIterations(std::max(1, icp_params_.coarse_iterations));
      coarse_gicp.setMaxCorrespondenceDistance(coarse_correspondence);

      PointCloudT coarse_aligned;
      coarse_gicp.align(coarse_aligned, initial.matrix().cast<float>());
      if (!coarse_gicp.hasConverged() || coarse_aligned.points.empty()) {
        continue;
      }
      fine_initial.matrix() = coarse_gicp.getFinalTransformation().cast<double>();
    }

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(current_match);
    gicp.setInputTarget(target_match);
    gicp.setMaximumIterations(std::max(1, icp_params_.max_iterations));
    gicp.setMaxCorrespondenceDistance(fine_correspondence);

    PointCloudT aligned;
    gicp.align(aligned, fine_initial.matrix().cast<float>());
    if (!gicp.hasConverged() || aligned.points.empty()) {
      continue;
    }

    Eigen::Isometry3d final = Eigen::Isometry3d::Identity();
    final.matrix() = gicp.getFinalTransformation().cast<double>();

    // fitness / 内点率 / 修正量门限都只在精配准之后判定。
    const double fitness = gicp.getFitnessScore();
    if (!std::isfinite(fitness) || fitness > icp_params_.max_fitness) {
      continue;
    }

    // 内点率：变换后的 source 点落入 target 最近邻 max_correspondence 内的比例。
    PointCloudT transformed;
    pcl::transformPointCloud(*current_match, transformed, final.matrix().cast<float>());
    pcl::KdTreeFLANN<PointT> target_tree;
    target_tree.setInputCloud(target_match);
    const double max_sq = icp_params_.max_correspondence * icp_params_.max_correspondence;
    size_t inliers = 0;
    std::vector<int> nn_index(1);
    std::vector<float> nn_distance(1);
    for (const auto& point : transformed.points) {
      if (target_tree.nearestKSearch(point, 1, nn_index, nn_distance) == 1 &&
          static_cast<double>(nn_distance[0]) <= max_sq) {
        ++inliers;
      }
    }
    const double inlier_ratio =
        static_cast<double>(inliers) / static_cast<double>(transformed.points.size());
    if (inlier_ratio < icp_params_.min_inlier_ratio) {
      continue;
    }

    // 相对初值的修正量门限，防止 GICP 把错误候选硬拉到局部极小。
    const Eigen::Isometry3d correction = final * initial.inverse();
    const double correction_trans = correction.translation().norm();
    const double correction_rot_deg =
        Eigen::AngleAxisd(correction.linear()).angle() * 180.0 / M_PI;
    if (correction_trans > icp_params_.max_correction_trans ||
        correction_rot_deg > icp_params_.max_correction_rot_deg) {
      continue;
    }

    if (fitness < best_fitness) {
      best_fitness = fitness;
      LoopConstraint constraint;
      constraint.from = keyframe.id;
      constraint.to = current.id;
      constraint.T_from_to = final;
      constraint.fitness = fitness;
      constraint.inlier_ratio = inlier_ratio;
      constraint.active = true;
      best = constraint;
      // 候选已按 SC 距离升序：第一个通过全部校验的通常就是正确回环。
      // 后面再跑 GICP 只会增加延迟（实测 8 候选全跑要 0.7~3.3 s/关键帧）。
      if (icp_params_.stop_at_first) {
        return best;
      }
    }
  }

  return best;
}

}  // namespace point_lio_backend
