#include "point_lio_backend/pose_graph.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <unordered_map>
#include <vector>

#include "point_lio_backend/pcm.hpp"

namespace point_lio_backend {
namespace {

gtsam::Pose3 toGtsam(const Eigen::Isometry3d& T) {
  return gtsam::Pose3(gtsam::Rot3(T.linear()), gtsam::Point3(T.translation()));
}

Eigen::Isometry3d fromGtsam(const gtsam::Pose3& p) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = p.rotation().matrix();
  T.translation() = p.translation();
  return T;
}

// 关键帧 id 任意（不等于下标），用 Symbol 作图节点键。
gtsam::Key keyOf(int id) { return gtsam::Symbol('k', id); }

bool isUsablePose(const Eigen::Isometry3d& pose) {
  return pose.matrix().allFinite();
}

}  // namespace

PoseGraph::PoseGraph(const GraphParams& params) : params_(params) {}

bool PoseGraph::optimize(std::vector<Keyframe>& keyframes,
                         const std::vector<LoopConstraint>& loops,
                         std::vector<bool>* loop_active) const {
  const std::size_t n = keyframes.size();

  // 门控只会关闭 active，不会重新打开
  std::vector<bool> active(loops.size(), false);
  for (std::size_t k = 0; k < loops.size(); ++k) active[k] = loops[k].active;

  // id -> index（id 不等于下标）
  std::unordered_map<int, std::size_t> id_to_index;
  id_to_index.reserve(n * 2);
  for (std::size_t i = 0; i < n; ++i) {
    id_to_index.emplace(keyframes[i].id, i);
  }
  // 端点不存在或自环的回环无效
  for (std::size_t k = 0; k < loops.size(); ++k) {
    if (!active[k]) continue;
    const auto it_from = id_to_index.find(loops[k].from);
    const auto it_to = id_to_index.find(loops[k].to);
    if (it_from == id_to_index.end() || it_to == id_to_index.end() ||
        it_from->second == it_to->second) {
      active[k] = false;
    }
  }

  if (loop_active != nullptr) *loop_active = active;
  if (n == 0) return true;

  // 初值沿用上一版 pose_opt，首次调用退回 pose_raw
  bool all_opt_identity = true;
  bool any_raw_non_identity = false;
  for (const auto& kf : keyframes) {
    if (!kf.pose_opt.isApprox(Eigen::Isometry3d::Identity(), 1e-9)) all_opt_identity = false;
    if (!kf.pose_raw.isApprox(Eigen::Isometry3d::Identity(), 1e-9)) any_raw_non_identity = true;
  }
  const bool use_raw_as_initial = all_opt_identity && any_raw_non_identity;
  std::vector<Eigen::Isometry3d> initial(n);
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Isometry3d& opt = keyframes[i].pose_opt;
    if (!use_raw_as_initial && isUsablePose(opt)) {
      initial[i] = opt;
    } else {
      initial[i] = keyframes[i].pose_raw;
    }
  }

  if (n < 2) {
    for (std::size_t i = 0; i < n; ++i) keyframes[i].pose_opt = initial[i];
    return true;
  }

  // PCM-lite 一致性筛选：只依赖里程计与测量，纯预处理，替代残差门控。
  if (params_.pcm_enabled) {
    active = pcmFilter(keyframes, id_to_index, loops, active, params_);
    if (loop_active != nullptr) *loop_active = active;
  }

  // ── 构建图并 LM 求解（构建含 values.insert，重复 key 等异常统一按失败处理）──
  try {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    for (std::size_t i = 0; i < n; ++i) {
      values.insert(keyOf(keyframes[i].id), toGtsam(initial[i]));
    }

    // 首关键帧固定，保证 map 原点 = 首关键帧（初值即锚点值，无 inf 风险）。
    graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3>>(
        keyOf(keyframes[0].id), toGtsam(initial[0]));

    const auto odom_noise = gtsam::noiseModel::Gaussian::Information(
        makeInformation(params_.odom_trans_sigma, deg2rad(params_.odom_rot_sigma_deg)));
    const auto loop_base = gtsam::noiseModel::Gaussian::Information(
        makeInformation(params_.loop_trans_sigma, deg2rad(params_.loop_rot_sigma_deg)));
    // 鲁棒核作用于白化残差，huber_delta 语义与原 Ceres HuberLoss 一致。
    const auto loop_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(params_.huber_delta), loop_base);

    // 相邻关键帧 odom 边：Z = pose_raw_i^{-1} * pose_raw_{i+1}
    for (std::size_t i = 0; i + 1 < n; ++i) {
      const gtsam::Pose3 z(
          toGtsam(keyframes[i].pose_raw.inverse() * keyframes[i + 1].pose_raw));
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          keyOf(keyframes[i].id), keyOf(keyframes[i + 1].id), z, odom_noise);
    }

    // 回环边（仅 PCM 保留者）。
    for (std::size_t k = 0; k < loops.size(); ++k) {
      if (!active[k]) continue;
      const auto it_from = id_to_index.find(loops[k].from);
      const auto it_to = id_to_index.find(loops[k].to);
      if (it_from == id_to_index.end() || it_to == id_to_index.end()) continue;
      if (it_from->second == it_to->second) continue;
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          keyOf(loops[k].from), keyOf(loops[k].to), toGtsam(loops[k].T_from_to), loop_noise);
    }

    gtsam::LevenbergMarquardtParams lm;
    lm.setMaxIterations(static_cast<std::size_t>(std::max(1, params_.max_iterations)));
    lm.setRelativeErrorTol(1e-6);
    lm.setAbsoluteErrorTol(1e-6);
    lm.setVerbosityLM("SILENT");

    const gtsam::Values result =
        gtsam::LevenbergMarquardtOptimizer(graph, values, lm).optimize();

    for (std::size_t i = 0; i < n; ++i) {
      const Eigen::Isometry3d pose =
          fromGtsam(result.at<gtsam::Pose3>(keyOf(keyframes[i].id)));
      if (!isUsablePose(pose)) {
        return false;  // 有限性校验失败，保留上一版
      }
      keyframes[i].pose_opt = pose;
    }
  } catch (const std::exception&) {
    return false;  // 契约：失败时调用方保留旧解，不写回
  }

  if (loop_active != nullptr) *loop_active = active;
  return true;
}

}  // namespace point_lio_backend
