#include "point_lio_backend/pose_graph.hpp"

#include <ceres/ceres.h>
#include <ceres/product_manifold.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace point_lio_backend {
namespace {

// ── SE(3) 小工具（模板化，同时服务 autodiff 与 double 残差评估）──────────
template <typename T>
Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& v) {
  Eigen::Matrix<T, 3, 3> m;
  m << T(0), -v(2), v(1), v(2), T(0), -v(0), -v(1), v(0), T(0);
  return m;
}

// 四元数 -> 旋转向量（Log），自动处理 q 与 -q 的二义性及小角度情形。
template <typename T>
Eigen::Matrix<T, 3, 1> logRotation(const Eigen::Quaternion<T>& q_in) {
  using std::atan2;
  using std::sqrt;
  Eigen::Quaternion<T> q = q_in;
  q.normalize();
  Eigen::Matrix<T, 3, 1> v = q.vec();
  T w = q.w();
  if (w < T(0)) {  // 取最短弧
    v = -v;
    w = -w;
  }
  const T n = sqrt(v.squaredNorm());
  const T angle = T(2) * atan2(n, w);
  if (n > T(1e-10)) {
    return (angle / n) * v;
  }
  return T(2) * v;  // sin(theta/2) ~ theta/2
}

// SE(3) Log 的平移部分需要左 Jacobian 的逆：rho = V^{-1}(phi) * t。
template <typename T>
Eigen::Matrix<T, 3, 3> leftJacobianInverse(const Eigen::Matrix<T, 3, 1>& phi) {
  using std::cos;
  using std::sin;
  using std::sqrt;
  const T theta2 = phi.squaredNorm();
  const T theta = sqrt(theta2);
  const Eigen::Matrix<T, 3, 3> Phi = skew(phi);
  const Eigen::Matrix<T, 3, 3> Phi2 = Phi * Phi;
  T c;
  if (theta > T(1e-8)) {
    const T half = theta / T(2);
    c = (T(1) - half * cos(half) / sin(half)) / theta2;
  } else {
    c = T(1) / T(12);
  }
  return Eigen::Matrix<T, 3, 3>::Identity() - T(0.5) * Phi + c * Phi2;
}

// 残差：r = Log( (T_i^{-1} T_j)^{-1} * Z ) ∈ R^6。
// z 为 7 维 [tx,ty,tz,qx,qy,qz,qw]；residual 已按 sqrt 信息矩阵加权。
template <typename T>
void evaluateResidual(const T* pi, const T* pj, const double* z, const T& sqrt_trans,
                      const T& sqrt_rot, T* residual) {
  Eigen::Quaternion<T> qi(pi[6], pi[3], pi[4], pi[5]);  // w,x,y,z
  Eigen::Quaternion<T> qj(pj[6], pj[3], pj[4], pj[5]);
  qi.normalize();
  qj.normalize();
  const Eigen::Matrix<T, 3, 1> ti(pi[0], pi[1], pi[2]);
  const Eigen::Matrix<T, 3, 1> tj(pj[0], pj[1], pj[2]);

  Eigen::Quaternion<T> qz =
      Eigen::Quaternion<T>(T(z[6]), T(z[3]), T(z[4]), T(z[5]));
  qz.normalize();
  const Eigen::Matrix<T, 3, 1> tz =
      (Eigen::Matrix<T, 3, 1>() << T(z[0]), T(z[1]), T(z[2])).finished();

  // T_ij = T_i^{-1} * T_j
  const Eigen::Quaternion<T> qi_inv = qi.conjugate();
  const Eigen::Matrix<T, 3, 1> t_ij = qi_inv * (tj - ti);
  const Eigen::Quaternion<T> q_ij = qi_inv * qj;

  // E = Z^{-1} * T_ij
  const Eigen::Matrix<T, 3, 1> t_e = qz.conjugate() * (t_ij - tz);
  const Eigen::Quaternion<T> q_e = qz.conjugate() * q_ij;

  const Eigen::Matrix<T, 3, 1> phi = logRotation(q_e);
  const Eigen::Matrix<T, 3, 1> rho = leftJacobianInverse(phi) * t_e;
  for (int k = 0; k < 3; ++k) {
    residual[k] = sqrt_trans * rho(k);
    residual[3 + k] = sqrt_rot * phi(k);
  }
}

// 自写 autodiff cost functor：7 维位姿块 i、j 与 7 维测量 z。
class PoseErrorCost {
 public:
  PoseErrorCost(const double* z, double sqrt_trans, double sqrt_rot)
      : sqrt_trans_(sqrt_trans), sqrt_rot_(sqrt_rot) {
    for (int k = 0; k < 7; ++k) z_[k] = z[k];
  }

  template <typename T>
  bool operator()(const T* const pi, const T* const pj, T* residual) const {
    evaluateResidual<T>(pi, pj, z_, T(sqrt_trans_), T(sqrt_rot_), residual);
    return true;
  }

 private:
  double z_[7];
  double sqrt_trans_;
  double sqrt_rot_;
};

using PoseArray = Eigen::Matrix<double, 7, 1>;

PoseArray toParams(const Eigen::Isometry3d& pose) {
  PoseArray p;
  const Eigen::Quaterniond q(pose.linear());
  p << pose.translation().x(), pose.translation().y(), pose.translation().z(),
      q.x(), q.y(), q.z(), q.w();
  return p;
}

Eigen::Isometry3d fromParams(const double* p) {
  Eigen::Quaterniond q(p[6], p[3], p[4], p[5]);
  q.normalize();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() = Eigen::Vector3d(p[0], p[1], p[2]);
  return pose;
}

PoseArray toParams(const LoopConstraint& loop) { return toParams(loop.T_from_to); }

// 无权重残差（用于外点门控）：返回 [rho; phi]。
Eigen::Matrix<double, 6, 1> unweightedResidual(const double* pi, const double* pj,
                                                const double* z) {
  Eigen::Matrix<double, 6, 1> r;
  evaluateResidual<double>(pi, pj, z, 1.0, 1.0, r.data());
  return r;
}

bool isUsablePose(const Eigen::Isometry3d& pose) {
  return pose.matrix().allFinite();
}

struct GraphSolveResult {
  bool success = false;
  std::vector<Eigen::Isometry3d> poses;
};

// 用给定初值、活跃回环集合求解一次。成功时 poses 为优化结果。
GraphSolveResult solveOnce(const std::vector<Keyframe>& keyframes,
                           const std::vector<Eigen::Isometry3d>& initial,
                           const std::vector<LoopConstraint>& loops,
                           const std::vector<bool>& active,
                           const std::unordered_map<int, std::size_t>& id_to_index,
                           const GraphParams& params) {
  GraphSolveResult result;
  const std::size_t n = keyframes.size();
  if (n == 0) {
    result.success = true;
    return result;
  }

  std::vector<double> pose_data(7 * n);
  for (std::size_t i = 0; i < n; ++i) {
    const PoseArray p = toParams(initial[i]);
    for (int k = 0; k < 7; ++k) pose_data[7 * i + k] = p(k);
  }

  ceres::Problem problem;
  for (std::size_t i = 0; i < n; ++i) {
    double* block = pose_data.data() + 7 * i;
    // 7 维单块 = 平移 3 + 四元数 4；quaternion 的 [x,y,z,w] 内存布局。
    problem.AddParameterBlock(block, 7);
    problem.SetManifold(
        block, new ceres::ProductManifold<ceres::EuclideanManifold<3>,
                                          ceres::EigenQuaternionManifold>());
  }
  // 首关键帧固定，保证 map 原点 = 首关键帧。
  problem.SetParameterBlockConstant(pose_data.data());

  const double sqrt_odom_trans = 1.0 / params.odom_trans_sigma;
  const double sqrt_odom_rot = 1.0 / deg2rad(params.odom_rot_sigma_deg);
  const double sqrt_loop_trans = 1.0 / params.loop_trans_sigma;
  const double sqrt_loop_rot = 1.0 / deg2rad(params.loop_rot_sigma_deg);

  auto add_edge = [&](std::size_t i, std::size_t j, const Eigen::Isometry3d& z,
                      double sqrt_trans, double sqrt_rot, bool robust) {
    const PoseArray zp = toParams(z);
    auto* cost = new ceres::AutoDiffCostFunction<PoseErrorCost, 6, 7, 7>(
        new PoseErrorCost(zp.data(), sqrt_trans, sqrt_rot));
    ceres::LossFunction* loss = robust ? new ceres::HuberLoss(params.huber_delta) : nullptr;
    problem.AddResidualBlock(cost, loss, pose_data.data() + 7 * i,
                             pose_data.data() + 7 * j);
  };

  // 相邻关键帧 odom 边：Z = pose_raw_i^{-1} * pose_raw_{i+1}
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const Eigen::Isometry3d z = keyframes[i].pose_raw.inverse() * keyframes[i + 1].pose_raw;
    add_edge(i, i + 1, z, sqrt_odom_trans, sqrt_odom_rot, false);
  }

  // 回环边。
  for (std::size_t k = 0; k < loops.size(); ++k) {
    if (!active[k]) continue;
    const auto it_from = id_to_index.find(loops[k].from);
    const auto it_to = id_to_index.find(loops[k].to);
    if (it_from == id_to_index.end() || it_to == id_to_index.end()) continue;
    if (it_from->second == it_to->second) continue;
    add_edge(it_from->second, it_to->second, loops[k].T_from_to, sqrt_loop_trans,
             sqrt_loop_rot, true);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = params.max_iterations;
  options.num_threads = std::max(1, params.num_threads);
  options.function_tolerance = 1e-6;
  options.logging_type = ceres::SILENT;
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // 达到 max_num_iterations 但解可用（NO_CONVERGENCE）不算失败，只有
  // FAILURE/USER_FAILURE 才丢弃；随后再校验所有位姿有限。
  if (!summary.IsSolutionUsable()) return result;

  result.poses.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    result.poses[i] = fromParams(pose_data.data() + 7 * i);
    if (!isUsablePose(result.poses[i])) {
      result.poses.clear();
      return result;  // success = false，调用方保留旧轨迹
    }
  }
  result.success = true;
  return result;
}

}  // namespace

PoseGraph::PoseGraph(const GraphParams& params) : params_(params) {}

bool PoseGraph::optimize(std::vector<Keyframe>& keyframes,
                         const std::vector<LoopConstraint>& loops,
                         std::vector<bool>* loop_active) const {
  const std::size_t n = keyframes.size();

  // 活跃状态：输入 active 为初值，外点门控只会关闭不会重新打开。
  std::vector<bool> active(loops.size(), false);
  for (std::size_t k = 0; k < loops.size(); ++k) active[k] = loops[k].active;

  // id -> index，不假设 id 等于下标。
  std::unordered_map<int, std::size_t> id_to_index;
  id_to_index.reserve(n * 2);
  for (std::size_t i = 0; i < n; ++i) {
    id_to_index.emplace(keyframes[i].id, i);
  }
  // 端点不存在或自环的回环直接视为无效。
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

  // 初值：优先沿用上一版 pose_opt（增量优化更稳）。
  // 首次调用时 pose_opt 可能还是默认的单位阵，此时退回 pose_raw。
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

  // 求解 + 外点门控重优化（最多 2 轮）。
  // 多回环互相冲突时，一次删掉所有超限回环会把真回环一起误删（折中位姿让两者
  // 残差都超限）。所以每轮保留归一化残差最小的一条，只删其余超限者，重优化后
  // 再评估；只有一条超限时直接删它。最多 2 轮，与规格一致。
  GraphSolveResult current;
  std::vector<Eigen::Isometry3d> guess = initial;
  auto solve_current = [&]() -> bool {
    GraphSolveResult solved = solveOnce(keyframes, guess, loops, active, id_to_index, params_);
    if (!solved.success) return false;
    current = std::move(solved);
    guess = current.poses;
    return true;
  };
  if (!solve_current()) return false;

  if (params_.remove_outlier_loops) {
    for (int round = 0; round < 2; ++round) {
      std::vector<std::pair<double, std::size_t>> violators;  // {score, loop index}
      for (std::size_t k = 0; k < loops.size(); ++k) {
        if (!active[k]) continue;
        const std::size_t i = id_to_index.at(loops[k].from);
        const std::size_t j = id_to_index.at(loops[k].to);
        const PoseArray pi = toParams(current.poses[i]);
        const PoseArray pj = toParams(current.poses[j]);
        const PoseArray z = toParams(loops[k]);
        const Eigen::Matrix<double, 6, 1> r =
            unweightedResidual(pi.data(), pj.data(), z.data());
        const double trans_score = r.head<3>().norm() / params_.max_loop_residual_trans;
        const double rot_score =
            (r.tail<3>().norm() * 180.0 / M_PI) / params_.max_loop_residual_rot_deg;
        const double score = std::max(trans_score, rot_score);
        if (score > 1.0) violators.emplace_back(score, k);
      }
      if (violators.empty()) break;

      // 多条同时超限时保留残差最小的一条（更可能是真回环）。
      std::size_t keep = violators.size() > 1 ? std::size_t(-1) : 0;
      if (keep == std::size_t(-1)) {
        keep = 0;
        for (std::size_t v = 1; v < violators.size(); ++v) {
          if (violators[v].first < violators[keep].first) keep = v;
        }
      }
      for (std::size_t v = 0; v < violators.size(); ++v) {
        if (v == keep) continue;
        active[violators[v].second] = false;
      }
      if (!solve_current()) return false;  // 失败时保留上一版，不写回
    }
  }

  for (std::size_t i = 0; i < n; ++i) keyframes[i].pose_opt = current.poses[i];
  if (loop_active != nullptr) *loop_active = active;
  return true;
}

}  // namespace point_lio_backend