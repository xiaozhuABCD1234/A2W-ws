#include "point_lio_backend/pcm.hpp"

#include <gtsam/geometry/Pose3.h>

#include <cmath>

namespace point_lio_backend {
namespace {

gtsam::Pose3 toGtsam(const Eigen::Isometry3d& T) {
  return gtsam::Pose3(gtsam::Rot3(T.linear()), gtsam::Point3(T.translation()));
}

// 可用回环的视图：测量 Z_ij 与端点下标。
struct LoopView {
  gtsam::Pose3 z;   // Z_ij = T_i^{-1} T_j
  std::size_t i = 0;
  std::size_t j = 0;
};

// 里程计链上的相对位姿（按下标，链边 = 相邻关键帧）。
gtsam::Pose3 odomBetween(const std::vector<Keyframe>& keyframes, std::size_t i, std::size_t j) {
  return toGtsam(keyframes[i].pose_raw).between(toGtsam(keyframes[j].pose_raw));
}

// 回环 a=(i→j) 与 b=(k→l) 的两两一致性：沿 i→j(回环 a)→k(里程)→l(回环 b)→i(里程)
// 拼合闭环，残差 e = Log(Z_a · O_jk · Z_b · O_li)，协方差按里程段边数线性累加。
// 返回马氏距离 d = e^T Σ^{-1} e（即卡方统计量）；元素非有限按 +inf（不一致）。
double pairChi2(const LoopView& a, const LoopView& b, const std::vector<Keyframe>& keyframes,
                const Eigen::Matrix<double, 6, 6>& sigma_loop,
                const Eigen::Matrix<double, 6, 6>& sigma_odom) {
  const gtsam::Pose3 o_jk = odomBetween(keyframes, a.j, b.i);
  const gtsam::Pose3 o_li = odomBetween(keyframes, b.j, a.i);
  const gtsam::Pose3 e_pose = a.z * o_jk * b.z * o_li;
  const Eigen::Matrix<double, 6, 1> e = gtsam::Pose3::Logmap(e_pose);

  // 里程段协方差按边数（下标差）线性累加，链可逆时近似一阶成立。
  const std::size_t n_jk = a.j > b.i ? a.j - b.i : b.i - a.j;
  const std::size_t n_li = b.j > a.i ? b.j - a.i : a.i - b.j;
  const Eigen::Matrix<double, 6, 6> sigma =
      sigma_loop + sigma_loop + static_cast<double>(n_jk) * sigma_odom +
      static_cast<double>(n_li) * sigma_odom;
  return e.dot(sigma.ldlt().solve(e));
}

bool finitePose(const Eigen::Isometry3d& T) { return T.matrix().allFinite(); }

}  // namespace

std::vector<bool> pcmFilter(const std::vector<Keyframe>& keyframes,
                            const std::unordered_map<int, std::size_t>& id_to_index,
                            const std::vector<LoopConstraint>& loops,
                            const std::vector<bool>& input_active,
                            const GraphParams& params) {
  const std::size_t k_count = loops.size();
  std::vector<bool> result = input_active;
  result.resize(k_count, false);

  // 结构与非有限校验：端点存在、非自环、测量与端点里程位姿均有限。
  std::vector<LoopView> views(k_count);
  for (std::size_t k = 0; k < k_count; ++k) {
    if (!result[k]) continue;
    const auto it_from = id_to_index.find(loops[k].from);
    const auto it_to = id_to_index.find(loops[k].to);
    if (it_from == id_to_index.end() || it_to == id_to_index.end() ||
        it_from->second == it_to->second) {
      result[k] = false;
      continue;
    }
    const LoopView view{toGtsam(loops[k].T_from_to), it_from->second, it_to->second};
    if (!loops[k].T_from_to.matrix().allFinite() ||
        !finitePose(keyframes[view.i].pose_raw) || !finitePose(keyframes[view.j].pose_raw)) {
      result[k] = false;
      continue;
    }
    views[k] = view;
  }

  if (!params.pcm_enabled) return result;

  const Eigen::Matrix<double, 6, 6> sigma_loop = makeInformation(params.loop_trans_sigma,
                                                                 deg2rad(params.loop_rot_sigma_deg))
                                                     .inverse();
  const Eigen::Matrix<double, 6, 6> sigma_odom = makeInformation(params.odom_trans_sigma,
                                                                 deg2rad(params.odom_rot_sigma_deg))
                                                     .inverse();

  // 贪心极大团：按插入顺序，与所有已接受回环两两一致（卡方门限内）才保留。
  std::vector<std::size_t> accepted;
  for (std::size_t k = 0; k < k_count; ++k) {
    if (!result[k]) continue;
    bool consistent = true;
    for (const std::size_t m : accepted) {
      if (pairChi2(views[k], views[m], keyframes, sigma_loop, sigma_odom) > params.pcm_chi2) {
        consistent = false;
        break;
      }
    }
    if (consistent) {
      accepted.push_back(k);
    } else {
      result[k] = false;
    }
  }
  return result;
}

}  // namespace point_lio_backend
