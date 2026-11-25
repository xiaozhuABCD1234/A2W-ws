#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

#include "point_lio_backend/pcm.hpp"

namespace point_lio_backend {
namespace {

using Iso = Eigen::Isometry3d;

Iso makePose(double x, double y, double yaw) {
  Iso pose = Iso::Identity();
  pose.translation() = Eigen::Vector3d(x, y, 0.0);
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

// n 个关键帧沿 +x 直线等距分布；odom 默认无漂移，隔离 PCM 逻辑与漂移影响。
std::vector<Keyframe> makeKeyframes(int n, double spacing = 1.0) {
  std::vector<Keyframe> keyframes(n);
  for (int i = 0; i < n; ++i) {
    keyframes[i].id = i;
    keyframes[i].stamp = static_cast<double>(i);
    keyframes[i].pose_raw = makePose(spacing * static_cast<double>(i), 0.0, 0.0);
    keyframes[i].pose_opt = Iso::Identity();
  }
  return keyframes;
}

// 每段平移乘 scale 的 odom（制造系统性漂移）。
std::vector<Keyframe> makeDriftedKeyframes(int n, double scale, double spacing = 1.0) {
  std::vector<Keyframe> keyframes = makeKeyframes(n, spacing);
  Iso raw = Iso::Identity();
  for (int i = 0; i < n; ++i) {
    if (i > 0) {
      Iso rel = Iso::Identity();
      rel.translation() = Eigen::Vector3d(spacing * scale, 0.0, 0.0);
      raw = raw * rel;
    }
    keyframes[i].pose_raw = raw;
  }
  return keyframes;
}

std::unordered_map<int, std::size_t> makeIdIndex(const std::vector<Keyframe>& keyframes) {
  std::unordered_map<int, std::size_t> id_to_index;
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    id_to_index.emplace(keyframes[i].id, i);
  }
  return id_to_index;
}

LoopConstraint makeLoop(int from, int to, const Iso& T_from_to, double offset_x = 0.0) {
  LoopConstraint loop;
  loop.from = from;
  loop.to = to;
  loop.T_from_to = T_from_to;
  loop.T_from_to.translation().x() += offset_x;
  loop.fitness = 0.05;
  loop.inlier_ratio = 0.9;
  loop.active = true;
  return loop;
}

// 前置条件：全部 active 的输入标记。
std::vector<bool> allActive(std::size_t k) { return std::vector<bool>(k, true); }

TEST(Pcm, ConsistentLoopsBothKept) {
  const std::vector<Keyframe> keyframes = makeKeyframes(12);
  const auto id_to_index = makeIdIndex(keyframes);

  const Iso z_0_11 = keyframes[0].pose_raw.inverse() * keyframes[11].pose_raw;
  const Iso z_3_8 = keyframes[3].pose_raw.inverse() * keyframes[8].pose_raw;
  std::vector<LoopConstraint> loops = {makeLoop(0, 11, z_0_11), makeLoop(3, 8, z_3_8)};

  const std::vector<bool> active =
      pcmFilter(keyframes, id_to_index, loops, allActive(loops.size()), GraphParams{});
  ASSERT_EQ(active.size(), 2u);
  EXPECT_TRUE(active[0]);
  EXPECT_TRUE(active[1]);
}

TEST(Pcm, OutlierLoopGated) {
  const std::vector<Keyframe> keyframes = makeKeyframes(12);
  const auto id_to_index = makeIdIndex(keyframes);

  const Iso z_0_11 = keyframes[0].pose_raw.inverse() * keyframes[11].pose_raw;
  const Iso z_3_8 = keyframes[3].pose_raw.inverse() * keyframes[8].pose_raw;
  // 两条一致回环 + 一条偏移 3 m 的重复回环（与真回环同端点）。
  std::vector<LoopConstraint> loops = {makeLoop(0, 11, z_0_11), makeLoop(3, 8, z_3_8),
                                       makeLoop(0, 11, z_0_11, 3.0)};

  const std::vector<bool> active =
      pcmFilter(keyframes, id_to_index, loops, allActive(loops.size()), GraphParams{});
  ASSERT_EQ(active.size(), 3u);
  EXPECT_TRUE(active[0]);   // 真回环保留
  EXPECT_TRUE(active[1]);   // 一致回环保留
  EXPECT_FALSE(active[2]);  // 偏移回环被剔除
}

TEST(Pcm, SingleLoopKeptDespiteOdomDrift) {
  // 里程计系统性漂移（每段平移 ×1.3），单条真回环对里程计偏差极大，
  // 但 PCM 只做回环-回环检查，绝不能因里程计漂移误杀真回环。
  const std::vector<Keyframe> keyframes = makeDriftedKeyframes(12, 1.3);
  const auto id_to_index = makeIdIndex(keyframes);

  // 测量取"真值"直线的相对位姿（与漂移 odom 相差巨大）。
  const Iso z_0_11 = makePose(0, 0, 0).inverse() * makePose(11, 0, 0);
  std::vector<LoopConstraint> loops = {makeLoop(0, 11, z_0_11)};

  const std::vector<bool> active =
      pcmFilter(keyframes, id_to_index, loops, allActive(loops.size()), GraphParams{});
  ASSERT_EQ(active.size(), 1u);
  EXPECT_TRUE(active[0]);
}

TEST(Pcm, DisabledReturnsStructuralFilterOnly) {
  const std::vector<Keyframe> keyframes = makeKeyframes(12);
  const auto id_to_index = makeIdIndex(keyframes);

  const Iso z_0_11 = keyframes[0].pose_raw.inverse() * keyframes[11].pose_raw;
  GraphParams params;
  params.pcm_enabled = false;
  // 偏移 3 m 的错误回环在 PCM 关闭时不被剔除（只做结构剪枝）。
  std::vector<LoopConstraint> loops = {makeLoop(0, 11, z_0_11), makeLoop(0, 11, z_0_11, 3.0),
                                       makeLoop(0, 999, z_0_11)};

  const std::vector<bool> active =
      pcmFilter(keyframes, id_to_index, loops, allActive(loops.size()), params);
  ASSERT_EQ(active.size(), 3u);
  EXPECT_TRUE(active[0]);
  EXPECT_TRUE(active[1]);   // PCM 关闭：外点保留
  EXPECT_FALSE(active[2]);  // 端点不存在：结构剪枝仍生效
}

TEST(Pcm, NonFiniteOrSelfLoopGated) {
  std::vector<Keyframe> keyframes = makeKeyframes(12);
  keyframes[5].pose_raw.translation().y() = std::numeric_limits<double>::quiet_NaN();
  const auto id_to_index = makeIdIndex(keyframes);

  const Iso z_0_11 = makePose(0, 0, 0).inverse() * makePose(11, 0, 0);
  LoopConstraint nan_loop = makeLoop(0, 11, z_0_11);
  nan_loop.T_from_to.translation().x() = std::numeric_limits<double>::quiet_NaN();

  std::vector<LoopConstraint> loops = {makeLoop(0, 11, z_0_11), nan_loop,
                                       makeLoop(5, 5, z_0_11), makeLoop(4, 5, z_0_11)};
  const std::vector<bool> active =
      pcmFilter(keyframes, id_to_index, loops, allActive(loops.size()), GraphParams{});
  ASSERT_EQ(active.size(), 4u);
  EXPECT_TRUE(active[0]);   // 基准回环
  EXPECT_FALSE(active[1]);  // 测量含 NaN
  EXPECT_FALSE(active[2]);  // 自环
  EXPECT_FALSE(active[3]);  // 端点里程位姿含 NaN
}

}  // namespace
}  // namespace point_lio_backend
