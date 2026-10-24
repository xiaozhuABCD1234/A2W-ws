#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "point_lio_backend/pose_graph.hpp"

namespace point_lio_backend {
namespace {

using Iso = Eigen::Isometry3d;

Iso makePose(double x, double y, double yaw) {
  Iso pose = Iso::Identity();
  pose.translation() = Eigen::Vector3d(x, y, 0.0);
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

// 沿 w x h 矩形闭环的位姿，从原点沿 +x 出发；s ∈ [0, 2(w+h)] 为弧长。
Iso rectanglePose(double w, double h, double s) {
  const double perimeter = 2.0 * (w + h);
  s = std::fmod(s, perimeter);
  if (s < w) return makePose(s, 0.0, 0.0);
  if (s < w + h) return makePose(w, s - w, M_PI / 2);
  if (s < 2.0 * w + h) return makePose(w - (s - w - h), h, M_PI);
  return makePose(0.0, h - (s - 2.0 * w - h), -M_PI / 2);
}

// n 个关键帧沿矩形等弧长分布；第 n-1 个恰好回到起点，构成闭环。
std::vector<Iso> rectangleTrajectory(int n, double w, double h) {
  std::vector<Iso> trajectory(n);
  const double perimeter = 2.0 * (w + h);
  for (int i = 0; i < n; ++i) {
    trajectory[i] = rectanglePose(w, h, perimeter * static_cast<double>(i) /
                                               static_cast<double>(n - 1));
  }
  return trajectory;
}

// 用真实相对位姿积分出带漂移的 odom 轨迹：
// 每段平移乘 1.05（5% 尺度误差），并绕局部 z 轴加 0.5°/m 航向偏置。
std::vector<Iso> integrateDriftedOdom(const std::vector<Iso>& truth) {
  std::vector<Iso> raw(truth.size());
  raw[0] = truth[0];
  for (std::size_t i = 1; i < truth.size(); ++i) {
    const Iso rel = truth[i - 1].inverse() * truth[i];
    const double seg = rel.translation().norm();
    Iso noisy = rel;
    noisy.translation() *= 1.05;
    const double yaw_bias = deg2rad(0.5) * seg;
    noisy.linear() =
        rel.linear() * Eigen::AngleAxisd(yaw_bias, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    raw[i] = raw[i - 1] * noisy;
  }
  return raw;
}

std::vector<Keyframe> makeKeyframes(const std::vector<Iso>& raw) {
  std::vector<Keyframe> keyframes(raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i) {
    keyframes[i].id = static_cast<int>(i);
    keyframes[i].stamp = static_cast<double>(i);
    keyframes[i].pose_raw = raw[i];
    keyframes[i].pose_opt = Iso::Identity();  // 尚未优化
  }
  return keyframes;
}

double translationError(const Iso& a, const Iso& b) {
  return (a.translation() - b.translation()).norm();
}

double ate(const std::vector<Keyframe>& keyframes, const std::vector<Iso>& truth) {
  double sum = 0.0;
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    const double e = translationError(keyframes[i].pose_opt, truth[i]);
    sum += e * e;
  }
  return std::sqrt(sum / static_cast<double>(keyframes.size()));
}

double rawAte(const std::vector<Keyframe>& keyframes, const std::vector<Iso>& truth) {
  double sum = 0.0;
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    const double e = translationError(keyframes[i].pose_raw, truth[i]);
    sum += e * e;
  }
  return std::sqrt(sum / static_cast<double>(keyframes.size()));
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

// 合成单回环场景专用参数。
// 生产配置（config/backend_mid360.yaml）里一个回环周期会累积多条回环边；
// 本测试只有 1~2 条回环边，要闭合 9x10 m 矩形上米级的初始回环误差，
// 需要让回环权重与 odom 同量级，并放大 Huber 尺度，避免真回环被当成外点降权。
GraphParams syntheticGraphParams() {
  GraphParams params;
  params.odom_trans_sigma = 0.02;
  params.odom_rot_sigma_deg = 0.5;
  params.loop_trans_sigma = 0.02;
  params.loop_rot_sigma_deg = 0.5;
  params.huber_delta = 10.0;
  params.max_iterations = 200;
  return params;
}

}  // namespace

TEST(PoseGraph, DriftClosedByTrueLoop) {
  const std::vector<Iso> truth = rectangleTrajectory(12, 9.0, 10.0);
  const std::vector<Iso> raw = integrateDriftedOdom(truth);
  std::vector<Keyframe> keyframes = makeKeyframes(raw);

  const double raw_ate = rawAte(keyframes, truth);
  const double raw_end = translationError(keyframes.back().pose_raw, truth.back());

  // 真实回环 0 -> 11，T 用真实相对位姿并加小噪声。
  Iso loop_measure = truth[0].inverse() * truth[11];
  loop_measure.translation() += Eigen::Vector3d(0.01, -0.01, 0.0);
  loop_measure.linear() =
      loop_measure.linear() *
      Eigen::AngleAxisd(deg2rad(0.1), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const std::vector<LoopConstraint> loops = {makeLoop(0, 11, loop_measure)};

  PoseGraph graph(syntheticGraphParams());
  std::vector<bool> loop_active;
  ASSERT_TRUE(graph.optimize(keyframes, loops, &loop_active));

  const double opt_ate = ate(keyframes, truth);
  const double opt_end = translationError(keyframes.back().pose_opt, truth.back());
  std::cout << "[graph] raw ATE=" << raw_ate << " m, raw end=" << raw_end
            << " m -> opt ATE=" << opt_ate << " m, opt end=" << opt_end << " m ("
            << (1.0 - opt_ate / raw_ate) * 100.0 << "% ATE reduction)" << std::endl;

  ASSERT_EQ(loop_active.size(), 1u);
  EXPECT_TRUE(loop_active[0]);
  EXPECT_LT(opt_end, 0.05);
  EXPECT_LT(opt_ate, 0.5 * raw_ate);
}

TEST(PoseGraph, OutlierLoopIsGated) {
  const std::vector<Iso> truth = rectangleTrajectory(12, 9.0, 10.0);
  const std::vector<Iso> raw = integrateDriftedOdom(truth);

  Iso loop_measure = truth[0].inverse() * truth[11];
  loop_measure.translation() += Eigen::Vector3d(0.01, -0.01, 0.0);

  // 基准：只有真回环。
  std::vector<Keyframe> good_only = makeKeyframes(raw);
  {
    PoseGraph graph(syntheticGraphParams());
    std::vector<LoopConstraint> loops = {makeLoop(0, 11, loop_measure)};
    std::vector<bool> active;
    ASSERT_TRUE(graph.optimize(good_only, loops, &active));
  }
  const double good_ate = ate(good_only, truth);

  // 真回环 + 偏移 3 m 的错误回环。
  std::vector<LoopConstraint> loops;
  loops.push_back(makeLoop(0, 11, loop_measure));
  loops.push_back(makeLoop(0, 11, loop_measure, 3.0));

  std::vector<Keyframe> mixed = makeKeyframes(raw);
  PoseGraph graph(syntheticGraphParams());
  std::vector<bool> loop_active;
  ASSERT_TRUE(graph.optimize(mixed, loops, &loop_active));

  const double opt_ate = ate(mixed, truth);
  const double opt_end = translationError(mixed.back().pose_opt, truth.back());
  std::cout << "[gating] good-only ATE=" << good_ate << " m; mixed ATE=" << opt_ate
            << " m, end=" << opt_end << " m, active=" << loop_active[0] << ","
            << loop_active[1] << std::endl;

  ASSERT_EQ(loop_active.size(), 2u);
  EXPECT_TRUE(loop_active[0]);   // 真回环保留
  EXPECT_FALSE(loop_active[1]);  // 错误回环被门控剔除
  EXPECT_LT(opt_end, 0.05);
  // 错误回环没有被优化器“吃进去”：结果与只用真回环时基本一致。
  EXPECT_LT(opt_ate, good_ate + 0.05);
}

TEST(PoseGraph, HandlesIdNotEqualToIndex) {
  const std::vector<Iso> truth = rectangleTrajectory(12, 9.0, 10.0);
  const std::vector<Iso> raw = integrateDriftedOdom(truth);
  std::vector<Keyframe> keyframes = makeKeyframes(raw);

  // id 故意与下标错位，验证 id -> index 查表。
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    keyframes[i].id = 500 + static_cast<int>(i) * 7;
  }
  Iso loop_measure = truth[0].inverse() * truth[11];
  std::vector<LoopConstraint> loops = {
      makeLoop(keyframes.front().id, keyframes.back().id, loop_measure)};

  PoseGraph graph(syntheticGraphParams());
  std::vector<bool> loop_active;
  ASSERT_TRUE(graph.optimize(keyframes, loops, &loop_active));
  EXPECT_TRUE(loop_active[0]);
  EXPECT_LT(translationError(keyframes.back().pose_opt, truth.back()), 0.05);
}

TEST(PoseGraph, FiftyNodesWithInjectedDrift) {
  // 50 个关键帧沿 9x10 m 矩形闭环，同样的 5% 尺度 + 0.5°/m 航向漂移注入，
  // 回环 0 -> 49 闭合。验证长链上 optimize 成功且真回环被保留。
  const int n = 50;
  const std::vector<Iso> truth = rectangleTrajectory(n, 9.0, 10.0);
  const std::vector<Iso> raw = integrateDriftedOdom(truth);
  std::vector<Keyframe> keyframes = makeKeyframes(raw);

  Iso loop_measure = truth[0].inverse() * truth[n - 1];
  loop_measure.translation() += Eigen::Vector3d(0.01, -0.01, 0.0);
  std::vector<LoopConstraint> loops = {
      makeLoop(keyframes.front().id, keyframes.back().id, loop_measure)};

  PoseGraph graph(syntheticGraphParams());
  std::vector<bool> loop_active;
  ASSERT_TRUE(graph.optimize(keyframes, loops, &loop_active));
  ASSERT_EQ(loop_active.size(), 1u);
  EXPECT_TRUE(loop_active[0]);
  EXPECT_LT(translationError(keyframes.back().pose_opt, truth.back()), 0.05);
  std::cout << "[fifty] raw ATE=" << rawAte(keyframes, truth)
            << " m -> opt ATE=" << ate(keyframes, truth) << " m" << std::endl;
}

TEST(PoseGraph, Performance500Nodes50Loops) {
  const int n = 500;
  std::vector<Keyframe> keyframes(n);
  std::vector<Eigen::Isometry3d> truth(n);
  const double radius = 50.0;
  for (int i = 0; i < n; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
    truth[i] = makePose(radius * std::cos(angle), radius * std::sin(angle),
                        angle + M_PI / 2.0);
    keyframes[i].id = i + 1000;  // 故意让 id != index
    keyframes[i].stamp = static_cast<double>(i);
    keyframes[i].pose_raw = truth[i];
    keyframes[i].pose_opt = truth[i];
  }
  // 圆环上相邻关键帧的 odom 边天然闭环（无漂移），再加 50 条回环边。
  std::vector<LoopConstraint> loops;
  for (int k = 0; k < 50; ++k) {
    const int from = (k * 7) % n;
    const int to = (from + 37) % n;
    const Iso z = truth[from].inverse() * truth[to];
    loops.push_back(makeLoop(keyframes[from].id, keyframes[to].id, z));
  }

  PoseGraph graph(GraphParams{});
  std::vector<bool> loop_active;
  const auto start = std::chrono::steady_clock::now();
  ASSERT_TRUE(graph.optimize(keyframes, loops, &loop_active));
  const auto end = std::chrono::steady_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(end - start).count();
  std::cout << "[perf] 500 nodes + 50 loops: " << ms << " ms" << std::endl;
  // 只看数量级（目标 100 ms 量级），不断言避免 CI 抖动。
}

}  // namespace point_lio_backend