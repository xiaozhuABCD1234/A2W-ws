#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "point_lio_backend/loop_detector.hpp"
#include "point_lio_backend/scan_context.hpp"

namespace {

using point_lio_backend::IcpParams;
using point_lio_backend::Keyframe;
using point_lio_backend::LoopDetector;
using point_lio_backend::PointCloudT;
using point_lio_backend::PointT;
using point_lio_backend::ScanContext;
using point_lio_backend::ScanContextParams;

constexpr double kPi = M_PI;

PointCloudT makeRoom(int points_per_wall = 4000, int floor_points = 4000) {
  PointCloudT cloud;
  cloud.reserve(static_cast<size_t>(4 * points_per_wall + floor_points));
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(-4.0, 4.0);
  std::uniform_real_distribution<double> h(-1.0, 1.0);
  std::uniform_real_distribution<double> f(-4.0, 4.0);

  for (int i = 0; i < points_per_wall; ++i) {
    PointT p;
    p.x = 4.0f; p.y = static_cast<float>(u(rng)); p.z = static_cast<float>(h(rng));
    cloud.push_back(p);
    p.x = -4.0f;
    cloud.push_back(p);
    p.x = static_cast<float>(u(rng)); p.y = 4.0f; p.z = static_cast<float>(h(rng));
    cloud.push_back(p);
    p.y = -4.0f;
    cloud.push_back(p);
  }
  for (int i = 0; i < floor_points; ++i) {
    PointT p;
    p.x = static_cast<float>(f(rng));
    p.y = static_cast<float>(f(rng));
    p.z = -1.0f;
    cloud.push_back(p);
  }
  return cloud;
}

// 一条与“房间”尺度差异明显的长走廊，作为“不同地点”。
PointCloudT makeCorridor(int points_per_wall = 4000, int floor_points = 4000) {
  PointCloudT cloud;
  cloud.reserve(static_cast<size_t>(4 * points_per_wall + floor_points));
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-10.0, 10.0);
  std::uniform_real_distribution<double> h(-1.0, 1.0);
  std::uniform_real_distribution<double> f(-10.0, 10.0);

  for (int i = 0; i < points_per_wall; ++i) {
    PointT p;
    p.x = static_cast<float>(u(rng)); p.y = 1.5f; p.z = static_cast<float>(h(rng));
    cloud.push_back(p);
    p.y = -1.5f;
    cloud.push_back(p);
    p.x = 10.0f; p.y = static_cast<float>(u(rng)); p.z = static_cast<float>(h(rng));
    cloud.push_back(p);
    p.x = -10.0f;
    cloud.push_back(p);
  }
  for (int i = 0; i < floor_points; ++i) {
    PointT p;
    p.x = static_cast<float>(f(rng));
    p.y = static_cast<float>(f(rng));
    p.z = -1.0f;
    cloud.push_back(p);
  }
  return cloud;
}

PointCloudT rotateZ(const PointCloudT& cloud, double degrees) {
  const double rad = degrees * kPi / 180.0;
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  PointCloudT out;
  out.reserve(cloud.size());
  for (const auto& p : cloud.points) {
    PointT q;
    q.x = static_cast<float>(c * p.x - s * p.y);
    q.y = static_cast<float>(s * p.x + c * p.y);
    q.z = p.z;
    q.intensity = p.intensity;
    out.push_back(q);
  }
  return out;
}

double wrapDeg(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

ScanContextParams defaultParams() {
  ScanContextParams params;
  params.rings = 20;
  params.sectors = 60;
  params.max_radius = 40.0;
  return params;
}

}  // namespace

TEST(ScanContext, DescribeShapeAndEmptyCells) {
  const ScanContext sc(defaultParams());
  const PointCloudT room = makeRoom();
  const Eigen::MatrixXd descriptor = sc.describe(room);
  EXPECT_EQ(descriptor.rows(), 20);
  EXPECT_EQ(descriptor.cols(), 60);
  EXPECT_GT(descriptor.cwiseAbs().sum(), 0.0);

  const PointCloudT empty;
  EXPECT_EQ(sc.describe(empty), Eigen::MatrixXd::Zero(20, 60));
}

TEST(ScanContext, SameCloudDistanceNearZero) {
  const ScanContext sc(defaultParams());
  const PointCloudT room = makeRoom();
  const Eigen::MatrixXd descriptor = sc.describe(room);
  int shift = -1;
  EXPECT_NEAR(sc.distance(descriptor, descriptor, &shift), 0.0, 1e-12);
  EXPECT_EQ(shift, 0);
}

TEST(ScanContext, RotationInvariance) {
  const ScanContext sc(defaultParams());
  const PointCloudT room = makeRoom();
  const PointCloudT rotated = rotateZ(room, 37.0);

  const Eigen::MatrixXd base_descriptor = sc.describe(room);
  const Eigen::MatrixXd rotated_descriptor = sc.describe(rotated);

  int shift = -1;
  const double distance = sc.distance(rotated_descriptor, base_descriptor, &shift);
  EXPECT_LT(distance, 0.35);

  // a.col(j) 对齐 b.col(j+shift)：旋转 +37°（等价于机体 -37° 航向）
  // 对应 shift ≈ -37 / 6°。
  const double estimated_yaw_deg = wrapDeg(static_cast<double>(shift) * 360.0 / 60.0);
  const double angular_error = std::abs(wrapDeg(estimated_yaw_deg + 37.0));
  EXPECT_LT(angular_error, 5.0) << "shift=" << shift << " yaw=" << estimated_yaw_deg;

  const double rk_distance =
      sc.ringKeyDistance(sc.ringKey(rotated_descriptor), sc.ringKey(base_descriptor));
  EXPECT_LT(rk_distance, 0.15);
}

TEST(ScanContext, DifferentRoomDistanceAboveThreshold) {
  const ScanContext sc(defaultParams());
  const Eigen::MatrixXd room_descriptor = sc.describe(makeRoom());
  const Eigen::MatrixXd corridor_descriptor = sc.describe(makeCorridor());
  EXPECT_GT(sc.distance(room_descriptor, corridor_descriptor), 0.35);
}

TEST(ScanContext, RingKeyDistanceEdgeCases) {
  const ScanContext sc(defaultParams());
  Eigen::VectorXd a(20);
  a.setConstant(1.0);
  Eigen::VectorXd b = a;
  EXPECT_NEAR(sc.ringKeyDistance(a, b), 0.0, 1e-12);
  b(0) = 3.0;
  EXPECT_NEAR(sc.ringKeyDistance(a, b), 2.0 / 20.0, 1e-12);
  EXPECT_EQ(sc.ringKeyDistance(a, Eigen::VectorXd::Zero(19)),
            std::numeric_limits<double>::max());
}

TEST(LoopDetector, DetectsRotatedRevisit) {
  ScanContextParams sc_params = defaultParams();
  // 本用例只验证几何链路（SC 航向 → GICP），历史只有 1 帧、行程为 0，
  // 必须关掉行程门限；门限行为见 RejectsStationarySelfMatch。
  sc_params.min_travel_m = 0.0;
  IcpParams icp_params;
  ScanContext sc(sc_params);

  const PointCloudT room = makeRoom(2000, 2000);
  const PointCloudT rotated = rotateZ(room, 37.0);

  Keyframe history;
  history.id = 0;
  history.stamp = 0.0;
  history.pose_opt = Eigen::Isometry3d::Identity();
  history.cloud = PointCloudT::Ptr(new PointCloudT(room));
  history.descriptor = sc.describe(room);
  history.ring_key = sc.ringKey(history.descriptor);

  Keyframe current;
  current.id = 25;
  current.stamp = 30.0;
  current.pose_opt = Eigen::Isometry3d(
      Eigen::AngleAxisd(-37.0 * kPi / 180.0, Eigen::Vector3d::UnitZ()));
  current.cloud = PointCloudT::Ptr(new PointCloudT(rotated));
  current.descriptor = sc.describe(rotated);
  current.ring_key = sc.ringKey(current.descriptor);

  LoopDetector detector(sc_params, icp_params);
  const std::vector<Keyframe> hist{history};
  const auto result = detector.detect(current, hist);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->from, history.id);
  EXPECT_EQ(result->to, current.id);
  EXPECT_LT(result->fitness, icp_params.max_fitness);
  EXPECT_GT(result->inlier_ratio, icp_params.min_inlier_ratio);

  const Eigen::AngleAxisd rotation(result->T_from_to.linear());
  EXPECT_NEAR(rotation.angle() * 180.0 / kPi, 37.0, 1.0);
  EXPECT_NEAR(std::abs(rotation.axis().z()), 1.0, 0.01);
  EXPECT_LT(result->T_from_to.translation().norm(), 0.05);
}

TEST(LoopDetector, DetectsWithDriftedInitialPose) {
  ScanContextParams sc_params = defaultParams();
  sc_params.min_travel_m = 0.0;  // 同上：验证几何与大漂移收敛，不验证行程门限
  IcpParams icp_params;
  ScanContext sc(sc_params);

  const PointCloudT room = makeRoom(2000, 2000);
  const PointCloudT rotated = rotateZ(room, 37.0);

  Keyframe history;
  history.id = 0;
  history.stamp = 0.0;
  history.pose_opt = Eigen::Isometry3d::Identity();
  history.cloud = PointCloudT::Ptr(new PointCloudT(room));
  history.descriptor = sc.describe(room);
  history.ring_key = sc.ringKey(history.descriptor);

  // 前端累计漂移约 1.5 m：粗配准必须能把它拉回来。
  Keyframe current;
  current.id = 25;
  current.stamp = 30.0;
  current.pose_opt = Eigen::Isometry3d::Identity();
  current.pose_opt.linear() =
      Eigen::AngleAxisd(-37.0 * kPi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  current.pose_opt.translation() = Eigen::Vector3d(1.2, -0.8, 0.15);
  current.cloud = PointCloudT::Ptr(new PointCloudT(rotated));
  current.descriptor = sc.describe(rotated);
  current.ring_key = sc.ringKey(current.descriptor);

  LoopDetector detector(sc_params, icp_params);
  const auto result = detector.detect(current, std::vector<Keyframe>{history});

  ASSERT_TRUE(result.has_value());
  EXPECT_LT(result->fitness, icp_params.max_fitness);
  EXPECT_GT(result->inlier_ratio, icp_params.min_inlier_ratio);
  EXPECT_LT(result->T_from_to.translation().norm(), 0.10);
  const Eigen::AngleAxisd rotation(result->T_from_to.linear());
  EXPECT_NEAR(rotation.angle() * 180.0 / kPi, 37.0, 1.0);
}

TEST(LoopDetector, RejectsRecentOrDissimilarFrames) {
  ScanContextParams sc_params = defaultParams();
  IcpParams icp_params;
  ScanContext sc(sc_params);
  LoopDetector detector(sc_params, icp_params);

  const PointCloudT room = makeRoom(1000, 1000);
  Keyframe history;
  history.id = 0;
  history.stamp = 0.0;
  history.cloud = PointCloudT::Ptr(new PointCloudT(room));
  history.descriptor = sc.describe(room);
  history.ring_key = sc.ringKey(history.descriptor);

  // 索引差不足：不进入 GICP。
  Keyframe too_recent = history;
  too_recent.id = 5;
  too_recent.stamp = 30.0;
  EXPECT_FALSE(detector.detect(too_recent, std::vector<Keyframe>{history}).has_value());

  // 时间差不足：同样拒绝。
  Keyframe too_soon = history;
  too_soon.id = 25;
  too_soon.stamp = 3.0;
  EXPECT_FALSE(detector.detect(too_soon, std::vector<Keyframe>{history}).has_value());

  // 不同地点：Scan Context 距离超阈值。
  Keyframe elsewhere = history;
  elsewhere.id = 25;
  elsewhere.stamp = 30.0;
  elsewhere.cloud = PointCloudT::Ptr(new PointCloudT(makeCorridor(1000, 1000)));
  elsewhere.descriptor = sc.describe(*elsewhere.cloud);
  elsewhere.ring_key = sc.ringKey(elsewhere.descriptor);
  EXPECT_FALSE(detector.detect(elsewhere, std::vector<Keyframe>{history}).has_value());
}

TEST(ScanContext, InvalidDescriptorDistance) {
  const ScanContext sc(defaultParams());
  const Eigen::MatrixXd a = Eigen::MatrixXd::Zero(20, 60);
  const Eigen::MatrixXd b = Eigen::MatrixXd::Zero(10, 60);
  EXPECT_EQ(sc.distance(a, b), std::numeric_limits<double>::max());
  EXPECT_EQ(sc.distance(a, a), std::numeric_limits<double>::max());
}

// 静止/原地打转时，SC 会不断匹配到早期关键帧；若不设行程下限，就会刷出
// 一堆"位移恒等"的假回环（真机静止实测过）。这里验证门限确实拦住了它。
TEST(LoopDetector, RejectsStationarySelfMatch) {
  ScanContextParams sc_params = defaultParams();
  sc_params.min_travel_m = 3.0;
  IcpParams icp_params;
  ScanContext sc(sc_params);

  const PointCloudT room = makeRoom(2000, 2000);

  Keyframe history;
  history.id = 0;
  history.stamp = 0.0;
  history.pose_opt = Eigen::Isometry3d::Identity();
  history.cloud = PointCloudT::Ptr(new PointCloudT(room));
  history.descriptor = sc.describe(room);
  history.ring_key = sc.ringKey(history.descriptor);

  Keyframe current;
  current.id = 40;
  current.stamp = 30.0;
  current.pose_opt = Eigen::Isometry3d::Identity();  // 原地不动
  current.cloud = PointCloudT::Ptr(new PointCloudT(room));
  current.descriptor = sc.describe(room);
  current.ring_key = sc.ringKey(current.descriptor);

  LoopDetector detector(sc_params, icp_params);
  const std::vector<Keyframe> hist{history};
  EXPECT_FALSE(detector.detect(current, hist).has_value());

  // 走过足够路程后，同样的观测才允许闭环。
  Keyframe moved = current;
  moved.id = 41;
  moved.pose_opt.translation() = Eigen::Vector3d(4.0, 0.0, 0.0);
  EXPECT_TRUE(detector.detect(moved, hist).has_value());
}