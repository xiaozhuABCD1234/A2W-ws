#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Geometry>

#include "relocalization/relocator.hpp"
#include "relocalization/voxel_index.hpp"

namespace {

using relocalization::PointCloudPtr;
using relocalization::PointCloudT;
using relocalization::PointT;
using relocalization::PriorMapRelocator;
using relocalization::RelocalizationParams;

void addPoint(PointCloudT& cloud, double x, double y, double z) {
  PointT point;
  point.x = static_cast<float>(x);
  point.y = static_cast<float>(y);
  point.z = static_cast<float>(z);
  point.intensity = 0.0f;
  cloud.push_back(point);
}

void addBox(PointCloudT& cloud, double cx, double cy, double z0, double sx, double sy,
            double sz, double step) {
  for (double x = -sx / 2.0; x <= sx / 2.0 + 1e-9; x += step) {
    for (double y = -sy / 2.0; y <= sy / 2.0 + 1e-9; y += step) {
      for (double z = 0.0; z <= sz + 1e-9; z += step) {
        const bool inner = std::abs(x) < sx / 2.0 - step * 0.5 &&
                           std::abs(y) < sy / 2.0 - step * 0.5;
        if (inner && z < sz - step * 0.5) continue;
        addPoint(cloud, cx + x, cy + y, z0 + z);
      }
    }
  }
}

void addFloor(PointCloudT& cloud, double width, double depth, double z, double step,
              unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> jitter(-0.01, 0.01);
  for (double x = -width / 2.0; x <= width / 2.0 + 1e-9; x += step) {
    for (double y = -depth / 2.0; y <= depth / 2.0 + 1e-9; y += step) {
      addPoint(cloud, x + jitter(rng), y + jitter(rng), z + jitter(rng));
    }
  }
}

void addWalls(PointCloudT& cloud, double width, double depth, double z0, double height,
              double door_half, double step) {
  for (double x = -width / 2.0; x <= width / 2.0 + 1e-9; x += step) {
    for (double z = z0; z <= z0 + height + 1e-9; z += step) {
      addPoint(cloud, x, -depth / 2.0, z);
      if (std::abs(x - 1.7) > door_half) addPoint(cloud, x, depth / 2.0, z);
    }
  }
  for (double y = -depth / 2.0; y <= depth / 2.0 + 1e-9; y += step) {
    for (double z = z0; z <= z0 + height + 1e-9; z += step) {
      addPoint(cloud, -width / 2.0, y, z);
      addPoint(cloud, width / 2.0, y, z);
    }
  }
}

PointCloudPtr makeRoom(double width, double depth, double z0, double height, unsigned seed) {
  PointCloudPtr cloud(new PointCloudT());
  addFloor(*cloud, width, depth, z0, 0.25, seed);
  addWalls(*cloud, width, depth, z0, height, 1.2, 0.25);
  addBox(*cloud, -3.5, 2.0, z0, 1.2, 1.4, 1.1, 0.15);
  addBox(*cloud, 4.2, -2.6, z0, 0.8, 2.2, 1.6, 0.15);
  addBox(*cloud, 0.5, -4.0, z0, 2.6, 0.7, 0.9, 0.15);
  addBox(*cloud, 5.0, 3.4, z0, 1.6, 1.0, 2.0, 0.18);
  for (double z = z0; z <= z0 + height + 1e-9; z += 0.3) {
    addPoint(*cloud, -6.4, 5.1, z);
    addPoint(*cloud, -6.1, 5.4, z);
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  return cloud;
}

PointCloudT makeScan(const PointCloudT& map, const Eigen::Isometry3d& truth, double range) {
  const Eigen::Isometry3d inverse = truth.inverse();
  PointCloudT scan;
  for (const auto& point : map.points) {
    const Eigen::Vector3d local = inverse * Eigen::Vector3d(point.x, point.y, point.z);
    if (local.norm() > range) continue;
    if (local.z() > 2.5 || local.z() < -2.5) continue;
    addPoint(scan, local.x(), local.y(), local.z());
  }
  scan.width = static_cast<uint32_t>(scan.size());
  scan.height = 1;
  return scan;
}

RelocalizationParams testParams() {
  RelocalizationParams params;
  params.pyramid_voxels = {2.0, 1.0, 0.5, 0.25};
  params.level_keep = {240, 60, 24, 12};
  params.max_query_points = 500;
  params.gicp_max_points = 4000;
  params.gicp_candidates = 6;
  params.max_fitness = 0.10;
  params.min_inlier_ratio = 0.35;
  params.min_ambiguity_margin = 0.0;
  return params;
}

Eigen::Isometry3d makePose(double x, double y, double z, double yaw, double roll = 0.0,
                           double pitch = 0.0) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() =
      (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()))
          .toRotationMatrix();
  return pose;
}

double translationError(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return (a.translation() - b.translation()).norm();
}

double rotationErrorDeg(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle() * 180.0 / M_PI;
}

}

TEST(VoxelIndex, RoundTripHandlesNegativeCoordinates) {
  PointCloudT cloud;
  for (int i = -20; i <= 20; ++i) {
    addPoint(cloud, i * 0.5, -i * 0.25, i * 0.1);
  }
  relocalization::VoxelIndex index;
  index.build(cloud, 0.25);
  EXPECT_GT(index.size(), 20u);
  for (const auto& point : cloud.points) {
    EXPECT_TRUE(index.occupied(point.x, point.y, point.z));
  }
  EXPECT_FALSE(index.occupied(100.0f, 100.0f, 100.0f));
  EXPECT_FALSE(index.occupied(-100.0f, -100.0f, -100.0f));
}

TEST(PriorMapRelocator, DerivesZRangeFromMap) {
  PointCloudT cloud;
  addFloor(cloud, 12.0, 10.0, -0.2, 0.3, 1u);
  addFloor(cloud, 12.0, 10.0, 6.0, 0.3, 2u);
  for (int i = 0; i < 5; ++i) addPoint(cloud, 50.0 + i, 0.0, 60.0);
  for (int i = 0; i < 5; ++i) addPoint(cloud, -50.0 - i, 0.0, -40.0);

  auto params = testParams();
  PriorMapRelocator relocator(params);
  std::string error;
  ASSERT_TRUE(relocator.loadCloud(cloud, &error)) << error;
  EXPECT_NEAR(relocator.zMin(), -0.2, 0.6);
  EXPECT_NEAR(relocator.zMax(), 6.0, 0.6);
  EXPECT_GT(relocator.zMax() - relocator.zMin(), 5.5);
}

TEST(PriorMapRelocator, RecoversPoseOnSingleFloor) {
  PointCloudPtr map = makeRoom(18.0, 14.0, 0.0, 2.8, 7u);
  auto params = testParams();
  PriorMapRelocator relocator(params);
  std::string error;
  ASSERT_TRUE(relocator.loadCloud(*map, &error)) << error;

  const Eigen::Isometry3d truth = makePose(2.6, -1.9, 0.55, 0.72);
  const PointCloudT scan = makeScan(*map, truth, 14.0);
  ASSERT_GT(scan.size(), 500u);

  std::string rejection;
  const auto result = relocator.localize(scan, Eigen::Isometry3d::Identity(), &rejection);
  ASSERT_TRUE(result.has_value()) << rejection;
  EXPECT_LT(translationError(result->T_map_body, truth), 0.4);
  EXPECT_LT(rotationErrorDeg(result->T_map_body, truth), 3.0);
}

TEST(PriorMapRelocator, RecoversElevatedSensorOnUpperFloor) {
  PointCloudPtr map = makeRoom(18.0, 14.0, 0.0, 2.8, 11u);
  PointCloudPtr upper = makeRoom(15.0, 12.0, 6.0, 2.6, 13u);
  *map += *upper;
  map->width = static_cast<uint32_t>(map->size());
  map->height = 1;

  auto params = testParams();
  PriorMapRelocator relocator(params);
  std::string error;
  ASSERT_TRUE(relocator.loadCloud(*map, &error)) << error;
  ASSERT_GT(relocator.zMax(), 6.0);

  const Eigen::Isometry3d truth = makePose(1.8, 2.4, 6.6, -1.15);
  const PointCloudT scan = makeScan(*map, truth, 14.0);
  ASSERT_GT(scan.size(), 500u);

  std::string rejection;
  const auto result = relocator.localize(scan, Eigen::Isometry3d::Identity(), &rejection);
  ASSERT_TRUE(result.has_value()) << rejection;
  EXPECT_LT(translationError(result->T_map_body, truth), 0.5);
  EXPECT_LT(std::abs(result->T_map_body.translation().z() - 6.6), 0.4);
  EXPECT_LT(rotationErrorDeg(result->T_map_body, truth), 3.5);
}

TEST(PriorMapRelocator, RecoversTiltedSensorInConeMode) {
  PointCloudPtr map = makeRoom(18.0, 14.0, 0.0, 2.8, 17u);
  auto params = testParams();
  params.tilt_mode = relocalization::TiltMode::Cone;
  params.tilt_deg = 15.0;
  params.tilt_step_deg = 7.5;
  PriorMapRelocator relocator(params);
  std::string error;
  ASSERT_TRUE(relocator.loadCloud(*map, &error)) << error;

  const double roll = 8.0 * M_PI / 180.0;
  const double pitch = -6.0 * M_PI / 180.0;
  const Eigen::Isometry3d truth = makePose(-3.1, 3.3, 0.9, 2.1, roll, pitch);
  const PointCloudT scan = makeScan(*map, truth, 14.0);
  ASSERT_GT(scan.size(), 500u);

  std::string rejection;
  const auto result = relocator.localize(scan, Eigen::Isometry3d::Identity(), &rejection);
  ASSERT_TRUE(result.has_value()) << rejection;
  EXPECT_GT(result->tilt_count, 1);
  EXPECT_LT(translationError(result->T_map_body, truth), 0.6);
  EXPECT_LT(rotationErrorDeg(result->T_map_body, truth), 5.0);
}

TEST(PriorMapRelocator, HonorsManualZRange) {
  PointCloudPtr map = makeRoom(18.0, 14.0, 0.0, 2.8, 19u);
  auto params = testParams();
  params.auto_z_range = false;
  params.z_min = -2.0;
  params.z_max = -1.0;
  PriorMapRelocator relocator(params);
  std::string error;
  ASSERT_TRUE(relocator.loadCloud(*map, &error)) << error;
  EXPECT_NEAR(relocator.zMin(), -2.0, 1e-6);
  EXPECT_NEAR(relocator.zMax(), -1.0, 1e-6);
}
