#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <vector>

#include "costmap_2d/costmap.hpp"

namespace {

using costmap_2d::CostmapParams;
using costmap_2d::CostmapSnapshot;
using costmap_2d::PointCloudT;
using costmap_2d::RollingCostmap;

CostmapParams makeParams() {
  CostmapParams params;
  params.resolution = 0.1;
  params.width = 10.0;
  params.height = 10.0;
  params.recenter_margin = 0.5;
  params.max_range = 8.0;
  params.blind_hold_sec = 1.0;
  params.raycast_dedupe_deg = 0.0;
  params.min_component_cells = 1;
  return params;
}

bool cellAt(const CostmapSnapshot& snapshot, double x, double y, int8_t* value) {
  const int cx = static_cast<int>(std::floor((x - snapshot.origin_x) / snapshot.resolution));
  const int cy = static_cast<int>(std::floor((y - snapshot.origin_y) / snapshot.resolution));
  if (cx < 0 || cy < 0 || cx >= snapshot.width || cy >= snapshot.height) return false;
  *value = snapshot.cells[static_cast<size_t>(cy) * static_cast<size_t>(snapshot.width) +
                          static_cast<size_t>(cx)];
  return true;
}

PointCloudT makeWall(double x, double y_from, double y_to, double z, double step = 0.1) {
  PointCloudT cloud;
  const int steps = static_cast<int>(std::lround((y_to - y_from) / step));
  for (int i = 0; i <= steps; ++i) {
    const double y = y_from + i * step;
    pcl::PointXYZI point;
    point.x = static_cast<float>(x);
    point.y = static_cast<float>(y);
    point.z = static_cast<float>(z);
    point.intensity = 1.0f;
    cloud.push_back(point);
  }
  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  return cloud;
}

}

TEST(CostmapTest, obstacleIsMarkedAndRayIsCleared) {
  CostmapParams params = makeParams();
  RollingCostmap costmap(params);
  costmap.reset(0.0, 0.0);

  PointCloudT ground;
  const PointCloudT wall = makeWall(3.0, -1.0, 1.0, 0.5);
  const Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  costmap.update(ground, wall, sensor, 0.0);

  CostmapSnapshot snapshot;
  costmap.takeSnapshot(&snapshot);

  int8_t wall_value = -1;
  int8_t ahead_value = -1;
  ASSERT_TRUE(cellAt(snapshot, 3.05, 0.0, &wall_value));
  ASSERT_TRUE(cellAt(snapshot, 1.5, 0.0, &ahead_value));
  EXPECT_GT(wall_value, 50);
  EXPECT_LT(ahead_value, 50);

  for (int frame = 0; frame < 6; ++frame) {
    costmap.update(ground, wall, sensor, 0.1 * (frame + 1));
  }
  costmap.takeSnapshot(&snapshot);
  ASSERT_TRUE(cellAt(snapshot, 1.5, 0.0, &ahead_value));
  EXPECT_LT(ahead_value, 20);
}

TEST(CostmapTest, obstaclesOutsideHeightBandAreIgnored) {
  CostmapParams params = makeParams();
  RollingCostmap costmap(params);
  costmap.reset(0.0, 0.0);

  PointCloudT ground;
  const PointCloudT ceiling = makeWall(3.0, -1.0, 1.0, 2.2);
  costmap.update(ground, ceiling, Eigen::Isometry3d::Identity(), 0.0);

  CostmapSnapshot snapshot;
  costmap.takeSnapshot(&snapshot);
  int8_t value = 0;
  ASSERT_TRUE(cellAt(snapshot, 3.05, 0.0, &value));
  EXPECT_EQ(value, -1) << "高出障碍带的天花板不应进入代价地图";
}

TEST(CostmapTest, rayAboveClearanceDoesNotMarkFree) {
  CostmapParams params = makeParams();
  params.obstacle_min_z = -2.0;
  params.obstacle_max_z = 3.0;
  params.clear_min_z = -0.3;
  params.clear_max_z = 0.4;
  RollingCostmap costmap(params);
  costmap.reset(0.0, 0.0);

  PointCloudT ground;
  const PointCloudT high = makeWall(4.0, -0.4, 0.4, 2.0);
  costmap.update(ground, high, Eigen::Isometry3d::Identity(), 0.0);

  CostmapSnapshot snapshot;
  costmap.takeSnapshot(&snapshot);
  int8_t value = 0;
  ASSERT_TRUE(cellAt(snapshot, 2.0, 0.0, &value));
  EXPECT_EQ(value, -1) << "高于机器人净空的射线不应把格子标成空闲";
}

TEST(CostmapTest, blindCellsStayUnknown) {
  CostmapParams params = makeParams();
  RollingCostmap costmap(params);
  costmap.reset(0.0, 0.0);

  PointCloudT cloud;
  costmap.update(cloud, cloud, Eigen::Isometry3d::Identity(), 0.0);

  CostmapSnapshot snapshot;
  costmap.takeSnapshot(&snapshot);
  int8_t value = 0;
  ASSERT_TRUE(cellAt(snapshot, 0.0, 0.0, &value));
  EXPECT_EQ(value, -1) << "从未观测的格子必须是未知";
}

TEST(CostmapTest, staleObstacleDecaysAfterHold) {
  CostmapParams params = makeParams();
  params.blind_hold_sec = 0.5;
  params.occupied_decay = 1.0;
  RollingCostmap costmap(params);
  costmap.reset(0.0, 0.0);

  PointCloudT ground;
  const PointCloudT wall = makeWall(2.0, -0.5, 0.5, 0.5);
  costmap.update(ground, wall, Eigen::Isometry3d::Identity(), 0.0);

  CostmapSnapshot snapshot;
  costmap.takeSnapshot(&snapshot);
  int8_t value = -1;
  ASSERT_TRUE(cellAt(snapshot, 2.05, 0.0, &value));
  EXPECT_GT(value, 50);

  costmap.decay(0.2);
  costmap.takeSnapshot(&snapshot);
  ASSERT_TRUE(cellAt(snapshot, 2.05, 0.0, &value));
  EXPECT_GT(value, 50) << "保持窗口内不应衰减";

  for (int step = 0; step < 20; ++step) costmap.decay(1.0 + 0.1 * step);
  costmap.takeSnapshot(&snapshot);
  ASSERT_TRUE(cellAt(snapshot, 2.05, 0.0, &value));
  EXPECT_LT(value, 30) << "超过保持窗口后陈旧障碍应被侵蚀";
}

TEST(CostmapTest, recenterKeepsObservedCells) {
  CostmapParams params = makeParams();
  RollingCostmap costmap(params);
  costmap.reset(0.0, 0.0);

  PointCloudT ground;
  const PointCloudT wall = makeWall(1.5, -0.5, 0.5, 0.5);
  costmap.update(ground, wall, Eigen::Isometry3d::Identity(), 0.0);

  EXPECT_FALSE(costmap.needsRecenter(0.2, 0.0));
  EXPECT_TRUE(costmap.needsRecenter(3.0, 0.0));
  costmap.recenter(1.0, 0.0);

  CostmapSnapshot snapshot;
  costmap.takeSnapshot(&snapshot);
  int8_t value = -1;
  ASSERT_TRUE(cellAt(snapshot, 1.55, 0.0, &value));
  EXPECT_GT(value, 50) << "滚动窗口平移后已观测的障碍应保留";
}

TEST(CostmapTest, clearResetsToUnknown) {
  CostmapParams params = makeParams();
  RollingCostmap costmap(params);
  costmap.reset(0.0, 0.0);

  PointCloudT ground;
  const PointCloudT wall = makeWall(2.0, -0.5, 0.5, 0.5);
  costmap.update(ground, wall, Eigen::Isometry3d::Identity(), 0.0);
  costmap.reset(0.0, 0.0);

  CostmapSnapshot snapshot;
  costmap.takeSnapshot(&snapshot);
  int8_t value = 0;
  ASSERT_TRUE(cellAt(snapshot, 2.05, 0.0, &value));
  EXPECT_EQ(value, -1);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
