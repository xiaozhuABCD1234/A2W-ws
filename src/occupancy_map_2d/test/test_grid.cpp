#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <initializer_list>
#include <memory>

#include "occupancy_map_2d/grid.hpp"

namespace {

using occupancy_map_2d::GridParams;
using occupancy_map_2d::GridSnapshot;
using occupancy_map_2d::OccupancyGrid2D;
using occupancy_map_2d::PointCloudT;

GridParams makeParams() {
  GridParams params;
  params.resolution = 0.1;
  params.width = 20.0;
  params.height = 20.0;
  params.origin_x = -10.0;
  params.origin_y = -10.0;
  params.max_range = 15.0;
  return params;
}

Eigen::Isometry3d makeSensor(double x = 0.0, double y = 0.0, double z = 0.0) {
  Eigen::Isometry3d sensor = Eigen::Isometry3d::Identity();
  sensor.translation() = Eigen::Vector3d(x, y, z);
  return sensor;
}

PointCloudT makeCloud(std::initializer_list<std::array<double, 3>> points) {
  PointCloudT cloud;
  for (const auto& point : points) {
    occupancy_map_2d::PointT p;
    p.x = static_cast<float>(point[0]);
    p.y = static_cast<float>(point[1]);
    p.z = static_cast<float>(point[2]);
    cloud.push_back(p);
  }
  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  return cloud;
}

int8_t cellValue(const nav_msgs::msg::OccupancyGrid& map, int cell_x, int cell_y) {
  return map.data[static_cast<size_t>(cell_y) * map.info.width + static_cast<size_t>(cell_x)];
}

constexpr int kSensorCell = 100;
constexpr int kObstacleCellX = 130;

}

TEST(OccupancyGrid2D, ObstacleIsOccupiedAndRayIsFree) {
  GridParams params = makeParams();
  params.min_component_cells = 1;
  OccupancyGrid2D grid(params);
  grid.update(makeCloud({{5.0, 0.0, 0.0}}), makeCloud({{3.0, 0.0, 0.5}}), makeSensor());

  const auto map = grid.message("map", rclcpp::Time(0, 0));
  EXPECT_EQ(cellValue(map, kObstacleCellX, kSensorCell), 70) << "一次占用观测应到 70%";

  EXPECT_EQ(cellValue(map, kSensorCell + 10, kSensorCell), 44) << "射线沿途应为 free";
  EXPECT_EQ(cellValue(map, kObstacleCellX - 1, kSensorCell), 44) << "障碍前一格是 free";
  EXPECT_EQ(cellValue(map, kObstacleCellX + 1, kSensorCell), -1) << "障碍后不应被观测";
  EXPECT_EQ(grid.occupiedCells(), 1u);
}

TEST(OccupancyGrid2D, ClearResetsCellsAndFrameMasks) {
  OccupancyGrid2D grid(makeParams());
  const auto ground = makeCloud({{5.0, 0.0, 0.0}});
  const auto obstacles = makeCloud({{3.0, 0.0, 0.5}});
  grid.update(ground, obstacles, makeSensor());
  const auto before = grid.message("map", rclcpp::Time(0, 0));
  const size_t observed_before = grid.updatedCells();

  grid.clear();
  const auto cleared = grid.message("map", rclcpp::Time(0, 0));
  EXPECT_EQ(grid.updatedCells(), 0u);
  EXPECT_EQ(grid.occupiedCells(), 0u);
  for (const auto value : cleared.data) EXPECT_EQ(value, -1);

  grid.update(ground, obstacles, makeSensor());
  const auto after = grid.message("map", rclcpp::Time(0, 0));
  ASSERT_EQ(before.data.size(), after.data.size());
  EXPECT_EQ(grid.updatedCells(), observed_before);
  for (size_t i = 0; i < before.data.size(); ++i) {
    ASSERT_EQ(before.data[i], after.data[i]) << "cell " << i;
  }
}

TEST(OccupancyGrid2D, SmallComponentsAreFiltered) {
  GridParams params = makeParams();
  params.min_component_cells = 3;
  OccupancyGrid2D grid(params);
  grid.update(makeCloud({{5.0, 0.0, 0.0}}), makeCloud({{3.0, 0.0, 0.5}}), makeSensor());
  EXPECT_EQ(cellValue(grid.message("map", rclcpp::Time(0, 0)), kObstacleCellX, kSensorCell), -1)
      << "单格占用应被 min_component_cells 过滤";

  OccupancyGrid2D clustered(makeParams());
  clustered.update(makeCloud({{5.0, 0.0, 0.0}}),
                   makeCloud({{3.0, 0.0, 0.5}, {3.0, 0.1, 0.5}, {3.0, 0.2, 0.5}}), makeSensor());
  const auto map = clustered.message("map", rclcpp::Time(0, 0));
  EXPECT_EQ(cellValue(map, kObstacleCellX, kSensorCell), 70);
  EXPECT_EQ(cellValue(map, kObstacleCellX, kSensorCell + 1), 70);
  EXPECT_EQ(cellValue(map, kObstacleCellX, kSensorCell + 2), 70);
}

TEST(OccupancyGrid2D, DedupeKeepsOccupancyAndCoversFreeSpace) {
  PointCloudT circle;
  for (int degree = 0; degree < 3600; ++degree) {
    const double angle = degree * 0.1 * M_PI / 180.0;
    occupancy_map_2d::PointT p;
    p.x = static_cast<float>(12.0 * std::cos(angle));
    p.y = static_cast<float>(12.0 * std::sin(angle));
    p.z = 0.0f;
    circle.push_back(p);
  }
  const auto obstacles = makeCloud({{3.0, 0.0, 0.5}});

  GridParams dedupe_params = makeParams();
  dedupe_params.raycast_dedupe_deg = 0.5;
  dedupe_params.min_component_cells = 1;
  GridParams brute_params = makeParams();
  brute_params.raycast_dedupe_deg = 0.0;
  brute_params.min_component_cells = 1;

  OccupancyGrid2D deduped(dedupe_params);
  OccupancyGrid2D brute(brute_params);
  deduped.update(circle, obstacles, makeSensor());
  brute.update(circle, obstacles, makeSensor());
  const auto fast = deduped.message("map", rclcpp::Time(0, 0));
  const auto slow = brute.message("map", rclcpp::Time(0, 0));

  for (size_t i = 0; i < fast.data.size(); ++i) {
    ASSERT_EQ(fast.data[i] >= 65, slow.data[i] >= 65) << "cell " << i;
  }

  const double ratio = static_cast<double>(deduped.updatedCells()) /
                       static_cast<double>(brute.updatedCells());
  EXPECT_GT(ratio, 0.80);
  EXPECT_LT(ratio, 1.15);
}

TEST(OccupancyGrid2D, SnapshotMessageMatchesDirectMessage) {
  OccupancyGrid2D grid(makeParams());
  grid.update(makeCloud({{5.0, 0.0, 0.0}, {-4.0, 2.0, 0.0}}),
              makeCloud({{3.0, 0.0, 0.5}, {-3.0, 2.0, 0.5}, {-3.1, 2.0, 0.5}}), makeSensor());
  const rclcpp::Time stamp(1, 2);
  GridSnapshot snapshot;
  grid.takeSnapshot(&snapshot);
  const auto from_snapshot = OccupancyGrid2D::toMessage(snapshot, "world", stamp);
  const auto direct = grid.message("world", stamp);
  ASSERT_EQ(from_snapshot.data.size(), direct.data.size());
  EXPECT_EQ(from_snapshot.header.frame_id, direct.header.frame_id);
  EXPECT_EQ(from_snapshot.header.stamp, direct.header.stamp);
  for (size_t i = 0; i < direct.data.size(); ++i) {
    ASSERT_EQ(from_snapshot.data[i], direct.data[i]) << "cell " << i;
  }
}

TEST(OccupancyGrid2D, VersionAdvancesOnUpdateAndClear) {
  OccupancyGrid2D grid(makeParams());
  const uint64_t start = grid.version();
  grid.update(makeCloud({{5.0, 0.0, 0.0}}), makeCloud({{3.0, 0.0, 0.5}}), makeSensor());
  const uint64_t after_update = grid.version();
  EXPECT_GT(after_update, start);
  grid.clear();
  EXPECT_GT(grid.version(), after_update);

  const uint64_t after_clear = grid.version();
  grid.update(makeCloud({{100.0, 100.0, 0.0}}), {}, makeSensor());
  EXPECT_EQ(grid.version(), after_clear);
}
