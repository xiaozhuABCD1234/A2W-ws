#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <vector>

#include "filter/pedestrian_filter.hpp"

namespace {

using filter::FilterParams;
using filter::PedestrianFilter;
using filter::PedestrianParams;
using filter::PedestrianStats;

void addWall(std::vector<Eigen::Vector3f>* points, float x, float half_width, float height) {
  for (float y = -half_width; y <= half_width + 1e-3f; y += 0.2f) {
    for (float z = 0.0f; z <= height + 1e-3f; z += 0.2f) {
      points->emplace_back(x, y, z);
    }
  }
}

void addPerson(std::vector<Eigen::Vector3f>* points, float x, float y) {
  for (float dy = -0.25f; dy <= 0.25f + 1e-3f; dy += 0.1f) {
    for (float z = 0.1f; z <= 1.7f + 1e-3f; z += 0.1f) {
      points->emplace_back(x, y + dy, z);
    }
  }
}

void addRoom(std::vector<Eigen::Vector3f>* points) {
  for (float x = 0.6f; x <= 8.0f + 1e-3f; x += 0.2f) {
    for (float y = -3.0f; y <= 3.0f + 1e-3f; y += 0.2f) {
      points->emplace_back(x, y, 0.0f);
      points->emplace_back(x, y, 2.4f);
    }
  }
  for (float y = -3.0f; y <= 3.0f + 1e-3f; y += 0.2f) {
    for (float z = 0.2f; z <= 2.4f + 1e-3f; z += 0.2f) {
      points->emplace_back(8.0f, y, z);
      points->emplace_back(-2.0f, y, z);
    }
  }
  for (float x = 0.6f; x <= 8.0f + 1e-3f; x += 0.2f) {
    for (float z = 0.2f; z <= 2.4f + 1e-3f; z += 0.2f) {
      points->emplace_back(x, 3.0f, z);
      points->emplace_back(x, -3.0f, z);
    }
  }
}

void addBox(std::vector<Eigen::Vector3f>* points, float x0, float x1, float y0, float y1,
            float z0, float z1) {
  for (float x = x0; x <= x1 + 1e-3f; x += 0.2f) {
    for (float y = y0; y <= y1 + 1e-3f; y += 0.2f) {
      for (float z = z0; z <= z1 + 1e-3f; z += 0.2f) {
        points->emplace_back(x, y, z);
      }
    }
  }
}

PedestrianFilter makeFilter() {
  FilterParams filter;
  filter.voxel_size = 0.20;
  filter.max_range = 15.0;
  filter.min_height = -1.5;
  filter.max_height = 2.5;
  filter.free_frames_threshold = 3;
  PedestrianParams pedestrian;
  return PedestrianFilter(filter, pedestrian);
}

PedestrianFilter makeFilter(int relearn_frames) {
  FilterParams filter;
  filter.voxel_size = 0.20;
  filter.max_range = 15.0;
  filter.min_height = -1.5;
  filter.max_height = 2.5;
  filter.free_frames_threshold = 3;
  filter.relearn_frames = relearn_frames;
  PedestrianParams pedestrian;
  return PedestrianFilter(filter, pedestrian);
}

size_t removedCount(const std::vector<uint8_t>& mask) {
  size_t count = 0;
  for (const uint8_t value : mask) {
    if (value != 0) ++count;
  }
  return count;
}

}

TEST(PedestrianFilterTest, staticWallIsNeverRemoved) {
  auto filter = makeFilter();
  const Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
  std::vector<Eigen::Vector3f> points;
  addWall(&points, 8.0f, 3.0f, 2.0f);
  std::vector<uint8_t> mask;
  for (int frame = 0; frame < 60; ++frame) {
    filter.filter(points, origin, &mask);
    ASSERT_EQ(mask.size(), points.size());
    EXPECT_EQ(removedCount(mask), 0u) << "frame " << frame;
  }
}

TEST(PedestrianFilterTest, pedestrianEnteringFreeSpaceIsRemoved) {
  auto filter = makeFilter();
  const Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
  std::vector<Eigen::Vector3f> room;
  addRoom(&room);
  std::vector<uint8_t> mask;
  for (int frame = 0; frame < 40; ++frame) {
    filter.filter(room, origin, &mask);
  }

  std::vector<Eigen::Vector3f> scene = room;
  const size_t wall_size = scene.size();
  addPerson(&scene, 3.0f, 0.0f);
  const size_t person_size = scene.size() - wall_size;
  filter.filter(scene, origin, &mask);
  ASSERT_EQ(mask.size(), scene.size());

  size_t person_removed = 0;
  for (size_t index = wall_size; index < scene.size(); ++index) {
    if (mask[index] != 0) ++person_removed;
  }
  EXPECT_GE(person_removed, person_size * 6 / 10);
  size_t wall_removed = 0;
  for (size_t index = 0; index < wall_size; ++index) {
    if (mask[index] != 0) ++wall_removed;
  }
  EXPECT_EQ(wall_removed, 0u);
}

TEST(PedestrianFilterTest, standingPersonStaysRemovedWhenRelearnDisabled) {
  auto filter = makeFilter(0);
  const Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
  std::vector<Eigen::Vector3f> room;
  addRoom(&room);
  std::vector<uint8_t> mask;
  for (int frame = 0; frame < 40; ++frame) filter.filter(room, origin, &mask);

  std::vector<Eigen::Vector3f> scene = room;
  const size_t wall_size = scene.size();
  addPerson(&scene, 3.0f, 0.0f);
  const size_t person_size = scene.size() - wall_size;
  for (int frame = 0; frame < 120; ++frame) {
    filter.filter(scene, origin, &mask);
    size_t removed = 0;
    for (size_t index = wall_size; index < scene.size(); ++index) {
      if (mask[index] != 0) ++removed;
    }
    ASSERT_GE(removed, person_size * 6 / 10) << "frame " << frame;
  }
}

TEST(PedestrianFilterTest, standingPersonIsRelearnedAsStatic) {
  auto filter = makeFilter(5);
  const Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
  std::vector<Eigen::Vector3f> room;
  addRoom(&room);
  std::vector<uint8_t> mask;
  for (int frame = 0; frame < 40; ++frame) filter.filter(room, origin, &mask);

  std::vector<Eigen::Vector3f> scene = room;
  const size_t person_begin = scene.size();
  addPerson(&scene, 3.0f, 0.0f);
  const size_t person_size = scene.size() - person_begin;
  size_t first_frame_removed = 0;
  for (int frame = 0; frame < 40; ++frame) {
    filter.filter(scene, origin, &mask);
    if (frame == 0) first_frame_removed = removedCount(mask);
  }
  const size_t late_removed = removedCount(mask);
  EXPECT_GE(first_frame_removed, person_size * 6 / 10) << "刚出现时应判为行人";
  EXPECT_LT(late_removed, first_frame_removed) << "开 relearn 后应逐步被当作静态结构吸收";
}

TEST(PedestrianFilterTest, wideObjectIsKept) {
  auto filter = makeFilter();
  const Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
  std::vector<Eigen::Vector3f> wall;
  addWall(&wall, 8.0f, 3.0f, 2.0f);
  std::vector<uint8_t> mask;
  for (int frame = 0; frame < 40; ++frame) {
    filter.filter(wall, origin, &mask);
  }

  std::vector<Eigen::Vector3f> scene = wall;
  addBox(&scene, 4.0f, 6.0f, -1.0f, 1.0f, 0.0f, 1.5f);
  filter.filter(scene, origin, &mask);
  EXPECT_EQ(removedCount(mask), 0u);
}

TEST(PedestrianFilterTest, lowObjectIsKept) {
  auto filter = makeFilter();
  const Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
  std::vector<Eigen::Vector3f> wall;
  addWall(&wall, 8.0f, 3.0f, 2.0f);
  std::vector<uint8_t> mask;
  for (int frame = 0; frame < 40; ++frame) {
    filter.filter(wall, origin, &mask);
  }

  std::vector<Eigen::Vector3f> scene = wall;
  addBox(&scene, 3.0f, 3.2f, -0.2f, 0.2f, 0.0f, 0.3f);
  filter.filter(scene, origin, &mask);
  EXPECT_EQ(removedCount(mask), 0u);
}

TEST(PedestrianFilterTest, farPointsAreIgnored) {
  auto filter = makeFilter();
  const Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
  std::vector<Eigen::Vector3f> points;
  addPerson(&points, 30.0f, 0.0f);
  std::vector<uint8_t> mask;
  for (int frame = 0; frame < 40; ++frame) {
    const PedestrianStats stats = filter.filter(points, origin, &mask);
    EXPECT_EQ(removedCount(mask), 0u);
    EXPECT_EQ(stats.voxel.considered_points, 0u);
  }
}

TEST(PedestrianFilterTest, resetDropsMap) {
  auto filter = makeFilter();
  const Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
  std::vector<Eigen::Vector3f> wall;
  addWall(&wall, 8.0f, 3.0f, 2.0f);
  std::vector<uint8_t> mask;
  for (int frame = 0; frame < 40; ++frame) {
    filter.filter(wall, origin, &mask);
  }
  EXPECT_GT(filter.mapVoxels(), 0u);
  filter.reset();
  EXPECT_EQ(filter.mapVoxels(), 0u);

  std::vector<Eigen::Vector3f> scene = wall;
  addPerson(&scene, 3.0f, 0.0f);
  filter.filter(scene, origin, &mask);
  EXPECT_EQ(removedCount(mask), 0u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
