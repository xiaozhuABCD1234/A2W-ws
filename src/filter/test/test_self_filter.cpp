#include <gtest/gtest.h>

#include <Eigen/Core>

#include <vector>

#include "filter/self_filter.hpp"

namespace {

using filter::SelfFilter;
using filter::SelfFilterParams;
using filter::SelfFilterStats;

std::vector<Eigen::Vector3f> makeCloud() {
  std::vector<Eigen::Vector3f> points;
  points.emplace_back(0.0f, 0.0f, 0.0f);
  points.emplace_back(0.30f, -0.20f, -0.60f);
  points.emplace_back(-0.40f, 0.40f, -0.90f);
  points.emplace_back(2.00f, 0.00f, 0.00f);
  points.emplace_back(0.00f, 3.00f, 0.50f);
  points.emplace_back(0.20f, 0.20f, 1.50f);
  return points;
}

}

TEST(SelfFilterTest, bodyBoxDropsClosePoints) {
  SelfFilterParams params;
  SelfFilter self(params);
  std::vector<uint8_t> mask;
  const SelfFilterStats stats = self.filter(makeCloud(), &mask);

  ASSERT_EQ(mask.size(), 6u);
  EXPECT_EQ(stats.self_points, 3u);
  EXPECT_EQ(mask[0], 1);
  EXPECT_EQ(mask[1], 1);
  EXPECT_EQ(mask[2], 1);
  EXPECT_EQ(mask[3], 0);
  EXPECT_EQ(mask[4], 0);
  EXPECT_EQ(mask[5], 0) << "高于盒子上沿的点应保留";
}

TEST(SelfFilterTest, disabledKeepsEverything) {
  SelfFilterParams params;
  params.enabled = false;
  SelfFilter self(params);
  std::vector<uint8_t> mask;
  const SelfFilterStats stats = self.filter(makeCloud(), &mask);
  EXPECT_EQ(stats.self_points, 0u);
  for (const uint8_t value : mask) EXPECT_EQ(value, 0);
}

TEST(SelfFilterTest, marginAndRangeWidenTheBox) {
  SelfFilterParams params;
  params.margin = 0.30;
  SelfFilter self(params);
  std::vector<Eigen::Vector3f> points;
  points.emplace_back(0.65f, 0.0f, 0.0f);
  std::vector<uint8_t> mask;
  self.filter(points, &mask);
  EXPECT_EQ(mask[0], 1) << "0.45 + 0.30 外扩应覆盖 0.65 m";

  SelfFilterParams range_params;
  range_params.min_range = 1.0;
  SelfFilter ranged(range_params);
  std::vector<Eigen::Vector3f> near_points;
  near_points.emplace_back(0.95f, 0.0f, 0.0f);
  near_points.emplace_back(1.05f, 0.0f, 0.0f);
  ranged.filter(near_points, &mask);
  EXPECT_EQ(mask[0], 1) << "min_range 内一律视为本体";
  EXPECT_EQ(mask[1], 0);
}

TEST(SelfFilterTest, isSelfMatchesFilterMask) {
  SelfFilterParams params;
  SelfFilter self(params);
  const auto points = makeCloud();
  std::vector<uint8_t> mask;
  self.filter(points, &mask);
  for (size_t index = 0; index < points.size(); ++index) {
    EXPECT_EQ(mask[index] != 0, self.isSelf(points[index]));
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
