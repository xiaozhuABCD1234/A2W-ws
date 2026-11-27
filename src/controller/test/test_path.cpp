#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "controller/path.hpp"

namespace {

controller::Point makePoint(double x, double y) {
  controller::Point point;
  point.x = x;
  point.y = y;
  return point;
}

}

TEST(Path, ResampleKeepsEndpointsAndSpacing) {
  const std::vector<controller::Point> input{makePoint(0.0, 0.0), makePoint(1.0, 0.0),
                                             makePoint(2.0, 0.0)};
  std::vector<controller::Point> output;
  controller::resamplePath(input, 0.25, &output);
  ASSERT_GT(output.size(), 8u);
  EXPECT_NEAR(output.front().x, 0.0, 1e-9);
  EXPECT_NEAR(output.back().x, 2.0, 1e-9);
  for (std::size_t i = 1; i < output.size(); ++i) {
    EXPECT_NEAR(controller::pointDistance(output[i - 1], output[i]), 0.25, 1e-6);
  }
}

TEST(Path, ResampleHandlesDuplicateInput) {
  const std::vector<controller::Point> input{makePoint(0.0, 0.0), makePoint(0.0, 0.0),
                                             makePoint(0.0, 0.0)};
  std::vector<controller::Point> output;
  controller::resamplePath(input, 0.1, &output);
  ASSERT_EQ(output.size(), 1u);
}

TEST(Path, ArcLengthAndYaw) {
  const std::vector<controller::Point> points{makePoint(0.0, 0.0), makePoint(1.0, 0.0),
                                              makePoint(1.0, 1.0)};
  const controller::Path2D path = controller::makePath(points);
  ASSERT_EQ(path.points.size(), 3u);
  EXPECT_NEAR(path.arc[0], 0.0, 1e-9);
  EXPECT_NEAR(path.arc[1], 1.0, 1e-9);
  EXPECT_NEAR(path.arc[2], 2.0, 1e-9);
  EXPECT_NEAR(path.yaw[0], 0.0, 1e-9);
  EXPECT_NEAR(path.yaw[2], M_PI / 2.0, 1e-9);
}

TEST(Path, NearestIndexWindow) {
  std::vector<controller::Point> points;
  for (int i = 0; i < 20; ++i) points.push_back(makePoint(static_cast<double>(i), 0.0));
  const controller::Path2D path = controller::makePath(points);
  EXPECT_EQ(controller::nearestIndex(path, makePoint(7.2, 0.3), 0, 0), 7u);
  EXPECT_EQ(controller::nearestIndex(path, makePoint(7.2, 0.3), 5, 1), 6u);
  EXPECT_EQ(controller::nearestIndex(path, makePoint(7.2, 0.3), 5, 3), 7u);
}

TEST(Path, NearestIndexClampsOutOfRangeHint) {
  std::vector<controller::Point> points;
  for (int i = 0; i < 20; ++i) points.push_back(makePoint(static_cast<double>(i), 0.0));
  const controller::Path2D path = controller::makePath(points);
  EXPECT_EQ(controller::nearestIndex(path, makePoint(19.1, 0.0), 1000, 2), 19u);
  EXPECT_EQ(controller::nearestIndex(path, makePoint(0.1, 0.0), 1000, 2), 17u);
  EXPECT_EQ(controller::nearestIndex(path, makePoint(1.0, 0.0), 0, 0), 1u);
}

TEST(Path, EmptyPathIsSafe) {
  const controller::Path2D path;
  const controller::Point query{0.0, 0.0};
  EXPECT_EQ(controller::nearestIndex(path, query, 10, 3), 0u);
  EXPECT_EQ(controller::advanceIndex(path, query, 10, 3), 0u);
  EXPECT_EQ(controller::indexAtDistance(path, 1.0), 0u);
  EXPECT_DOUBLE_EQ(controller::remainingDistance(path, 0), 0.0);
}

TEST(Path, AdvanceIndexOnlyMovesForward) {
  std::vector<controller::Point> points;
  for (int i = 0; i < 20; ++i) points.push_back(makePoint(static_cast<double>(i), 0.0));
  const controller::Path2D path = controller::makePath(points);
  EXPECT_EQ(controller::advanceIndex(path, makePoint(9.4, 0.0), 4, 10), 9u);
  EXPECT_EQ(controller::advanceIndex(path, makePoint(1.0, 0.0), 9, 10), 9u);
  EXPECT_EQ(controller::advanceIndex(path, makePoint(25.0, 0.0), 0, 100), 19u);
}

TEST(Path, IndexAtDistanceAndRemaining) {
  std::vector<controller::Point> points;
  for (int i = 0; i <= 10; ++i) points.push_back(makePoint(static_cast<double>(i), 0.0));
  const controller::Path2D path = controller::makePath(points);
  EXPECT_EQ(controller::indexAtDistance(path, -1.0), 0u);
  EXPECT_EQ(controller::indexAtDistance(path, 0.4), 1u);
  EXPECT_EQ(controller::indexAtDistance(path, 3.6), 4u);
  EXPECT_EQ(controller::indexAtDistance(path, 100.0), 10u);
  EXPECT_NEAR(controller::remainingDistance(path, 4), 6.0, 1e-9);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
