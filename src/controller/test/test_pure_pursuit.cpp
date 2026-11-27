#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "controller/pure_pursuit.hpp"

namespace {

controller::Path2D straightPath(double length) {
  std::vector<controller::Point> points;
  for (int i = 0; i <= static_cast<int>(length * 20.0); ++i) {
    controller::Point point;
    point.x = 0.5 + static_cast<double>(i) * 0.05;
    point.y = 0.0;
    points.push_back(point);
  }
  return controller::makePath(points);
}

controller::Path2D offsetPath(double offset) {
  std::vector<controller::Point> points;
  for (int i = 0; i <= 100; ++i) {
    controller::Point point;
    point.x = 0.0 + static_cast<double>(i) * 0.05;
    point.y = offset;
    points.push_back(point);
  }
  return controller::makePath(points);
}

}

TEST(PurePursuit, EmptyPathStops) {
  const controller::Path2D path;
  const controller::PurePursuitOutput output =
      controller::purePursuit(path, controller::Pose2D{}, 0.6, 1.0, controller::PurePursuitParams{});
  EXPECT_DOUBLE_EQ(output.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(output.command.angular, 0.0);
  EXPECT_FALSE(output.reached);
}

TEST(PurePursuit, DrivesStraightAlongPath) {
  const controller::Path2D path = straightPath(6.0);
  controller::Pose2D pose;
  pose.x = 0.4;
  pose.y = 0.0;
  pose.yaw = 0.0;
  controller::PurePursuitParams params;
  const controller::PurePursuitOutput output =
      controller::purePursuit(path, pose, 0.6, 1.0, params);
  EXPECT_GT(output.command.linear, 0.3);
  EXPECT_NEAR(output.command.angular, 0.0, 0.05);
  EXPECT_FALSE(output.rotating);
  EXPECT_FALSE(output.reached);
}

TEST(PurePursuit, TurnsTowardOffsetPath) {
  const controller::Path2D path = offsetPath(0.8);
  controller::Pose2D pose;
  controller::PurePursuitParams params;
  const controller::PurePursuitOutput output =
      controller::purePursuit(path, pose, 0.6, 1.0, params);
  EXPECT_GT(output.command.angular, 0.0);
  EXPECT_GT(output.command.linear, 0.0);
}

TEST(PurePursuit, LookaheadScalesWithSpeedAndClamps) {
  const controller::Path2D path = straightPath(8.0);
  controller::Pose2D pose;
  controller::PurePursuitParams params;
  const controller::PurePursuitOutput slow =
      controller::purePursuit(path, pose, 0.05, 1.0, params);
  const controller::PurePursuitOutput medium =
      controller::purePursuit(path, pose, 0.6, 1.0, params);
  const controller::PurePursuitOutput fast =
      controller::purePursuit(path, pose, 5.0, 1.0, params);
  EXPECT_NEAR(slow.lookahead, params.min_lookahead, 1e-9);
  EXPECT_NEAR(medium.lookahead, 0.6 * params.lookahead_time, 1e-9);
  EXPECT_NEAR(fast.lookahead, params.max_lookahead, 1e-9);
}

TEST(PurePursuit, RotatesInPlaceWhenTargetBehind) {
  std::vector<controller::Point> points;
  for (int i = 0; i <= 40; ++i) {
    controller::Point point;
    point.x = -1.0 - static_cast<double>(i) * 0.05;
    point.y = 0.0;
    points.push_back(point);
  }
  const controller::Path2D path = controller::makePath(points);
  controller::Pose2D pose;
  const controller::PurePursuitOutput output =
      controller::purePursuit(path, pose, 0.3, 1.0, controller::PurePursuitParams{});
  EXPECT_TRUE(output.rotating);
  EXPECT_DOUBLE_EQ(output.command.linear, 0.0);
  EXPECT_GT(std::fabs(output.command.angular), 1e-6);
}

TEST(PurePursuit, StopsWithinGoalTolerance) {
  const controller::Path2D path = straightPath(1.0);
  controller::Pose2D pose;
  pose.x = path.points.back().x - 0.05;
  pose.y = 0.0;
  controller::PurePursuitParams params;
  params.goal_tolerance = 0.20;
  const controller::PurePursuitOutput output =
      controller::purePursuit(path, pose, 0.6, 1.0, params);
  EXPECT_TRUE(output.reached);
  EXPECT_DOUBLE_EQ(output.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(output.command.angular, 0.0);
}

TEST(PurePursuit, SlowsDownNearGoal) {
  const controller::Path2D path = straightPath(1.0);
  controller::Pose2D pose;
  pose.x = 1.0;
  controller::PurePursuitParams params;
  const controller::PurePursuitOutput output =
      controller::purePursuit(path, pose, 0.6, 1.0, params);
  EXPECT_LT(output.command.linear, params.desired_linear);
  EXPECT_GT(output.command.linear, 0.0);
}

TEST(PurePursuit, SpeedScaleReducesVelocity) {
  const controller::Path2D path = straightPath(6.0);
  controller::Pose2D pose;
  controller::PurePursuitParams params;
  const controller::PurePursuitOutput full =
      controller::purePursuit(path, pose, 0.6, 1.0, params);
  const controller::PurePursuitOutput scaled =
      controller::purePursuit(path, pose, 0.6, 0.2, params);
  EXPECT_LT(scaled.command.linear, full.command.linear);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
