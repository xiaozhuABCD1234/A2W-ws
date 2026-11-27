#include "robot_model/limits.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

TEST(LimitsTest, clampsToMaximumVelocity) {
  robot_model::VelocityLimits limits;
  limits.max_linear_velocity = 0.5;
  limits.max_angular_velocity = 1.0;
  limits.max_linear_acceleration = 100.0;
  limits.max_angular_acceleration = 100.0;

  robot_model::Twist2D current;
  const robot_model::Twist2D result =
      robot_model::applyLimits(current, {2.0, -3.0}, 1.0, limits);
  EXPECT_NEAR(result.linear, 0.5, 1e-9);
  EXPECT_NEAR(result.angular, -1.0, 1e-9);
}

TEST(LimitsTest, respectsAccelerationPerStep) {
  robot_model::VelocityLimits limits;
  limits.max_linear_velocity = 1.0;
  limits.max_linear_acceleration = 0.5;
  limits.max_angular_velocity = 2.0;
  limits.max_angular_acceleration = 1.0;

  robot_model::Twist2D current;
  robot_model::Twist2D result = robot_model::applyLimits(current, {1.0, 2.0}, 0.1, limits);
  EXPECT_NEAR(result.linear, 0.05, 1e-9);
  EXPECT_NEAR(result.angular, 0.1, 1e-9);

  current = result;
  for (int i = 0; i < 100; ++i) {
    result = robot_model::applyLimits(current, {1.0, 2.0}, 0.1, limits);
    current = result;
  }
  EXPECT_NEAR(result.linear, 1.0, 1e-9);
  EXPECT_NEAR(result.angular, 2.0, 1e-9);
}

TEST(LimitsTest, deceleratesTowardsZero) {
  robot_model::VelocityLimits limits;
  limits.max_linear_velocity = 2.0;
  limits.max_angular_velocity = 2.0;
  limits.max_linear_acceleration = 0.2;
  limits.max_angular_acceleration = 0.4;

  robot_model::Twist2D current{1.0, -1.0};
  const robot_model::Twist2D result = robot_model::applyLimits(current, {0.0, 0.0}, 0.5, limits);
  EXPECT_NEAR(result.linear, 0.9, 1e-9);
  EXPECT_NEAR(result.angular, -0.8, 1e-9);
}

TEST(LimitsTest, invalidStepKeepsCurrent) {
  robot_model::VelocityLimits limits;
  robot_model::Twist2D current{0.3, 0.2};
  const robot_model::Twist2D result = robot_model::applyLimits(current, {1.0, 1.0}, 0.0, limits);
  EXPECT_NEAR(result.linear, current.linear, 1e-12);
  EXPECT_NEAR(result.angular, current.angular, 1e-12);
}

TEST(LimitsTest, nonFiniteCommandsAreRejected) {
  robot_model::VelocityLimits limits;
  const robot_model::Twist2D current{0.2, 0.1};
  const robot_model::Twist2D result = robot_model::applyLimits(
      current, {std::numeric_limits<double>::quiet_NaN(),
               std::numeric_limits<double>::infinity()}, 0.1, limits);
  EXPECT_TRUE(std::isfinite(result.linear));
  EXPECT_TRUE(std::isfinite(result.angular));
  EXPECT_NEAR(result.linear, 0.2, 0.1);
  EXPECT_NEAR(result.angular, 0.1, 0.1);
}

TEST(LimitsTest, staleOversizedCommandIsClamped) {
  robot_model::VelocityLimits limits;
  limits.max_linear_velocity = 0.5;
  limits.max_angular_velocity = 1.0;
  limits.max_linear_acceleration = 10.0;
  limits.max_angular_acceleration = 10.0;
  const robot_model::Twist2D result =
      robot_model::applyLimits({5.0, -5.0}, {5.0, -5.0}, 1.0, limits);
  EXPECT_NEAR(result.linear, 0.5, 1e-9);
  EXPECT_NEAR(result.angular, -1.0, 1e-9);
}
