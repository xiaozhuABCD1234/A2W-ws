#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <numeric>
#include <vector>

#include "controller/mppi.hpp"

namespace {

controller::Path2D straightPath(double from, double to) {
  std::vector<controller::Point> points;
  for (double x = from; x <= to + 1e-9; x += 0.05) {
    controller::Point point;
    point.x = x;
    point.y = 0.0;
    points.push_back(point);
  }
  return controller::makePath(points);
}

controller::DistanceField makeField(double origin_x, double origin_y, int width, int height,
                                    double resolution, double max_distance,
                                    const std::function<double(double, double)>& value) {
  std::vector<float> cells(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const double world_x = origin_x + static_cast<double>(x) * resolution;
      const double world_y = origin_y + static_cast<double>(y) * resolution;
      cells[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(x)] = static_cast<float>(value(world_x, world_y));
    }
  }
  controller::DistanceField field;
  field.update(cells, width, height, resolution, origin_x, origin_y, max_distance);
  return field;
}

controller::MppiParams smallParams() {
  controller::MppiParams params;
  params.batch = 120;
  params.time_steps = 30;
  params.model_dt = 0.05;
  return params;
}

}

TEST(MppiSoftmax, UniformCostsGiveUniformWeights) {
  const std::vector<double> costs{2.0, 2.0, 2.0, 2.0};
  const std::vector<double> weights = controller::softmaxWeights(costs, 0.3);
  ASSERT_EQ(weights.size(), 4u);
  for (double weight : weights) {
    EXPECT_NEAR(weight, 0.25, 1e-9);
  }
  EXPECT_NEAR(std::accumulate(weights.begin(), weights.end(), 0.0), 1.0, 1e-9);
}

TEST(MppiSoftmax, SingleBestSampleDominates) {
  const std::vector<double> costs{0.0, 100.0, 100.0};
  const std::vector<double> weights = controller::softmaxWeights(costs, 0.3);
  EXPECT_NEAR(weights[0], 1.0, 1e-6);
  EXPECT_LT(weights[1], 1e-9);
}

TEST(MppiSoftmax, EmptyInput) {
  const std::vector<double> weights = controller::softmaxWeights({}, 0.3);
  EXPECT_TRUE(weights.empty());
}

TEST(Mppi, EmptyPathIsInfeasible) {
  controller::Mppi mppi;
  mppi.configure(smallParams());
  const controller::MppiOutput output =
      mppi.optimize(controller::Pose2D{}, controller::DistanceField{}, controller::Point{5.0, 0.0});
  EXPECT_FALSE(output.feasible);
  EXPECT_DOUBLE_EQ(output.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(output.command.angular, 0.0);
}

TEST(Mppi, SetPathAcceptsBarePointList) {
  controller::Mppi mppi;
  controller::MppiParams params = smallParams();
  mppi.configure(params);
  controller::Path2D path;
  path.points = straightPath(0.5, 3.0).points;
  mppi.setPath(path);
  ASSERT_EQ(mppi.path().yaw.size(), mppi.path().points.size());
  ASSERT_EQ(mppi.path().arc.size(), mppi.path().points.size());
  mppi.warmStart(controller::Command{0.5, 0.0});
  const controller::MppiOutput output = mppi.optimize(
      controller::Pose2D{}, controller::DistanceField{}, controller::Point{3.0, 0.0});
  EXPECT_TRUE(output.feasible);
}

TEST(Mppi, DrivesForwardOnFreePath) {
  controller::Mppi mppi;
  controller::MppiParams params = smallParams();
  mppi.configure(params);
  mppi.setPath(straightPath(0.5, 8.0));
  mppi.warmStart(controller::Command{0.5, 0.0});
  controller::Pose2D pose;
  const controller::MppiOutput output =
      mppi.optimize(pose, controller::DistanceField{}, controller::Point{8.0, 0.0});
  EXPECT_TRUE(output.feasible);
  EXPECT_GT(output.command.linear, 0.1);
  EXPECT_LT(std::fabs(output.command.angular), 0.5);
  ASSERT_GE(output.best_trajectory.size(), 2u);
  EXPECT_GT(output.best_trajectory.back().x, output.best_trajectory.front().x);
}

TEST(Mppi, RespectsVelocityLimits) {
  controller::Mppi mppi;
  controller::MppiParams params = smallParams();
  params.max_linear = 0.35;
  params.max_angular = 0.60;
  mppi.configure(params);
  mppi.setPath(straightPath(0.5, 8.0));
  controller::Pose2D pose;
  pose.yaw = 1.0;
  const controller::MppiOutput output =
      mppi.optimize(pose, controller::DistanceField{}, controller::Point{8.0, 0.0});
  EXPECT_LE(output.command.linear, params.max_linear + 1e-9);
  EXPECT_LE(std::fabs(output.command.angular), params.max_angular + 1e-9);
  for (double value : mppi.controlLinear()) {
    EXPECT_LE(value, params.max_linear + 1e-9);
    EXPECT_GE(value, params.min_linear - 1e-9);
  }
}

TEST(Mppi, SlowsDownBeforeObstacleWall) {
  controller::Mppi mppi;
  controller::MppiParams params = smallParams();
  mppi.configure(params);
  mppi.setPath(straightPath(0.5, 6.0));
  const controller::DistanceField field = makeField(
      -2.0, -3.0, 120, 120, 0.05, 8.0,
      [](double x, double) { return std::fabs(x - 1.5); });
  controller::Pose2D pose;
  const controller::MppiOutput output = mppi.optimize(pose, field, controller::Point{6.0, 0.0});
  ASSERT_TRUE(output.have_field);
  EXPECT_LT(output.command.linear, 0.45);
  for (const controller::Point& point : output.best_trajectory) {
    EXPECT_GT(field.distance(point.x, point.y), params.critical_clearance);
  }
}

TEST(Mppi, InfeasibleWhenAlreadyInCollision) {
  controller::Mppi mppi;
  controller::MppiParams params = smallParams();
  mppi.configure(params);
  mppi.setPath(straightPath(0.5, 6.0));
  const controller::DistanceField field = makeField(
      -2.0, -3.0, 80, 120, 0.05, 8.0, [](double, double) { return 0.05; });
  controller::Pose2D pose;
  const controller::MppiOutput output = mppi.optimize(pose, field, controller::Point{6.0, 0.0});
  EXPECT_FALSE(output.feasible);
  EXPECT_EQ(output.colliding_candidates, params.batch);
}

TEST(Mppi, ControlSequenceShiftsTowardsTail) {
  controller::Mppi mppi;
  controller::MppiParams params = smallParams();
  params.shift_steps = 1;
  mppi.configure(params);
  mppi.setPath(straightPath(0.5, 8.0));
  controller::Pose2D pose;
  const controller::MppiOutput first =
      mppi.optimize(pose, controller::DistanceField{}, controller::Point{8.0, 0.0});
  ASSERT_GT(first.command.linear, 0.0);
  const std::vector<double> linear = mppi.controlLinear();
  ASSERT_GE(linear.size(), 3u);
  EXPECT_NEAR(linear.back(), linear[linear.size() - 2], 1e-12);
  EXPECT_NE(linear.back(), first.command.linear);
}

TEST(Mppi, ResetClearsControlSequence) {
  controller::Mppi mppi;
  mppi.configure(smallParams());
  mppi.setPath(straightPath(0.5, 8.0));
  controller::Pose2D pose;
  mppi.optimize(pose, controller::DistanceField{}, controller::Point{8.0, 0.0});
  mppi.reset();
  for (double value : mppi.controlLinear()) {
    EXPECT_DOUBLE_EQ(value, 0.0);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
