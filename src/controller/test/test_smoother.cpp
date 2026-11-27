#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include "controller/smoother.hpp"

namespace {

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

double curvatureSum(const std::vector<controller::Point>& points) {
  double sum = 0.0;
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const double x = points[i - 1].x - 2.0 * points[i].x + points[i + 1].x;
    const double y = points[i - 1].y - 2.0 * points[i].y + points[i + 1].y;
    sum += std::hypot(x, y);
  }
  return sum;
}

std::vector<controller::Point> zigzag() {
  std::vector<controller::Point> points;
  for (int i = 0; i <= 60; ++i) {
    controller::Point point;
    point.x = static_cast<double>(i) * 0.05;
    point.y = (i % 2 == 0) ? 0.10 : -0.10;
    points.push_back(point);
  }
  return points;
}

}

TEST(Smoother, RejectsShortInput) {
  const std::vector<controller::Point> input{{0.0, 0.0}, {1.0, 0.0}};
  std::vector<controller::Point> output;
  controller::SmootherStats stats;
  EXPECT_FALSE(controller::smoothPath(input, controller::DistanceField{},
                                      controller::SmootherParams{}, &output, &stats));
  ASSERT_EQ(output.size(), input.size());
}

TEST(Smoother, StraightensZigZagAndKeepsEndpoints) {
  const std::vector<controller::Point> input = zigzag();
  controller::SmootherParams params;
  params.min_clearance = 0.0;
  std::vector<controller::Point> output;
  controller::SmootherStats stats;
  ASSERT_TRUE(controller::smoothPath(input, controller::DistanceField{}, params, &output, &stats));
  ASSERT_EQ(output.size(), input.size());
  EXPECT_FALSE(stats.used_field);
  EXPECT_LT(curvatureSum(output), 0.25 * curvatureSum(input));
  EXPECT_NEAR(output.front().x, input.front().x, 1e-12);
  EXPECT_NEAR(output.front().y, input.front().y, 1e-12);
  EXPECT_NEAR(output.back().x, input.back().x, 1e-12);
  EXPECT_NEAR(output.back().y, input.back().y, 1e-12);
  EXPECT_GT(stats.iterations, 0);
  EXPECT_LT(stats.max_deviation, 0.15);
}

TEST(Smoother, ProjectsAwayFromObstacle) {
  const controller::DistanceField field = makeField(
      0.0, 0.0, 120, 120, 0.05, 10.0, [](double, double y) { return y; });

  std::vector<controller::Point> input;
  for (int i = 0; i <= 40; ++i) {
    controller::Point point;
    point.x = 0.5 + static_cast<double>(i) * 0.05;
    point.y = 0.08;
    input.push_back(point);
  }

  controller::SmootherParams params;
  params.min_clearance = 0.22;
  std::vector<controller::Point> output;
  controller::SmootherStats stats;
  ASSERT_TRUE(controller::smoothPath(input, field, params, &output, &stats));
  EXPECT_TRUE(stats.used_field);
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i + 1 < output.size(); ++i) {
    minimum = std::min(minimum, field.distance(output[i].x, output[i].y));
  }
  EXPECT_GE(minimum, 0.20);
  EXPECT_GE(stats.min_clearance, 0.20);
  EXPECT_NEAR(output.front().y, 0.08, 1e-12);
}

TEST(Smoother, RespectsTimeBudget) {
  const std::vector<controller::Point> input = zigzag();
  controller::SmootherParams params;
  params.time_budget_ms = 0.0;
  params.iterations = 100000;
  std::vector<controller::Point> output;
  controller::SmootherStats stats;
  ASSERT_TRUE(controller::smoothPath(input, controller::DistanceField{}, params, &output, &stats));
  EXPECT_TRUE(stats.limited_by_time);
  EXPECT_EQ(stats.iterations, 1);
}

TEST(Smoother, ConvergesWithoutFieldOnSmoothPath) {
  std::vector<controller::Point> input;
  for (int i = 0; i <= 50; ++i) {
    controller::Point point;
    point.x = static_cast<double>(i) * 0.05;
    point.y = 0.0;
    input.push_back(point);
  }
  controller::SmootherParams params;
  std::vector<controller::Point> output;
  controller::SmootherStats stats;
  ASSERT_TRUE(controller::smoothPath(input, controller::DistanceField{}, params, &output, &stats));
  EXPECT_FALSE(stats.limited_by_time);
  EXPECT_LT(stats.max_deviation, 1e-3);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
