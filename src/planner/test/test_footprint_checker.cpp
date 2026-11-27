#include "planner/footprint_checker.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using planner::EsdfField;
using robot_model::Footprint;
using robot_model::FootprintParams;

namespace {

Footprint rectangle(double length, double width) {
  FootprintParams params;
  params.points = {{0.5 * length, -0.5 * width},
                   {0.5 * length, 0.5 * width},
                   {-0.5 * length, 0.5 * width},
                   {-0.5 * length, -0.5 * width}};
  return robot_model::makeFootprint(params);
}

EsdfField makeField(int width, int height, double resolution, float max_distance,
                    const std::vector<std::pair<double, double>>& obstacles) {
  EsdfField field;
  field.width = width;
  field.height = height;
  field.resolution = resolution;
  field.origin_x = -0.5 * width * resolution;
  field.origin_y = -0.5 * height * resolution;
  field.max_distance = max_distance;
  field.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height), max_distance);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const double wx = field.origin_x + (x + 0.5) * resolution;
      const double wy = field.origin_y + (y + 0.5) * resolution;
      double best = max_distance;
      for (const auto& obstacle : obstacles) {
        best = std::min(best, std::hypot(wx - obstacle.first, wy - obstacle.second));
      }
      field.data[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)] =
          static_cast<float>(best);
    }
  }
  return field;
}

}

TEST(FootprintCheckerTest, farFromObstacleIsFree) {
  const Footprint footprint = rectangle(1.0, 0.4);
  planner::FootprintChecker checker;
  checker.configure(footprint, 0.05, 0.05);
  ASSERT_TRUE(checker.valid());
  EXPECT_NEAR(checker.inscribedRadius(), 0.25, 1e-6);
  EXPECT_NEAR(checker.circumscribedRadius(), std::hypot(0.5, 0.2) + 0.05, 1e-6);

  const EsdfField field = makeField(100, 100, 0.05, 3.0f, {});
  EXPECT_TRUE(checker.freeAt(field, 0.0, 0.0, 0.0));
}

TEST(FootprintCheckerTest, centerInsideInscribedCircleIsBlocked) {
  const Footprint footprint = rectangle(1.0, 0.4);
  planner::FootprintChecker checker;
  checker.configure(footprint, 0.0, 0.05);

  const EsdfField field = makeField(100, 100, 0.05, 3.0f, {{0.2, 0.0}});
  EXPECT_FALSE(checker.freeCircle(field, 0.0, 0.0));
  EXPECT_FALSE(checker.freeAt(field, 0.0, 0.0, 0.0));
}

TEST(FootprintCheckerTest, polygonSamplingCatchesCornerObstacle) {
  const Footprint footprint = rectangle(1.0, 0.4);
  planner::FootprintChecker checker;
  checker.configure(footprint, 0.05, 0.05);

  const EsdfField field = makeField(100, 100, 0.05, 3.0f, {{0.35, 0.15}});

  EXPECT_TRUE(checker.freeCircle(field, 0.0, 0.0));
  EXPECT_FALSE(checker.freeAt(field, 0.0, 0.0, 0.0));
  EXPECT_TRUE(checker.freeAt(field, 0.0, 0.0, M_PI_2));
}

TEST(FootprintCheckerTest, circumscribedRadiusShortCircuitsAsFree) {
  const Footprint footprint = rectangle(1.0, 0.4);
  planner::FootprintChecker checker;
  checker.configure(footprint, 0.0, 0.05);

  const EsdfField field = makeField(100, 100, 0.05, 6.0f, {{5.0, 0.0}});
  EXPECT_TRUE(checker.freeAt(field, 0.0, 0.0, 0.0));
}

TEST(FootprintCheckerTest, minClearanceGrowsRequiredDistance) {
  const Footprint footprint = rectangle(1.0, 0.4);

  planner::FootprintChecker strict;
  strict.configure(footprint, 0.30, 0.05);
  planner::FootprintChecker relaxed;
  relaxed.configure(footprint, 0.0, 0.05);

  const EsdfField field = makeField(200, 200, 0.05, 6.0f, {{0.60, 0.0}});

  EXPECT_TRUE(relaxed.freeAt(field, 0.0, 0.0, 0.0));
  EXPECT_FALSE(strict.freeAt(field, 0.0, 0.0, 0.0));
}

TEST(FootprintCheckerTest, outsideFieldIsTreatedAsFar) {
  const Footprint footprint = rectangle(0.6, 0.4);
  planner::FootprintChecker checker;
  checker.configure(footprint, 0.05, 0.05);
  const EsdfField field = makeField(40, 40, 0.05, 2.0f, {{0.0, 0.0}});
  EXPECT_TRUE(checker.freeAt(field, 30.0, -30.0, 0.0));
  EXPECT_FALSE(checker.freeAt(field, 0.0, 0.0, 0.0));
}
