#include "robot_model/footprint.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

using robot_model::Footprint;
using robot_model::FootprintParams;
using robot_model::Point2;

namespace {

FootprintParams rectangle(double length, double width) {
  FootprintParams params;
  params.points = {{0.5 * length, -0.5 * width},
                   {0.5 * length, 0.5 * width},
                   {-0.5 * length, 0.5 * width},
                   {-0.5 * length, -0.5 * width}};
  return params;
}

}

TEST(FootprintTest, radiiOfRectangle) {
  const Footprint footprint = robot_model::makeFootprint(rectangle(1.0, 0.6));
  ASSERT_TRUE(footprint.valid);
  EXPECT_NEAR(footprint.inscribed_radius, 0.3, 1e-6);
  EXPECT_NEAR(footprint.circumscribed_radius, std::hypot(0.5, 0.3), 1e-6);
}

TEST(FootprintTest, paddingExpandsBothRadii) {
  FootprintParams params = rectangle(1.0, 0.6);
  params.padding = 0.05;
  const Footprint footprint = robot_model::makeFootprint(params);
  ASSERT_TRUE(footprint.valid);
  EXPECT_NEAR(footprint.inscribed_radius, 0.35, 1e-6);
  EXPECT_NEAR(footprint.circumscribed_radius, std::hypot(0.55, 0.35), 1e-6);
}

TEST(FootprintTest, clockwiseInputIsNormalized) {
  FootprintParams params = rectangle(1.0, 0.6);
  std::reverse(params.points.begin(), params.points.end());
  const Footprint footprint = robot_model::makeFootprint(params);
  ASSERT_TRUE(footprint.valid);
  EXPECT_GT(robot_model::polygonArea(footprint.points), 0.0);
  EXPECT_NEAR(robot_model::polygonArea(footprint.points), 0.6, 1e-6);
}

TEST(FootprintTest, tooFewPointsIsInvalid) {
  FootprintParams params;
  params.points = {{0.0, 0.0}, {1.0, 0.0}};
  EXPECT_FALSE(robot_model::makeFootprint(params).valid);
}

TEST(FootprintTest, nonFinitePointsAreInvalid) {
  FootprintParams params = rectangle(1.0, 0.6);
  params.points[0][0] = std::numeric_limits<double>::quiet_NaN();
  const Footprint footprint = robot_model::makeFootprint(params);
  EXPECT_FALSE(footprint.valid);
  EXPECT_TRUE(footprint.points.empty());
}

TEST(FootprintTest, degeneratePolygonHasFiniteRadii) {
  FootprintParams params;
  params.points = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
  const Footprint footprint = robot_model::makeFootprint(params);
  EXPECT_FALSE(footprint.valid);
  EXPECT_TRUE(std::isfinite(footprint.inscribed_radius));
  EXPECT_TRUE(std::isfinite(footprint.circumscribed_radius));
}

TEST(FootprintTest, pointInPolygonHandlesConcaveShape) {
  const std::vector<Point2> shape = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.3},
                                     {0.3, 0.3}, {0.3, 1.0}, {0.0, 1.0}};
  EXPECT_TRUE(robot_model::pointInPolygon(shape, 0.15, 0.15));
  EXPECT_TRUE(robot_model::pointInPolygon(shape, 0.8, 0.15));
  EXPECT_FALSE(robot_model::pointInPolygon(shape, 0.8, 0.6));
  EXPECT_FALSE(robot_model::pointInPolygon(shape, 1.5, 0.5));
}

TEST(FootprintTest, transformRotatesAndTranslates) {
  const std::vector<Point2> points = {{1.0, 0.0}, {0.0, 1.0}};
  const std::vector<Point2> rotated =
      robot_model::transformPolygon(points, 2.0, 3.0, M_PI_2);
  ASSERT_EQ(rotated.size(), 2u);
  EXPECT_NEAR(rotated[0][0], 2.0, 1e-6);
  EXPECT_NEAR(rotated[0][1], 4.0, 1e-6);
  EXPECT_NEAR(rotated[1][0], 1.0, 1e-6);
  EXPECT_NEAR(rotated[1][1], 3.0, 1e-6);
}

TEST(FootprintTest, samplesCoverEdgesAndOuterRing) {
  const Footprint footprint = robot_model::makeFootprint(rectangle(1.0, 0.6));
  const std::vector<Point2> samples = robot_model::sampleFootprint(footprint, 1.0, 2.0, 0.3, 0.1);
  ASSERT_GT(samples.size(), 20u);

  for (const Point2& point : samples) {
    const double dx = point[0] - 1.0;
    const double dy = point[1] - 2.0;
    const double radius = std::hypot(dx, dy);
    EXPECT_LE(radius, footprint.circumscribed_radius + 1e-6);
    EXPECT_GE(radius, footprint.inscribed_radius - 1e-6);
  }

  bool found_corner = false;
  for (const Point2& point : samples) {
    if (std::hypot(point[0] - (1.0 + 0.5 * std::cos(0.3) + 0.3 * std::sin(0.3)),
                   point[1] - (2.0 + 0.5 * std::sin(0.3) - 0.3 * std::cos(0.3))) < 1e-6) {
      found_corner = true;
    }
  }
  EXPECT_TRUE(found_corner);
}

TEST(FootprintTest, samplesStayInsidePolygon) {
  const Footprint footprint = robot_model::makeFootprint(rectangle(0.8, 0.8));
  const std::vector<Point2> samples = robot_model::sampleFootprint(footprint, 0.0, 0.0, 0.0, 0.05);
  for (const Point2& point : samples) {
    EXPECT_LE(std::abs(point[0]), 0.4 + 1e-6);
    EXPECT_LE(std::abs(point[1]), 0.4 + 1e-6);
    EXPECT_GE(std::hypot(point[0], point[1]), footprint.inscribed_radius - 1e-6);
  }
}
