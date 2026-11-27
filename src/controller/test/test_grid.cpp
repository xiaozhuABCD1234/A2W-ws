#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <vector>

#include "controller/grid.hpp"

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

}

TEST(Grid, EmptyByDefault) {
  controller::DistanceField field;
  EXPECT_TRUE(field.empty());
  EXPECT_DOUBLE_EQ(field.distance(1.0, 2.0), 3.0);
  double gradient_x = 1.0;
  double gradient_y = 1.0;
  EXPECT_FALSE(field.gradient(1.0, 2.0, &gradient_x, &gradient_y));
  EXPECT_DOUBLE_EQ(gradient_x, 0.0);
  EXPECT_DOUBLE_EQ(gradient_y, 0.0);
}

TEST(Grid, BilinearMatchesLinearField) {
  const controller::DistanceField field = makeField(
      0.0, 0.0, 100, 100, 0.1, 50.0, [](double x, double) { return x; });
  EXPECT_FALSE(field.empty());
  EXPECT_EQ(field.width(), 100);
  EXPECT_EQ(field.height(), 100);
  EXPECT_NEAR(field.distance(3.55, 4.111), 3.55, 1e-3);
  EXPECT_NEAR(field.distance(0.0, 0.0), 0.0, 1e-6);
}

TEST(Grid, GradientPointsAwayFromObstacle) {
  const controller::DistanceField field_x = makeField(
      0.0, 0.0, 120, 120, 0.05, 50.0, [](double x, double) { return x; });
  double gradient_x = 0.0;
  double gradient_y = 0.0;
  ASSERT_TRUE(field_x.gradient(2.0, 1.0, &gradient_x, &gradient_y));
  EXPECT_NEAR(gradient_x, 1.0, 0.02);
  EXPECT_NEAR(gradient_y, 0.0, 0.02);

  const controller::DistanceField field_y = makeField(
      0.0, 0.0, 120, 120, 0.05, 50.0, [](double, double y) { return y; });
  ASSERT_TRUE(field_y.gradient(2.0, 1.0, &gradient_x, &gradient_y));
  EXPECT_NEAR(gradient_x, 0.0, 0.02);
  EXPECT_NEAR(gradient_y, 1.0, 0.02);
}

TEST(Grid, OutsideReturnsMaxDistance) {
  const controller::DistanceField field = makeField(
      0.0, 0.0, 40, 40, 0.1, 4.0, [](double x, double y) { return std::hypot(x, y); });
  EXPECT_FALSE(field.inside(-1.0, 1.0));
  EXPECT_DOUBLE_EQ(field.distance(-1.0, 1.0), 4.0);
  EXPECT_DOUBLE_EQ(field.distance(10.0, 1.0), 4.0);
  EXPECT_TRUE(field.inside(1.0, 1.0));
}

TEST(Grid, ClearUsesDistance) {
  const controller::DistanceField field = makeField(
      0.0, 0.0, 60, 60, 0.05, 10.0, [](double, double y) { return y; });
  EXPECT_TRUE(field.clear(1.0, 0.5, 0.3));
  EXPECT_FALSE(field.clear(1.0, 0.2, 0.3));
}

TEST(Grid, ResetClearsField) {
  controller::DistanceField field = makeField(
      0.0, 0.0, 20, 20, 0.1, 5.0, [](double, double y) { return y; });
  ASSERT_FALSE(field.empty());
  field.reset();
  EXPECT_TRUE(field.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
