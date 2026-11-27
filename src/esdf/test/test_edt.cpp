#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "esdf/edt.hpp"

namespace {

std::vector<float> bruteForce(const std::vector<int8_t>& cells, int width, int height,
                              double resolution, double max_distance, int threshold) {
  std::vector<float> distance(static_cast<size_t>(width) * static_cast<size_t>(height),
                             static_cast<float>(max_distance));
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      double best = std::numeric_limits<double>::max();
      for (int oy = 0; oy < height; ++oy) {
        for (int ox = 0; ox < width; ++ox) {
          if (cells[static_cast<size_t>(oy) * static_cast<size_t>(width) +
                    static_cast<size_t>(ox)] < threshold) {
            continue;
          }
          const double dx = x - ox;
          const double dy = y - oy;
          best = std::min(best, std::sqrt(dx * dx + dy * dy) * resolution);
        }
      }
      distance[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)] =
          static_cast<float>(std::min(max_distance, best));
    }
  }
  return distance;
}

}

TEST(EdtTest, singleObstacleMatchesBruteForce) {
  const int width = 21;
  const int height = 17;
  std::vector<int8_t> cells(static_cast<size_t>(width) * static_cast<size_t>(height), 0);
  cells[static_cast<size_t>(8) * static_cast<size_t>(width) + static_cast<size_t>(10)] = 100;

  esdf::EdtParams params;
  params.resolution = 0.2;
  params.max_distance = 20.0;
  params.occupied_threshold = 65;
  std::vector<float> distance;
  esdf::distanceTransform(cells, width, height, params, &distance);

  const std::vector<float> expected =
      bruteForce(cells, width, height, params.resolution, params.max_distance,
                 params.occupied_threshold);
  for (size_t i = 0; i < distance.size(); ++i) {
    ASSERT_NEAR(distance[i], expected[i], 1e-4) << "index " << i;
  }
}

TEST(EdtTest, multipleObstaclesMatchBruteForce) {
  const int width = 13;
  const int height = 11;
  std::vector<int8_t> cells(static_cast<size_t>(width) * static_cast<size_t>(height), -1);
  cells[0] = 100;
  cells[static_cast<size_t>(5) * static_cast<size_t>(width) + 6] = 90;
  cells[static_cast<size_t>(10) * static_cast<size_t>(width) + 2] = 80;
  cells[static_cast<size_t>(10) * static_cast<size_t>(width) + 12] = 70;

  esdf::EdtParams params;
  params.resolution = 0.05;
  params.max_distance = 1.0;
  params.occupied_threshold = 65;
  std::vector<float> distance;
  esdf::distanceTransform(cells, width, height, params, &distance);

  const std::vector<float> expected =
      bruteForce(cells, width, height, params.resolution, params.max_distance,
                 params.occupied_threshold);
  for (size_t i = 0; i < distance.size(); ++i) {
    ASSERT_NEAR(distance[i], expected[i], 1e-4) << "index " << i;
  }
}

TEST(EdtTest, maxDistanceClampsFarCells) {
  const int width = 41;
  const int height = 41;
  std::vector<int8_t> cells(static_cast<size_t>(width) * static_cast<size_t>(height), 0);
  cells[0] = 100;

  esdf::EdtParams params;
  params.resolution = 0.1;
  params.max_distance = 1.0;
  std::vector<float> distance;
  esdf::distanceTransform(cells, width, height, params, &distance);

  EXPECT_FLOAT_EQ(distance[static_cast<size_t>(40) * static_cast<size_t>(width) + 40], 1.0f);
  EXPECT_NEAR(distance[static_cast<size_t>(0)], 0.0f, 1e-6);
  EXPECT_NEAR(distance[static_cast<size_t>(5) * static_cast<size_t>(width)],
              0.5f, 1e-4);
}

TEST(EdtTest, unknownCanBeTreatedAsObstacle) {
  const int width = 9;
  const int height = 9;
  std::vector<int8_t> cells(static_cast<size_t>(width) * static_cast<size_t>(height), -1);
  cells[static_cast<size_t>(4) * static_cast<size_t>(width)] = 100;

  esdf::EdtParams params;
  params.resolution = 0.5;
  params.max_distance = 10.0;
  std::vector<float> distance_free;
  esdf::distanceTransform(cells, width, height, params, &distance_free);

  params.unknown_is_obstacle = true;
  std::vector<float> distance_unknown;
  esdf::distanceTransform(cells, width, height, params, &distance_unknown);

  const size_t center = static_cast<size_t>(4) * static_cast<size_t>(width) + 4;
  EXPECT_GT(distance_free[center], distance_unknown[center]);
}

TEST(EdtTest, emptyGridReturnsMaxDistance) {
  const int width = 7;
  const int height = 5;
  std::vector<int8_t> cells(static_cast<size_t>(width) * static_cast<size_t>(height), 0);

  esdf::EdtParams params;
  params.max_distance = 3.0;
  std::vector<float> distance;
  esdf::distanceTransform(cells, width, height, params, &distance);

  for (const float value : distance) {
    EXPECT_FLOAT_EQ(value, 3.0f);
  }
}

TEST(EdtTest, gradientPointsAwayFromObstacle) {
  const int width = 11;
  const int height = 11;
  std::vector<int8_t> cells(static_cast<size_t>(width) * static_cast<size_t>(height), 0);
  cells[static_cast<size_t>(5) * static_cast<size_t>(width) + 5] = 100;

  esdf::EdtParams params;
  params.resolution = 0.1;
  params.max_distance = 10.0;
  std::vector<float> distance;
  esdf::distanceTransform(cells, width, height, params, &distance);
  std::vector<float> gradient_x;
  std::vector<float> gradient_y;
  esdf::gradient(distance, width, height, params.resolution, params.max_distance, &gradient_x,
                 &gradient_y);

  const size_t right = static_cast<size_t>(5) * static_cast<size_t>(width) + 7;
  const size_t left = static_cast<size_t>(5) * static_cast<size_t>(width) + 3;
  EXPECT_GT(gradient_x[right], 0.0f);
  EXPECT_LT(gradient_x[left], 0.0f);
  EXPECT_NEAR(gradient_y[right], 0.0f, 1e-5);
}

TEST(EdtTest, bilinearInterpolatesBetweenCells) {
  const int width = 4;
  const int height = 1;
  std::vector<float> values{0.0f, 1.0f, 2.0f, 3.0f};
  EXPECT_NEAR(esdf::bilinear(values, width, height, 1.5, 0.0, -1.0f), 1.5f, 1e-5);
  EXPECT_NEAR(esdf::bilinear(values, width, height, -3.0, 0.0, -1.0f), 0.0f, 1e-5);
  EXPECT_NEAR(esdf::bilinear(values, width, height, 99.0, 0.0, -1.0f), 3.0f, 1e-5);
  EXPECT_NEAR(esdf::bilinear({}, 0, 0, 1.0, 1.0, -1.0f), -1.0f, 1e-6);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
