#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <vector>

#include "filter/detection_gate.hpp"

namespace {

using filter::DetectionBox;
using filter::DetectionGate;
using filter::DetectionGateParams;

DetectionBox makeBox(float x, float y, float z, float sx, float sy, float sz) {
  DetectionBox box;
  box.center = Eigen::Vector3f(x, y, z);
  box.half_size = Eigen::Vector3f(sx * 0.5f, sy * 0.5f, sz * 0.5f);
  return box;
}

std::vector<Eigen::Vector3f> makeCloud() {
  std::vector<Eigen::Vector3f> points;
  points.emplace_back(3.0f, 0.0f, 0.9f);
  points.emplace_back(3.2f, 0.2f, 1.2f);
  points.emplace_back(5.0f, 0.0f, 0.9f);
  points.emplace_back(3.0f, 2.0f, 0.9f);
  return points;
}

}

TEST(DetectionGateTest, pointsInsideBoxAreMasked) {
  DetectionGateParams params;
  params.expand = 0.0;
  DetectionGate gate(params);
  gate.set({makeBox(3.0f, 0.0f, 0.9f, 0.8f, 0.8f, 1.8f)}, 10.0);

  std::vector<uint8_t> mask;
  const size_t count = gate.filter(makeCloud(), 10.0, &mask);
  EXPECT_EQ(count, 2u);
  EXPECT_EQ(mask[0], 1);
  EXPECT_EQ(mask[1], 1);
  EXPECT_EQ(mask[2], 0);
  EXPECT_EQ(mask[3], 0);
}

TEST(DetectionGateTest, expandGrowsTheBox) {
  std::vector<Eigen::Vector3f> points;
  points.emplace_back(3.85f, 0.0f, 0.9f);
  points.emplace_back(4.05f, 0.0f, 0.9f);

  DetectionGateParams tight_params;
  tight_params.expand = 0.0;
  DetectionGate tight(tight_params);
  tight.set({makeBox(3.0f, 0.0f, 0.9f, 0.8f, 0.8f, 1.8f)}, 10.0);
  std::vector<uint8_t> mask;
  EXPECT_EQ(tight.filter(points, 10.0, &mask), 0u) << "无外扩时两点都在盒外";

  DetectionGateParams params;
  params.expand = 0.5;
  DetectionGate gate(params);
  gate.set({makeBox(3.0f, 0.0f, 0.9f, 0.8f, 0.8f, 1.8f)}, 10.0);
  EXPECT_EQ(gate.filter(points, 10.0, &mask), 1u);
  EXPECT_EQ(mask[0], 1);
  EXPECT_EQ(mask[1], 0);
}

TEST(DetectionGateTest, staleDetectionsAreIgnored) {
  DetectionGateParams params;
  params.timeout = 0.3;
  DetectionGate gate(params);
  gate.set({makeBox(3.0f, 0.0f, 0.9f, 2.0f, 2.0f, 2.0f)}, 10.0);

  std::vector<uint8_t> mask;
  EXPECT_EQ(gate.filter(makeCloud(), 10.2, &mask), 2u);
  EXPECT_EQ(gate.filter(makeCloud(), 10.5, &mask), 0u) << "过期检测框不应再生效";
}

TEST(DetectionGateTest, rotatedBoxUsesLocalCoordinates) {
  DetectionGateParams params;
  params.expand = 0.0;
  DetectionGate gate(params);
  DetectionBox box = makeBox(0.0f, 0.0f, 1.0f, 4.0f, 0.6f, 2.0f);
  const float yaw = static_cast<float>(M_PI_2);
  box.rotation = Eigen::Quaternionf(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
  gate.set({box}, 1.0);

  std::vector<Eigen::Vector3f> points;
  points.emplace_back(0.0f, 1.8f, 1.0f);
  points.emplace_back(1.8f, 0.0f, 1.0f);
  std::vector<uint8_t> mask;
  gate.filter(points, 1.0, &mask);
  EXPECT_EQ(mask[0], 1) << "旋转 90° 后长边应沿 y";
  EXPECT_EQ(mask[1], 0);
}

TEST(DetectionGateTest, smallOrOverHeightBoxesAreDropped) {
  DetectionGateParams params;
  params.min_size = 0.2;
  params.max_height = 2.0;
  DetectionGate gate(params);
  gate.set({makeBox(1.0f, 0.0f, 1.0f, 0.05f, 0.05f, 0.05f),
            makeBox(2.0f, 0.0f, 6.0f, 1.0f, 1.0f, 1.0f),
            makeBox(3.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f)},
           1.0);
  EXPECT_EQ(gate.boxes(), 1u);
}

TEST(DetectionGateTest, resetAndDisabledAreInert) {
  DetectionGateParams params;
  DetectionGate gate(params);
  gate.set({makeBox(3.0f, 0.0f, 0.9f, 2.0f, 2.0f, 2.0f)}, 5.0);
  gate.reset();
  std::vector<uint8_t> mask;
  EXPECT_EQ(gate.filter(makeCloud(), 5.0, &mask), 0u);

  DetectionGateParams off;
  off.enabled = false;
  DetectionGate disabled(off);
  disabled.set({makeBox(3.0f, 0.0f, 0.9f, 2.0f, 2.0f, 2.0f)}, 5.0);
  EXPECT_EQ(disabled.filter(makeCloud(), 5.0, &mask), 0u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
