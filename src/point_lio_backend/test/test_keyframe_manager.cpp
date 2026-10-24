#include <gtest/gtest.h>

#include <cmath>

#include "point_lio_backend/keyframe_manager.hpp"

namespace point_lio_backend {
namespace {

KeyframeParams baseParams() {
  KeyframeParams p;
  p.min_distance = 1.0;
  p.min_rotation_deg = 10.0;
  p.max_interval_sec = 2.0;
  p.min_interval_sec = 0.3;
  p.submap_frames = 2;
  p.submap_voxel = 0.0;
  p.max_keyframes = 100;
  return p;
}

PointCloudPtr makeCloud(float x = 0.0f, float y = 0.0f, float z = 0.0f) {
  PointCloudPtr cloud(new PointCloudT());
  cloud->push_back(PointT(x, y, z, 0.0f));
  cloud->push_back(PointT(x + 0.1f, y, z, 0.0f));
  cloud->push_back(PointT(x, y + 0.1f, z, 0.0f));
  return cloud;
}

Eigen::Isometry3d poseAt(double x, double y, double yaw_rad) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << x, y, 0.0;
  pose.linear() = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

// 先放一帧并成功生成第 0 个关键帧，返回该时刻。
void seedFirstKeyframe(KeyframeManager* manager, double stamp = 0.0) {
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  manager->addFrame(stamp, pose, makeCloud());
  Keyframe kf;
  ASSERT_TRUE(manager->buildNextKeyframe(&kf));
  ASSERT_EQ(kf.id, 0);
}

TEST(KeyframeManager, FirstFrameCreatesKeyframe) {
  KeyframeManager manager(baseParams());
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  EXPECT_TRUE(manager.shouldCreate(10.0, pose));
  EXPECT_EQ(manager.frameCount(), 0u);

  manager.addFrame(10.0, pose, makeCloud());
  EXPECT_EQ(manager.frameCount(), 1u);
  Keyframe kf;
  ASSERT_TRUE(manager.buildNextKeyframe(&kf));
  EXPECT_EQ(kf.id, 0);
  EXPECT_DOUBLE_EQ(kf.stamp, 10.0);
  EXPECT_TRUE(kf.pose_raw.isApprox(pose));
  EXPECT_EQ(manager.keyframeCount(), 1u);
}

TEST(KeyframeManager, DistanceTrigger) {
  KeyframeManager manager(baseParams());
  seedFirstKeyframe(&manager);

  const Eigen::Isometry3d pose_near = poseAt(0.5, 0.0, 0.0);
  manager.addFrame(0.5, pose_near, makeCloud());
  EXPECT_FALSE(manager.shouldCreate(0.5, pose_near));

  const Eigen::Isometry3d pose_far = poseAt(1.2, 0.0, 0.0);
  manager.addFrame(0.6, pose_far, makeCloud());
  EXPECT_TRUE(manager.shouldCreate(0.6, pose_far));

  Keyframe kf;
  ASSERT_TRUE(manager.buildNextKeyframe(&kf));
  EXPECT_EQ(kf.id, 1);
  EXPECT_EQ(manager.keyframeCount(), 2u);
}

TEST(KeyframeManager, RotationTrigger) {
  KeyframeManager manager(baseParams());
  seedFirstKeyframe(&manager);

  // 平移 0.1 m（< 1.0 m），但航向 15 deg（> 10 deg）。
  const Eigen::Isometry3d pose_rotate = poseAt(0.1, 0.0, 15.0 * M_PI / 180.0);
  manager.addFrame(0.5, pose_rotate, makeCloud());
  EXPECT_TRUE(manager.shouldCreate(0.5, pose_rotate));
  Keyframe kf;
  ASSERT_TRUE(manager.buildNextKeyframe(&kf));

  // 再来 5 deg（相对上一关键帧仅 5 deg）不应触发。
  const Eigen::Isometry3d pose_small = poseAt(0.2, 0.0, 20.0 * M_PI / 180.0);
  manager.addFrame(1.0, pose_small, makeCloud());
  EXPECT_FALSE(manager.shouldCreate(1.0, pose_small));
}

TEST(KeyframeManager, TimeTrigger) {
  KeyframeManager manager(baseParams());
  seedFirstKeyframe(&manager);

  // 没有平移/旋转，但未到 max_interval。
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  manager.addFrame(1.0, pose, makeCloud());
  EXPECT_FALSE(manager.shouldCreate(1.0, pose));

  // 间隔超过 max_interval_sec（2.0 s）后强制触发。
  manager.addFrame(2.05, pose, makeCloud());
  EXPECT_TRUE(manager.shouldCreate(2.05, pose));

  Keyframe kf;
  ASSERT_TRUE(manager.buildNextKeyframe(&kf));
  EXPECT_EQ(kf.id, 1);
}

TEST(KeyframeManager, MinIntervalIsHardLowerBound) {
  KeyframeManager manager(baseParams());
  seedFirstKeyframe(&manager);

  // 仅隔 0.2 s，即使平移 5 m 也不能出关键帧。
  const Eigen::Isometry3d pose_far = poseAt(5.0, 0.0, 0.0);
  manager.addFrame(0.2, pose_far, makeCloud());
  EXPECT_FALSE(manager.shouldCreate(0.2, pose_far));

  // 超过硬下限 0.3 s 后，同样的远距离触发。
  manager.addFrame(0.31, pose_far, makeCloud());
  EXPECT_TRUE(manager.shouldCreate(0.31, pose_far));

  Keyframe kf;
  ASSERT_TRUE(manager.buildNextKeyframe(&kf));
  EXPECT_EQ(manager.keyframeCount(), 2u);
}

TEST(KeyframeManager, MaxKeyframesLimit) {
  KeyframeParams params = baseParams();
  params.max_keyframes = 3;
  KeyframeManager manager(params);

  for (int i = 0; i < 5; ++i) {
    const double stamp = static_cast<double>(i);
    const Eigen::Isometry3d pose = poseAt(2.0 * i, 0.0, 0.0);
    manager.addFrame(stamp, pose, makeCloud());
    Keyframe kf;
    const bool created = manager.buildNextKeyframe(&kf);
    if (i < 3) {
      EXPECT_TRUE(created) << "i=" << i;
    } else {
      EXPECT_FALSE(created) << "i=" << i;
    }
  }
  EXPECT_EQ(manager.keyframeCount(), 3u);
}

TEST(KeyframeManager, SubmapContainsRecentFrames) {
  KeyframeParams params = baseParams();
  params.submap_frames = 3;
  params.submap_voxel = 0.0;
  KeyframeManager manager(params);

  // 三个不同位置的小点云，最后一帧在 x = 2 m。
  manager.addFrame(0.0, poseAt(0.0, 0.0, 0.0), makeCloud(0.0f, 0.0f, 0.0f));
  manager.addFrame(1.0, poseAt(1.0, 0.0, 0.0), makeCloud(1.0f, 0.0f, 0.0f));
  manager.addFrame(2.0, poseAt(2.0, 0.0, 0.0), makeCloud(2.0f, 0.0f, 0.0f));

  Keyframe kf;
  ASSERT_TRUE(manager.buildNextKeyframe(&kf));
  // 子图在最新机体系下拼成：三帧同一点 -> 3 个点（每帧 3 点，共 9 点）。
  EXPECT_EQ(kf.cloud->size(), 9u);
  EXPECT_TRUE(kf.pose_raw.isApprox(poseAt(2.0, 0.0, 0.0)));
  EXPECT_TRUE(kf.pose_opt.isApprox(kf.pose_raw));
}

TEST(KeyframeManager, ResetClearsState) {
  KeyframeManager manager(baseParams());
  seedFirstKeyframe(&manager, 5.0);
  ASSERT_EQ(manager.keyframeCount(), 1u);

  manager.reset();
  EXPECT_EQ(manager.keyframeCount(), 0u);
  EXPECT_EQ(manager.frameCount(), 0u);
  EXPECT_TRUE(manager.shouldCreate(20.0, Eigen::Isometry3d::Identity()));

  Keyframe kf;
  manager.addFrame(20.0, Eigen::Isometry3d::Identity(), makeCloud());
  ASSERT_TRUE(manager.buildNextKeyframe(&kf));
  EXPECT_EQ(kf.id, 0);
}

}  // namespace
}  // namespace point_lio_backend
