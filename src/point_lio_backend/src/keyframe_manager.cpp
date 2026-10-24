#include "point_lio_backend/keyframe_manager.hpp"

#include <pcl/filters/voxel_grid.h>

#include <algorithm>

namespace point_lio_backend {

KeyframeManager::KeyframeManager(const KeyframeParams& params) : params_(params) {}

void KeyframeManager::addFrame(double stamp, const Eigen::Isometry3d& pose_raw,
                               const PointCloudPtr& cloud_body) {
  Frame frame;
  frame.stamp = stamp;
  frame.pose = pose_raw;
  frame.cloud = cloud_body;
  frames_.push_back(frame);
  const size_t keep = static_cast<size_t>(std::max(1, params_.submap_frames));
  while (frames_.size() > keep) {
    frames_.pop_front();
  }
}

bool KeyframeManager::shouldCreate(double stamp, const Eigen::Isometry3d& pose_raw) const {
  if (!has_kf_) return true;
  const double dt = stamp - last_kf_stamp_;
  if (dt < params_.min_interval_sec) return false;
  if (dt >= params_.max_interval_sec) return true;
  const double dist = (pose_raw.translation() - last_kf_pose_.translation()).norm();
  if (dist >= params_.min_distance) return true;
  const double rot_deg =
      Eigen::AngleAxisd(pose_raw.linear().transpose() * last_kf_pose_.linear())
          .angle() * 180.0 / M_PI;
  return rot_deg >= params_.min_rotation_deg;
}

bool KeyframeManager::buildNextKeyframe(Keyframe* out) {
  if (!out || frames_.empty()) return false;
  if (static_cast<int>(keyframe_count_) >= params_.max_keyframes) return false;
  const Frame& latest = frames_.back();

  PointCloudT::Ptr merged(new PointCloudT());
  for (const auto& f : frames_) {
    if (!f.cloud) continue;
    const Eigen::Isometry3d rel = latest.pose.inverse() * f.pose;
    PointCloudT transformed;
    transformed.reserve(f.cloud->size());
    for (const auto& p : f.cloud->points) {
      const Eigen::Vector3d q = rel * Eigen::Vector3d(p.x, p.y, p.z);
      PointT pt = p;
      pt.x = static_cast<float>(q.x());
      pt.y = static_cast<float>(q.y());
      pt.z = static_cast<float>(q.z());
      transformed.push_back(pt);
    }
    *merged += transformed;
  }

  if (params_.submap_voxel > 0.0 && !merged->empty()) {
    PointCloudT::Ptr filtered(new PointCloudT());
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(static_cast<float>(params_.submap_voxel),
                      static_cast<float>(params_.submap_voxel),
                      static_cast<float>(params_.submap_voxel));
    voxel.setInputCloud(merged);
    voxel.filter(*filtered);
    merged.swap(filtered);
  }

  out->id = next_id_++;
  out->stamp = latest.stamp;
  out->pose_raw = latest.pose;
  out->pose_opt = latest.pose;
  out->cloud = merged;
  // descriptor / ring_key 由节点用 ScanContext 填。

  last_kf_pose_ = latest.pose;
  last_kf_stamp_ = latest.stamp;
  has_kf_ = true;
  ++keyframe_count_;
  return true;
}

void KeyframeManager::reset() {
  frames_.clear();
  last_kf_pose_.setIdentity();
  last_kf_stamp_ = -1.0;
  has_kf_ = false;
  next_id_ = 0;
  keyframe_count_ = 0;
}

}  // namespace point_lio_backend
