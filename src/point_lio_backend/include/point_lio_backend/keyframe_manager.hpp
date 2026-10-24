#pragma once

#include <deque>

#include "point_lio_backend/types.hpp"

namespace point_lio_backend {

// 关键帧采样与子图拼接。addFrame() 缓存最近的机体系点云；
// shouldCreate() 判断是否该出新关键帧；buildNextKeyframe() 生成子图。
class KeyframeManager {
 public:
  explicit KeyframeManager(const KeyframeParams& params);

  void addFrame(double stamp, const Eigen::Isometry3d& pose_raw,
                const PointCloudPtr& cloud_body);
  bool shouldCreate(double stamp, const Eigen::Isometry3d& pose_raw) const;
  bool buildNextKeyframe(Keyframe* out);
  void reset();

  size_t frameCount() const { return frames_.size(); }
  size_t keyframeCount() const { return keyframe_count_; }

 private:
  struct Frame {
    double stamp = 0.0;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    PointCloudPtr cloud;
  };

  KeyframeParams params_;
  std::deque<Frame> frames_;
  Eigen::Isometry3d last_kf_pose_ = Eigen::Isometry3d::Identity();
  double last_kf_stamp_ = -1.0;
  bool has_kf_ = false;
  int next_id_ = 0;
  size_t keyframe_count_ = 0;
};

}  // namespace point_lio_backend
