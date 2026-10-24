#pragma once

#include <vector>

#include "point_lio_backend/types.hpp"

namespace point_lio_backend {

// Ceres 6-DOF 位姿图。输入关键帧的 pose_raw（odom 系）与回环约束，
// 输出写回 keyframes[i].pose_opt（map 系）。首节点固定，map 原点 = 首关键帧。
class PoseGraph {
 public:
  explicit PoseGraph(const GraphParams& params);

  bool optimize(std::vector<Keyframe>& keyframes,
                const std::vector<LoopConstraint>& loops,
                std::vector<bool>* loop_active = nullptr) const;

 private:
  GraphParams params_;
};

}  // namespace point_lio_backend
