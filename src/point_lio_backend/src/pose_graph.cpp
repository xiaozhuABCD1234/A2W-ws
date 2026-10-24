#include "point_lio_backend/pose_graph.hpp"

namespace point_lio_backend {

PoseGraph::PoseGraph(const GraphParams& params) : params_(params) {}

// TODO(M3): Ceres 6-DOF 位姿图 + 外点门控。当前退化为“优化位姿 = 原始位姿”。
bool PoseGraph::optimize(std::vector<Keyframe>& keyframes,
                         const std::vector<LoopConstraint>& loops,
                         std::vector<bool>* loop_active) const {
  (void)loops;
  if (loop_active) loop_active->assign(loops.size(), false);
  for (auto& kf : keyframes) {
    kf.pose_opt = kf.pose_raw;
  }
  return true;
}

}  // namespace point_lio_backend
