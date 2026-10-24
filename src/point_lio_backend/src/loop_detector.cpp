#include "point_lio_backend/loop_detector.hpp"

namespace point_lio_backend {

LoopDetector::LoopDetector(const ScanContextParams& sc_params, const IcpParams& icp_params)
    : scan_context_(sc_params), sc_params_(sc_params), icp_params_(icp_params) {}

// TODO(M2): ring-key 粗筛 + SC 精筛 + GICP 校验。
std::optional<LoopConstraint> LoopDetector::detect(
    const Keyframe& current, const std::vector<Keyframe>& history) const {
  (void)current;
  (void)history;
  return std::nullopt;
}

}  // namespace point_lio_backend
