#pragma once

#include <optional>
#include <vector>

#include "point_lio_backend/scan_context.hpp"
#include "point_lio_backend/types.hpp"

namespace point_lio_backend {

// 回环检测：ring-key 粗筛 -> Scan Context 精筛 -> GICP 校验。
class LoopDetector {
 public:
  LoopDetector(const ScanContextParams& sc_params, const IcpParams& icp_params);

  // current 为最新关键帧（其 descriptor/ring_key 由调用方填好），
  // history 为全部历史关键帧（不含 current）。检测不到返回 nullopt。
  std::optional<LoopConstraint> detect(const Keyframe& current,
                                       const std::vector<Keyframe>& history) const;

 private:
  ScanContext scan_context_;
  ScanContextParams sc_params_;
  IcpParams icp_params_;
};

}  // namespace point_lio_backend
