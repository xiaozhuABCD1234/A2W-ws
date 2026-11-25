#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include "point_lio_backend/types.hpp"

namespace point_lio_backend {

// PCM-lite（Mangelson 等 Pairwise Consistent Measurement 的简化版）：
// 只做回环-回环的两两一致性贪心筛选（近似极大团），不与里程计比较——
// 里程计自身漂移，单回环对里程计的检查会误杀真回环，这正是 PCM 的核心洞察。
//
// 输入 active 为已做过结构剪枝（端点存在、非自环）的初值；输出只会把
// 不一致者置 false，不会重新打开。pcm_enabled == false 时原样返回。
std::vector<bool> pcmFilter(const std::vector<Keyframe>& keyframes,
                            const std::unordered_map<int, std::size_t>& id_to_index,
                            const std::vector<LoopConstraint>& loops,
                            const std::vector<bool>& input_active,
                            const GraphParams& params);

}  // namespace point_lio_backend
