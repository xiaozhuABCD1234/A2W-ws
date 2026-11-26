#include "relocalization/voxel_index.hpp"

#include <algorithm>
#include <utility>

namespace relocalization {

void VoxelIndex::clear() {
  slots_.clear();
  slots_.shrink_to_fit();
  mask_ = 0;
  count_ = 0;
}

void VoxelIndex::build(const PointCloudT& cloud, double resolution) {
  clear();
  resolution_ = std::max(1.0e-3, resolution);
  inv_ = static_cast<float>(1.0 / resolution_);
  origin_[0] = 0.0f;
  origin_[1] = 0.0f;
  origin_[2] = 0.0f;
  if (cloud.empty()) return;

  size_t capacity = 64;
  const size_t wanted = cloud.size() * 2;
  while (capacity < wanted) capacity <<= 1;
  slots_.assign(capacity, 0);
  mask_ = static_cast<uint64_t>(capacity - 1);

  for (const auto& point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    int64_t cx = 0;
    int64_t cy = 0;
    int64_t cz = 0;
    coords(point.x, point.y, point.z, &cx, &cy, &cz);
    const int64_t key = packKey(cx, cy, cz);
    uint64_t slot = mix(key) & mask_;
    for (;;) {
      const int64_t value = slots_[slot];
      if (value == key + 1) break;
      if (value == 0) {
        slots_[slot] = key + 1;
        ++count_;
        break;
      }
      slot = (slot + 1) & mask_;
    }
  }
}

}
