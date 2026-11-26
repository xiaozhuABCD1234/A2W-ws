#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "relocalization/types.hpp"

namespace relocalization {

class VoxelIndex {
 public:
  void build(const PointCloudT& cloud, double resolution);
  void clear();

  double resolution() const { return resolution_; }
  size_t size() const { return count_; }
  bool empty() const { return count_ == 0; }
  float originX() const { return origin_[0]; }
  float originY() const { return origin_[1]; }
  float originZ() const { return origin_[2]; }
  float inverseResolution() const { return inv_; }

  void coords(float x, float y, float z, int64_t* cx, int64_t* cy, int64_t* cz) const {
    *cx = static_cast<int64_t>(std::floor((x - origin_[0]) * inv_));
    *cy = static_cast<int64_t>(std::floor((y - origin_[1]) * inv_));
    *cz = static_cast<int64_t>(std::floor((z - origin_[2]) * inv_));
  }

  bool occupiedCell(int64_t cx, int64_t cy, int64_t cz) const {
    const int64_t key = packKey(cx, cy, cz);
    uint64_t slot = mix(key) & mask_;
    for (;;) {
      const int64_t value = slots_[slot];
      if (value == 0) return false;
      if (value == key + 1) return true;
      slot = (slot + 1) & mask_;
    }
  }

  bool occupied(float x, float y, float z) const {
    int64_t cx = 0;
    int64_t cy = 0;
    int64_t cz = 0;
    coords(x, y, z, &cx, &cy, &cz);
    return occupiedCell(cx, cy, cz);
  }

  bool occupied(const Eigen::Vector3f& point) const {
    return occupied(point.x(), point.y(), point.z());
  }

 private:
  static int64_t packKey(int64_t cx, int64_t cy, int64_t cz) {
    const int64_t bits = 21;
    const int64_t low = (int64_t(1) << bits) - 1;
    return ((cx & low) << (2 * bits)) | ((cy & low) << bits) | (cz & low);
  }

  static uint64_t mix(int64_t key) {
    uint64_t value = static_cast<uint64_t>(key) + 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31);
  }

  double resolution_ = 0.5;
  float inv_ = 2.0f;
  float origin_[3] = {0.0f, 0.0f, 0.0f};
  std::vector<int64_t> slots_;
  uint64_t mask_ = 0;
  size_t count_ = 0;
};

}
