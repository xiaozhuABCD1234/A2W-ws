#ifndef FILTER__VOXEL_FILTER_HPP_
#define FILTER__VOXEL_FILTER_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace filter {

struct FilterParams {
  double voxel_size = 0.20;
  double min_range = 0.5;
  double max_range = 20.0;
  double min_height = -2.0;
  double max_height = 3.0;
  double azimuth_step_deg = 2.0;
  double elevation_step_deg = 2.0;
  int max_rays_per_frame = 6000;
  int free_frames_threshold = 4;
  int free_label_score = -3;
  int occupied_score = 2;
  int free_score = 1;
  int min_score = -6;
  int max_score = 6;
  int occupied_recent_gain = 4;
  double cluster_radius = 0.6;
  int relearn_frames = 30;
  int decay_period_frames = 300;
  double max_voxels = 400000;
};

struct FrameStats {
  size_t input_points = 0;
  size_t considered_points = 0;
  size_t dynamic_points = 0;
  size_t dynamic_voxels = 0;
  size_t map_voxels = 0;
  size_t rays = 0;
};

class VoxelFilter {
 public:
  explicit VoxelFilter(const FilterParams& params);

  void reset();

  FrameStats filter(const std::vector<Eigen::Vector3f>& points,
                    const Eigen::Vector3f& origin,
                    std::vector<uint8_t>* dynamic_mask);

  size_t mapVoxels() const { return voxels_.size(); }

  const FilterParams& params() const { return params_; }

 private:
  struct Voxel {
    int16_t score = 0;
    uint16_t free_frames = 0;
    uint16_t occupied_frames = 0;
    uint8_t occupied_recent = 0;
    uint8_t dynamic_streak = 0;
    uint32_t free_frame = 0;
    uint32_t occupied_frame = 0;
  };

  struct Bin {
    int32_t index = -1;
    float range2 = 0.0f;
  };

  struct Cell {
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
  };

  static int64_t pack(int32_t x, int32_t y, int32_t z);
  static Cell unpack(int64_t key);

  Voxel& touch(int32_t x, int32_t y, int32_t z);
  Voxel* find(int32_t x, int32_t y, int32_t z);
  bool knownFree(const Voxel& voxel) const;
  void applyFree(int32_t x, int32_t y, int32_t z);
  void applyOccupied(Voxel& voxel);
  void castFreeRay(const Eigen::Vector3f& origin, const Eigen::Vector3f& target);
  void decay();
  void forget();

  FilterParams params_;
  std::unordered_map<int64_t, Voxel> voxels_;
  std::unordered_set<int64_t> dynamic_cells_;
  std::unordered_set<int64_t> blocked_cells_;
  std::vector<int32_t> considered_;
  std::vector<Cell> cells_;
  std::vector<Bin> bins_;
  std::vector<int32_t> touched_bins_;
  uint32_t frame_id_ = 0;
  int azimuth_bins_ = 1;
  int elevation_bins_ = 1;
  float voxel_size_ = 0.2f;
  float min_range2_ = 0.0f;
  float max_range2_ = 400.0f;
};

}

#endif
