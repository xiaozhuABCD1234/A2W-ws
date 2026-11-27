#ifndef FILTER__PEDESTRIAN_FILTER_HPP_
#define FILTER__PEDESTRIAN_FILTER_HPP_

#include "filter/voxel_filter.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace filter {

struct PedestrianParams {
  double cell_size = 0.30;
  int min_points = 20;
  double max_width = 1.20;
  double max_depth = 1.20;
  double min_extent = 0.70;
  double max_extent = 2.20;
  double max_footprint = 1.40;
  double min_aspect = 2.00;
  bool keep_unmatched = true;
};

struct PedestrianStats {
  FrameStats voxel;
  size_t clusters = 0;
  size_t pedestrian_clusters = 0;
  size_t rejected_clusters = 0;
  size_t pedestrian_points = 0;
};

class PedestrianFilter {
 public:
  PedestrianFilter(const FilterParams& filter_params, const PedestrianParams& pedestrian_params);

  void reset();

  PedestrianStats filter(const std::vector<Eigen::Vector3f>& points,
                         const Eigen::Vector3f& origin,
                         std::vector<uint8_t>* pedestrian_mask);

  size_t mapVoxels() const { return voxels_.mapVoxels(); }

  const PedestrianParams& params() const { return params_; }

 private:
  struct Cluster {
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    double min_z = 0.0;
    double max_z = 0.0;
    size_t points = 0;
  };

  bool accepts(const Cluster& cluster) const;

  VoxelFilter voxels_;
  PedestrianParams params_;
  std::vector<int32_t> candidates_;
  std::vector<Cluster> clusters_;
  std::vector<int32_t> parent_;
  std::vector<int32_t> cell_head_;
  std::vector<uint8_t> candidate_mask_;
};

}

#endif
