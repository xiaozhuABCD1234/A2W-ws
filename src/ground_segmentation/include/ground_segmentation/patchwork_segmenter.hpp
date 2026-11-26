#pragma once

#include <vector>

#include "ground_segmentation/ground_segmenter.hpp"

namespace ground_segmentation {

struct PatchworkParams {
  double min_range = 0.5;
  double max_range = 30.0;

  std::vector<double> zone_ranges = {0.0, 4.0, 8.0, 14.0, 30.0};
  std::vector<int64_t> sectors_per_zone = {16, 32, 54, 32};
  std::vector<int64_t> rings_per_zone = {2, 4, 4, 4};

  int num_iter = 3;
  int num_lpr = 20;
  int min_bucket_points = 5;
  double th_seeds = 0.30;
  double th_dist = 0.125;

  double uprightness_thr = 0.707;
  double flatness_thr = 0.05;
  double max_ground_height = 1.50;

  double ground_bottom_percentile = 0.05;

  bool enable_temporal_revert = true;
  double temporal_revert_ratio = 0.5;
};

class PatchworkGroundSegmenter : public GroundSegmenter {
 public:
  explicit PatchworkGroundSegmenter(const PatchworkParams& params);

  std::string name() const override { return "patchwork"; }

  SegmentationResult segment(const PointCloudT& cloud,
                             const Eigen::Matrix3d& gravity_rotation) override;

 private:
  struct Plane {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double offset = 0.0;
    double flatness = 1.0;
    bool valid = false;
  };

  bool fitPlane(const std::vector<int>& indices,
                const std::vector<Eigen::Vector3d>& points, Plane* plane) const;

  PatchworkParams params_;
  int num_zones_ = 0;
  std::vector<int> bucket_offset_;
  std::vector<Plane> previous_;
};

}
