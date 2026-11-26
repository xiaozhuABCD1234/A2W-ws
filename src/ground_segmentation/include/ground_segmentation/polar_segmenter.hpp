#pragma once

#include "ground_segmentation/ground_segmenter.hpp"

namespace ground_segmentation {

struct PolarParams {
  double min_range = 0.5;
  double max_range = 30.0;
  double ring_size = 0.5;
  int sectors = 120;
  int min_cell_points = 1;
  double seed_height_band = 0.15;
  double ground_height_threshold = 0.12;
  double max_ground_step = 0.15;
  double max_slope_deg = 20.0;
  double ground_alpha = 0.65;
  double initial_ground_tolerance = 0.8;
  double ground_estimation_range = 10.0;
};

class PolarGroundSegmenter : public GroundSegmenter {
 public:
  explicit PolarGroundSegmenter(const PolarParams& params);

  std::string name() const override { return "polar"; }

  SegmentationResult segment(const PointCloudT& cloud,
                             const Eigen::Matrix3d& gravity_rotation) override;

 private:
  PolarParams params_;
  int rings_ = 1;
};

}
