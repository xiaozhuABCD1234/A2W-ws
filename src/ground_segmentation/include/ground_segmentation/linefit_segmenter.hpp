#pragma once

#include "ground_segmentation/ground_segmenter.hpp"

namespace ground_segmentation {

struct LineFitParams {
  double min_range = 0.5;
  double max_range = 30.0;
  double ring_size = 0.5;
  int sectors = 360;

  int num_lpr = 20;
  int num_iter = 3;
  int min_seed_points = 3;
  double th_seeds = 0.20;
  double th_dist = 0.10;
  double uprightness_thr = 0.707;
  double max_slope_deg = 20.0;
  double max_ground_step = 0.15;
  double initial_ground_tolerance = 0.8;
  double ground_bottom_percentile = 0.05;
};

class LineFitGroundSegmenter : public GroundSegmenter {
 public:
  explicit LineFitGroundSegmenter(const LineFitParams& params);

  std::string name() const override { return "linefit"; }

  SegmentationResult segment(const PointCloudT& cloud,
                             const Eigen::Matrix3d& gravity_rotation) override;

 private:
  LineFitParams params_;
  int rings_ = 1;
  double max_slope_ = 0.0;
};

}
