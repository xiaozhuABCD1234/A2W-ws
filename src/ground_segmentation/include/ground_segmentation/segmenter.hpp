#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

namespace ground_segmentation {

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloudT::Ptr;

struct SegmentationParams {
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
  double elevated_height = 2.0;
  double ground_estimation_range = 10.0;
};

struct SegmentationResult {
  PointCloudPtr ground = PointCloudPtr(new PointCloudT());
  PointCloudPtr obstacles = PointCloudPtr(new PointCloudT());
  PointCloudPtr elevated = PointCloudPtr(new PointCloudT());
  size_t finite_points = 0;
};

class PolarGroundSegmenter {
 public:
  explicit PolarGroundSegmenter(const SegmentationParams& params);
  SegmentationResult segment(const PointCloudT& cloud,
                             const Eigen::Matrix3d& gravity_rotation) const;

 private:
  SegmentationParams params_;
  int rings_ = 1;
};

}
