#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>

namespace ground_segmentation {

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloudT::Ptr;

struct SegmentationResult {
  PointCloudPtr ground = std::make_shared<PointCloudT>();
  PointCloudPtr nonground = std::make_shared<PointCloudT>();
  size_t finite_points = 0;
};

class GroundSegmenter {
 public:
  virtual ~GroundSegmenter() = default;

  virtual std::string name() const = 0;

  virtual SegmentationResult segment(const PointCloudT& cloud,
                                     const Eigen::Matrix3d& gravity_rotation) = 0;
};

}
