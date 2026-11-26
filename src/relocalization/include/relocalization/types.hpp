#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace relocalization {

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloudT::Ptr;
using PointCloudConstPtr = PointCloudT::ConstPtr;

struct RelocalizationParams {
  std::string map_path = "PCD/backend_map.pcd";
  bool auto_start = true;
  int confirmation_frames = 3;
  int max_attempts = 30;
  int submap_frames = 4;
  double submap_voxel = 0.2;
  double min_frame_interval = 0.5;
  double stamp_tolerance = 0.05;
  bool gravity_align = true;
  std::vector<double> pyramid_voxels{1.5, 0.75, 0.35};
  double coarse_xy_step = 2.0;
  double coarse_z_step = 1.0;
  double coarse_yaw_step_deg = 30.0;
  double search_z_min = -1.0;
  double search_z_max = 1.0;
  int max_candidates = 6;
  int max_query_points = 400;
  bool filter_ground = true;
  double ground_distance_threshold = 0.10;
  double gicp_voxel = 0.25;
  int gicp_max_points = 3000;
  int gicp_iterations = 50;
  double gicp_max_correspondence = 1.5;
  double validation_correspondence = 0.5;
  double max_fitness = 0.08;
  double min_inlier_ratio = 0.45;
  double max_roll_pitch_deg = 12.0;
  double max_refinement_trans = 3.0;
  double max_refinement_rot_deg = 35.0;
  double min_ambiguity_margin = 0.01;
  double ambiguity_translation = 3.0;
  double ambiguity_rotation_deg = 30.0;
  double confirmation_trans_tolerance = 1.0;
  double confirmation_rot_tolerance_deg = 10.0;
};

}
