#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace relocalization {

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloudT::Ptr;
using PointCloudConstPtr = PointCloudT::ConstPtr;

enum class TiltMode { Odom, Cone, Free };

struct RelocalizationParams {
  std::string map_path = "PCD/backend_map.pcd";
  bool auto_start = true;
  int confirmation_frames = 3;
  int max_attempts = 30;
  int submap_frames = 4;
  double submap_voxel = 0.2;
  double min_frame_interval = 0.5;
  double stamp_tolerance = 0.05;

  TiltMode tilt_mode = TiltMode::Odom;
  double tilt_deg = 15.0;
  double tilt_step_deg = 7.5;

  std::vector<double> pyramid_voxels{2.0, 1.0, 0.5, 0.25};
  std::vector<int> level_keep{240, 60, 24, 12};
  double coarse_yaw_step_deg = 45.0;
  double level_keep_ratio = 0.4;
  double level_min_score = 0.10;
  bool auto_z_range = true;
  double z_min = 0.0;
  double z_max = 0.0;
  double z_margin = 0.0;
  int max_query_points = 400;
  int coarse_query_points = 200;
  double min_candidate_separation = 1.0;
  double min_candidate_rotation_deg = 15.0;

  int gicp_candidates = 8;
  double gicp_voxel = 0.25;
  int gicp_max_points = 6000;
  int gicp_iterations = 40;
  double gicp_max_correspondence = 1.5;
  double validation_correspondence = 0.5;
  double max_fitness = 0.08;
  double min_inlier_ratio = 0.45;
  double max_refinement_trans = 1.5;
  double max_refinement_rot_deg = 25.0;
  double max_tilt_change_deg = 20.0;
  double min_score_margin = 0.05;
  double min_ambiguity_margin = 0.01;
  double ambiguity_translation = 2.0;
  double ambiguity_rotation_deg = 20.0;
  double confirmation_trans_tolerance = 1.0;
  double confirmation_rot_tolerance_deg = 10.0;
};

}
