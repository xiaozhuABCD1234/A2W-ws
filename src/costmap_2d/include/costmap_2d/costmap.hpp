#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <vector>

namespace costmap_2d {

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

struct CostmapParams {
  double resolution = 0.05;
  double width = 10.0;
  double height = 10.0;
  double recenter_distance = 0.6;
  double max_range = 8.0;
  double clear_min_z = -0.30;
  double clear_max_z = 1.30;
  double obstacle_min_z = 0.05;
  double obstacle_max_z = 1.50;
  double occupied_log_odds = 0.85;
  double free_log_odds = -0.12;
  double min_log_odds = -3.0;
  double max_log_odds = 3.5;
  double ray_block_probability = 0.90;
  double free_ray_occupied_delta = 0.25;
  bool ground_relative = true;
  double ground_radius = 4.0;
  double ground_percentile = 0.15;
  double ground_smoothing = 0.3;
  double ground_max_step = 0.25;
  double blind_hold_sec = 2.0;
  double occupied_decay = 0.12;
  double free_decay = 0.02;
  int min_component_cells = 2;
  double raycast_dedupe_deg = 0.5;
};

struct CostmapSnapshot {
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  int width = 0;
  int height = 0;
  std::vector<int8_t> cells;
};

class RollingCostmap {
 public:
  explicit RollingCostmap(const CostmapParams& params);

  void reset(double robot_x, double robot_y);

  void update(const PointCloudT& ground, const PointCloudT& nonground,
              const Eigen::Isometry3d& sensor_to_map, double now_sec);

  void decay(double now_sec);

  void takeSnapshot(CostmapSnapshot* snapshot) const;

  static nav_msgs::msg::OccupancyGrid toMessage(const CostmapSnapshot& snapshot,
                                                const std::string& frame_id,
                                                const rclcpp::Time& stamp);

  bool needsRecenter(double robot_x, double robot_y) const;

  void recenter(double robot_x, double robot_y);

  size_t occupiedCells() const { return occupied_cells_; }

  double groundZ() const { return ground_z_; }

  bool hasGround() const { return ground_valid_; }

  uint64_t version() const { return version_; }

  double resolution() const { return params_.resolution; }

  int width() const { return width_; }

  int height() const { return height_; }

  double originX() const { return origin_x_; }

  double originY() const { return origin_y_; }

 private:
  void allocate(int width, int height, double origin_x, double origin_y);
  bool index(double x, double y, int* cell_x, int* cell_y) const;
  size_t position(int x, int y) const {
    return static_cast<size_t>(y) * static_cast<size_t>(width_) + static_cast<size_t>(x);
  }
  void addFree(int x, int y, double now_sec);
  void addOccupied(int x, int y, double now_sec);
  void castRay(int start_x, int start_y, int end_x, int end_y, double z_start, double z_end,
               double now_sec);
  void clearGround(const PointCloudT& ground, const Eigen::Isometry3d& sensor_to_map,
                   double now_sec);
  void updateGroundLevel(const PointCloudT& ground, const Eigen::Isometry3d& sensor_to_map);
  void castObstacles(const PointCloudT& nonground, const Eigen::Isometry3d& sensor_to_map,
                     double now_sec);
  size_t countOccupied() const;

  CostmapParams params_;
  int width_ = 0;
  int height_ = 0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  float occupied_threshold_ = 0.0f;
  float ray_block_threshold_ = 0.0f;
  float free_ray_occupied_delta_ = 0.25f;
  float min_log_odds_ = -3.0f;
  float max_log_odds_ = 3.5f;
  float occupied_delta_ = 0.85f;
  float free_delta_ = 0.12f;
  float free_decay_ = 0.02f;
  float occupied_decay_ = 0.12f;
  double max_range2_ = 0.0;

  std::vector<float> log_odds_;
  std::vector<float> last_seen_;

  double ground_z_ = 0.0;
  bool ground_valid_ = false;
  double ground_smoothing_ = 0.3;
  float clear_min_z_ = -0.30f;
  float clear_max_z_ = 1.30f;
  float obstacle_min_z_ = 0.05f;
  float obstacle_max_z_ = 1.50f;

  size_t occupied_cells_ = 0;
  uint64_t version_ = 0;
};

}
