#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace occupancy_map_2d {

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

struct GridParams {
  double resolution = 0.10;
  double width = 120.0;
  double height = 120.0;
  double origin_x = -60.0;
  double origin_y = -60.0;
  double max_range = 20.0;
  double obstacle_min_height = -0.4;
  double obstacle_max_height = 2.0;
  double occupied_log_odds = 0.85;
  double free_log_odds = -0.12;
  double min_log_odds = -5.0;
  double max_log_odds = 5.0;
  double occupied_probability = 0.65;
  double ray_block_probability = 0.90;
  double free_ray_occupied_delta = 0.30;
  int occupied_patience_frames = 5;
  int min_component_cells = 3;

  double raycast_dedupe_deg = 0.5;
};

struct GridSnapshot {
  double resolution = 0.1;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double occupied_probability = 0.65;
  double ray_block_probability = 0.90;
  double free_ray_occupied_delta = 0.30;
  int occupied_patience_frames = 5;
  int min_component_cells = 3;
  int width = 0;
  int height = 0;
  std::vector<int32_t> cells;
  std::vector<float> log_odds;
};

class OccupancyGrid2D {
 public:
  explicit OccupancyGrid2D(const GridParams& params);

  void clear();
  void update(const PointCloudT& ground, const PointCloudT& nonground,
              const Eigen::Isometry3d& sensor_to_map);

  void takeSnapshot(GridSnapshot* snapshot) const;

  static nav_msgs::msg::OccupancyGrid toMessage(const GridSnapshot& snapshot,
                                                const std::string& frame_id,
                                                const rclcpp::Time& stamp);

  nav_msgs::msg::OccupancyGrid message(const std::string& frame_id,
                                       const rclcpp::Time& stamp) const;

  size_t updatedCells() const { return observed_cells_.size(); }
  size_t occupiedCells() const { return occupied_cells_; }

  uint64_t version() const { return version_; }

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return params_.resolution; }

 private:
  bool index(double x, double y, int* cell_x, int* cell_y) const;
  void raycast(int start_x, int start_y, int end_x, int end_y, bool clear_endpoint,
               const std::vector<uint8_t>* blocked);
  void addFree(int x, int y);
  void erode(int x, int y);
  void addOccupied(int x, int y);
  const std::vector<int>& dedupeGroundRays(int sensor_x, int sensor_y);

  GridParams params_;
  int width_ = 0;
  int height_ = 0;
  float occupied_threshold_ = 0.0f;
  float free_delta_ = 0.0f;
  float occupied_delta_ = 0.0f;
  float min_log_odds_ = 0.0f;
  float max_log_odds_ = 0.0f;
  float ray_block_threshold_ = 0.0f;
  float free_ray_occupied_delta_ = 0.30f;
  uint32_t frame_ = 0;
  uint32_t occupied_patience_ = 5;
  double max_range2_ = 0.0;

  std::vector<float> log_odds_;
  std::vector<uint8_t> observed_;
  std::vector<uint8_t> ground_mask_;
  std::vector<uint8_t> obstacle_mask_;
  std::vector<uint32_t> obstacle_frame_;

  std::vector<int> observed_cells_;
  std::vector<int> ground_touched_;
  std::vector<int> obstacle_touched_;
  std::vector<int> ground_cells_;
  std::vector<int> obstacle_cells_;
  std::vector<int> ground_ray_cells_;
  std::vector<int32_t> bucket_dist2_;
  std::vector<int32_t> bucket_cell_;

  size_t occupied_cells_ = 0;
  uint64_t version_ = 0;
};

}
