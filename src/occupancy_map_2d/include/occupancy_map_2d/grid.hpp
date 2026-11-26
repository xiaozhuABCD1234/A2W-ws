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
  int min_component_cells = 3;
};

class OccupancyGrid2D {
 public:
  explicit OccupancyGrid2D(const GridParams& params);

  void clear();
  void update(const PointCloudT& ground, const PointCloudT& obstacles,
              const Eigen::Isometry3d& sensor_to_map);
  nav_msgs::msg::OccupancyGrid message(const std::string& frame_id,
                                       const rclcpp::Time& stamp) const;
  size_t updatedCells() const { return updated_cells_; }
  size_t occupiedCells() const;

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return params_.resolution; }

 private:
  bool index(double x, double y, int* cell_x, int* cell_y) const;
  void raycast(int start_x, int start_y, int end_x, int end_y, bool clear_endpoint,
               const std::vector<uint8_t>* blocked, float occupied_threshold);
  void addFree(int x, int y);
  void addOccupied(int x, int y);
  double probability(int index) const;

  GridParams params_;
  int width_ = 0;
  int height_ = 0;
  std::vector<float> log_odds_;
  std::vector<uint8_t> observed_;
  std::vector<uint8_t> ground_mask_;
  std::vector<uint8_t> obstacle_mask_;
  std::vector<int> ground_touched_;
  std::vector<int> obstacle_touched_;
  size_t updated_cells_ = 0;
};

}
