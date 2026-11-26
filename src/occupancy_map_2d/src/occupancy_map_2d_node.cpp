#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "occupancy_map_2d/grid.hpp"

namespace occupancy_map_2d {
namespace {

class OccupancyMap2DNode : public rclcpp::Node {
 public:
  OccupancyMap2DNode() : Node("occupancy_map_2d"), tf_buffer_(get_clock()) {
    loadParameters();
    grid_ = std::make_unique<OccupancyGrid2D>(params_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    ground_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        ground_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { groundCallback(msg); });
    obstacles_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        obstacles_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { obstaclesCallback(msg); });
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).transient_local().reliable());
    clear_service_ = create_service<std_srvs::srv::Trigger>(
        "~/clear",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(grid_mutex_);
          grid_->clear();
          map_pub_->publish(grid_->message(world_frame_, now()));
          response->success = true;
          response->message = "occupancy grid cleared";
        });
    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_), [this]() { publishMap(); });
    RCLCPP_INFO(get_logger(), "occupancy map started: %dx%d cells at %.3f m",
                grid_->width(), grid_->height(), grid_->resolution());
  }

 private:
  template <typename T>
  T param(const std::string& name, const T& default_value) {
    return declare_parameter<T>(name, default_value);
  }

  void loadParameters() {
    world_frame_ = param<std::string>("world_frame", world_frame_);
    ground_topic_ = param<std::string>("ground_topic", ground_topic_);
    obstacles_topic_ = param<std::string>("obstacles_topic", obstacles_topic_);
    map_topic_ = param<std::string>("map_topic", map_topic_);
    stamp_tolerance_ = param<double>("runtime.stamp_tolerance", stamp_tolerance_);
    publish_period_ms_ = param<int>("runtime.publish_period_ms", publish_period_ms_);
    tf_timeout_ms_ = param<int>("runtime.tf_timeout_ms", tf_timeout_ms_);
    log_period_ms_ = param<int>("runtime.log_period_ms", log_period_ms_);
    params_.resolution = param<double>("grid.resolution", params_.resolution);
    params_.width = param<double>("grid.width", params_.width);
    params_.height = param<double>("grid.height", params_.height);
    params_.origin_x = param<double>("grid.origin_x", params_.origin_x);
    params_.origin_y = param<double>("grid.origin_y", params_.origin_y);
    params_.max_range = param<double>("grid.max_range", params_.max_range);
    params_.obstacle_min_height =
        param<double>("grid.obstacle_min_height", params_.obstacle_min_height);
    params_.obstacle_max_height =
        param<double>("grid.obstacle_max_height", params_.obstacle_max_height);
    params_.occupied_log_odds =
        param<double>("grid.occupied_log_odds", params_.occupied_log_odds);
    params_.free_log_odds = param<double>("grid.free_log_odds", params_.free_log_odds);
    params_.min_log_odds = param<double>("grid.min_log_odds", params_.min_log_odds);
    params_.max_log_odds = param<double>("grid.max_log_odds", params_.max_log_odds);
    params_.occupied_probability =
        param<double>("grid.occupied_probability", params_.occupied_probability);
    params_.min_component_cells =
        param<int>("grid.min_component_cells", params_.min_component_cells);
    publish_period_ms_ = std::max(50, publish_period_ms_);
    tf_timeout_ms_ = std::max(0, tf_timeout_ms_);
  }

  void groundCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    try {
      auto cloud = std::make_shared<PointCloudT>();
      pcl::fromROSMsg(*msg, *cloud);
      std::lock_guard<std::mutex> lock(input_mutex_);
      latest_ground_ = cloud;
      latest_ground_header_ = msg->header;
    } catch (const std::exception& exception) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "ground cloud conversion failed: %s", exception.what());
    }
  }

  void obstaclesCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    try {
      std::shared_ptr<PointCloudT> ground;
      {
        std::lock_guard<std::mutex> lock(input_mutex_);
        if (!latest_ground_ || std::abs(rclcpp::Time(msg->header.stamp).seconds() -
                                        rclcpp::Time(latest_ground_header_.stamp).seconds()) >
                                   stamp_tolerance_) {
          return;
        }
        ground = latest_ground_;
      }
      PointCloudT obstacles;
      pcl::fromROSMsg(*msg, obstacles);
      const auto transform = tf_buffer_.lookupTransform(
          world_frame_, msg->header.frame_id, msg->header.stamp,
          tf2::durationFromSec(static_cast<double>(tf_timeout_ms_) / 1000.0));
      const Eigen::Isometry3d sensor_to_map = tf2::transformToEigen(transform.transform);
      const auto start = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        grid_->update(*ground, obstacles, sensor_to_map);
      }
      if (log_period_ms_ > 0) {
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_period_ms_,
                             "updated=%zu occupied=%zu ground=%zu obstacles=%zu time=%.2f ms",
                             grid_->updatedCells(), grid_->occupiedCells(), ground->size(),
                             obstacles.size(), elapsed_ms);
      }
    } catch (const std::exception& exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "occupancy update skipped: %s", exception.what());
    }
  }

  void publishMap() {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    map_pub_->publish(grid_->message(world_frame_, now()));
  }

  GridParams params_;
  std::unique_ptr<OccupancyGrid2D> grid_;
  std::string world_frame_ = "map";
  std::string ground_topic_ = "/ground_segmentation/ground";
  std::string obstacles_topic_ = "/ground_segmentation/obstacles";
  std::string map_topic_ = "/occupancy_map_2d/map";
  double stamp_tolerance_ = 0.05;
  int publish_period_ms_ = 1000;
  int tf_timeout_ms_ = 50;
  int log_period_ms_ = 5000;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::mutex input_mutex_;
  std::shared_ptr<PointCloudT> latest_ground_;
  std_msgs::msg::Header latest_ground_header_;
  std::mutex grid_mutex_;
};

}
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<occupancy_map_2d::OccupancyMap2DNode>());
  rclcpp::shutdown();
  return 0;
}
