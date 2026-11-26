#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
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

    io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    output_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = io_group_;
    ground_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        ground_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { groundCallback(msg); },
        sub_options);
    obstacles_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        obstacles_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { obstaclesCallback(msg); },
        sub_options);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).transient_local().reliable());
    clear_service_ = create_service<std_srvs::srv::Trigger>(
        "~/clear",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          {
            std::lock_guard<std::mutex> lock(grid_mutex_);
            grid_->clear();
          }
          publishMap(true);
          response->success = true;
          response->message = "occupancy grid cleared";
        },
        rclcpp::ServicesQoS(), output_group_);
    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_), [this]() { publishMap(false); },
        output_group_);
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
    params_.raycast_dedupe_deg =
        param<double>("grid.raycast_dedupe_deg", params_.raycast_dedupe_deg);
    publish_period_ms_ = std::max(50, publish_period_ms_);

    tf_timeout_ms_ = std::max(0, tf_timeout_ms_);
    params_.raycast_dedupe_deg = std::max(0.0, params_.raycast_dedupe_deg);
  }

  void groundCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    auto cloud = std::make_shared<PointCloudT>();
    try {
      pcl::fromROSMsg(*msg, *cloud);
    } catch (const std::exception& exception) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "ground cloud conversion failed: %s", exception.what());
      return;
    }
    std::lock_guard<std::mutex> lock(input_mutex_);
    ground_ = std::move(cloud);
    ground_stamp_ = rclcpp::Time(msg->header.stamp);
    matchAndUpdate();
  }

  void obstaclesCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    auto cloud = std::make_shared<PointCloudT>();
    try {
      pcl::fromROSMsg(*msg, *cloud);
    } catch (const std::exception& exception) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "obstacle cloud conversion failed: %s", exception.what());
      return;
    }
    std::lock_guard<std::mutex> lock(input_mutex_);
    obstacles_ = std::move(cloud);
    obstacles_stamp_ = rclcpp::Time(msg->header.stamp);
    obstacles_frame_id_ = msg->header.frame_id;
    matchAndUpdate();
  }

  void matchAndUpdate() {
    if (!ground_ || !obstacles_) return;

    const double delta = (obstacles_stamp_ - ground_stamp_).seconds();
    if (std::abs(delta) > stamp_tolerance_) {
      if (delta > 0.0) {
        ground_.reset();
      } else {
        obstacles_.reset();
      }
      return;
    }

    const std::shared_ptr<PointCloudT> ground = std::move(ground_);
    const std::shared_ptr<PointCloudT> obstacles = std::move(obstacles_);
    ground_.reset();
    obstacles_.reset();

    try {
      const bool ready = tf_buffer_.canTransform(
          world_frame_, obstacles_frame_id_, obstacles_stamp_,
          tf2::durationFromSec(tf_timeout_ms_ > 0 ? static_cast<double>(tf_timeout_ms_) / 1000.0
                                                  : 0.0));
      if (!ready) return;
      const auto transform = tf_buffer_.lookupTransform(
          world_frame_, obstacles_frame_id_, obstacles_stamp_, tf2::Duration(0));
      const Eigen::Isometry3d sensor_to_map = tf2::transformToEigen(transform.transform);

      {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        grid_->update(*ground, *obstacles, sensor_to_map);
      }
    } catch (const std::exception& exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "occupancy update skipped: %s", exception.what());
    }
  }

  void publishMap(bool force) {
    {
      std::lock_guard<std::mutex> lock(grid_mutex_);
      const uint64_t version = grid_->version();
      if (!force && published_ && version == published_version_) return;
      grid_->takeSnapshot(&snapshot_);
      published_version_ = version;
    }
    map_pub_->publish(OccupancyGrid2D::toMessage(snapshot_, world_frame_, now()));
    published_ = true;
  }

  GridParams params_;
  std::unique_ptr<OccupancyGrid2D> grid_;
  std::string world_frame_ = "map";
  std::string ground_topic_ = "/ground_segmentation/ground";
  std::string obstacles_topic_ = "/ground_segmentation/obstacles";
  std::string map_topic_ = "/occupancy_map_2d/map";
  double stamp_tolerance_ = 0.05;
  int publish_period_ms_ = 1000;
  int tf_timeout_ms_ = 100;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::CallbackGroup::SharedPtr io_group_;
  rclcpp::CallbackGroup::SharedPtr output_group_;
  std::mutex input_mutex_;
  std::shared_ptr<PointCloudT> ground_;
  std::shared_ptr<PointCloudT> obstacles_;
  rclcpp::Time ground_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time obstacles_stamp_{0, 0, RCL_ROS_TIME};
  std::string obstacles_frame_id_;
  std::mutex grid_mutex_;

  GridSnapshot snapshot_;
  uint64_t published_version_ = 0;
  bool published_ = false;
};

}
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<occupancy_map_2d::OccupancyMap2DNode>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
