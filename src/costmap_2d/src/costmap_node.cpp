#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
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

#include "costmap_2d/costmap.hpp"

namespace costmap_2d {
namespace {

class CostmapNode : public rclcpp::Node {
 public:
  CostmapNode() : Node("costmap_2d"), tf_buffer_(get_clock()) {
    loadParameters();
    costmap_ = std::make_unique<RollingCostmap>(params_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    output_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = io_group_;
    ground_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        ground_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { groundCallback(msg); },
        sub_options);
    nonground_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        nonground_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          nongroundCallback(msg);
        },
        sub_options);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).transient_local().reliable());
    clear_service_ = create_service<std_srvs::srv::Trigger>(
        "~/clear",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            costmap_->reset(robot_x_, robot_y_);
          }
          publishMap(true);
          response->success = true;
          response->message = "costmap cleared";
        },
        rclcpp::ServicesQoS(), output_group_);
    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_), [this]() { publishMap(false); },
        output_group_);
    decay_timer_ = create_wall_timer(
        std::chrono::milliseconds(decay_period_ms_), [this]() { applyDecay(); },
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
    nonground_topic_ = param<std::string>("nonground_topic", nonground_topic_);
    map_topic_ = param<std::string>("map_topic", map_topic_);
    stamp_tolerance_ = param<double>("runtime.stamp_tolerance", stamp_tolerance_);
    publish_period_ms_ = param<int>("runtime.publish_period_ms", publish_period_ms_);
    decay_period_ms_ = param<int>("runtime.decay_period_ms", decay_period_ms_);
    tf_timeout_ms_ = param<int>("runtime.tf_timeout_ms", tf_timeout_ms_);
    params_.resolution = param<double>("window.resolution", params_.resolution);
    params_.width = param<double>("window.width", params_.width);
    params_.height = param<double>("window.height", params_.height);
    params_.recenter_distance =
        param<double>("window.recenter_distance", params_.recenter_distance);
    params_.max_range = param<double>("window.max_range", params_.max_range);
    params_.clear_min_z = param<double>("z.clear_min", params_.clear_min_z);
    params_.clear_max_z = param<double>("z.clear_max", params_.clear_max_z);
    params_.obstacle_min_z = param<double>("z.obstacle_min", params_.obstacle_min_z);
    params_.obstacle_max_z = param<double>("z.obstacle_max", params_.obstacle_max_z);
    params_.occupied_log_odds = param<double>("fuse.occupied_log_odds", params_.occupied_log_odds);
    params_.free_log_odds = param<double>("fuse.free_log_odds", params_.free_log_odds);
    params_.min_log_odds = param<double>("fuse.min_log_odds", params_.min_log_odds);
    params_.max_log_odds = param<double>("fuse.max_log_odds", params_.max_log_odds);
    params_.ray_block_probability =
        param<double>("fuse.ray_block_probability", params_.ray_block_probability);
    params_.free_ray_occupied_delta =
        param<double>("fuse.free_ray_occupied_delta", params_.free_ray_occupied_delta);
    params_.ground_relative = param<bool>("z.ground_relative", params_.ground_relative);
    params_.ground_radius = param<double>("z.ground_radius", params_.ground_radius);
    params_.ground_percentile = param<double>("z.ground_percentile", params_.ground_percentile);
    params_.ground_smoothing = param<double>("z.ground_smoothing", params_.ground_smoothing);
    params_.ground_max_step = param<double>("z.ground_max_step", params_.ground_max_step);
    params_.blind_hold_sec = param<double>("fuse.blind_hold_sec", params_.blind_hold_sec);
    params_.occupied_decay = param<double>("fuse.occupied_decay", params_.occupied_decay);
    params_.free_decay = param<double>("fuse.free_decay", params_.free_decay);
    params_.min_component_cells =
        param<int>("fuse.min_component_cells", params_.min_component_cells);
    params_.raycast_dedupe_deg =
        param<double>("fuse.raycast_dedupe_deg", params_.raycast_dedupe_deg);
    publish_period_ms_ = std::max(50, publish_period_ms_);
    decay_period_ms_ = std::max(50, decay_period_ms_);
    tf_timeout_ms_ = std::max(0, tf_timeout_ms_);
  }

  void applyDecay() {
    double now_sec = 0.0;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      now_sec = now().seconds();
      if (!started_) return;
      costmap_->decay(now_sec);
    }
    publishMap(false);
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

  void nongroundCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    auto cloud = std::make_shared<PointCloudT>();
    try {
      pcl::fromROSMsg(*msg, *cloud);
    } catch (const std::exception& exception) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "nonground cloud conversion failed: %s", exception.what());
      return;
    }
    std::lock_guard<std::mutex> lock(input_mutex_);
    nonground_ = std::move(cloud);
    nonground_stamp_ = rclcpp::Time(msg->header.stamp);
    nonground_frame_id_ = msg->header.frame_id;
    matchAndUpdate();
  }

  void matchAndUpdate() {
    if (!ground_ || !nonground_) return;

    const double delta = (nonground_stamp_ - ground_stamp_).seconds();
    if (std::abs(delta) > stamp_tolerance_) {
      if (delta > 0.0) {
        ground_.reset();
      } else {
        nonground_.reset();
      }
      return;
    }

    const std::shared_ptr<PointCloudT> ground = std::move(ground_);
    const std::shared_ptr<PointCloudT> nonground = std::move(nonground_);
    ground_.reset();
    nonground_.reset();

    try {
      const bool ready = tf_buffer_.canTransform(
          world_frame_, nonground_frame_id_, nonground_stamp_,
          tf2::durationFromSec(tf_timeout_ms_ > 0 ? static_cast<double>(tf_timeout_ms_) / 1000.0
                                                  : 0.0));
      if (!ready) return;
      const auto transform = tf_buffer_.lookupTransform(
          world_frame_, nonground_frame_id_, nonground_stamp_, tf2::Duration(0));
      const Eigen::Isometry3d sensor_to_map = tf2::transformToEigen(transform.transform);

      const double start = static_cast<double>(now().seconds());
      {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        robot_x_ = sensor_to_map.translation().x();
        robot_y_ = sensor_to_map.translation().y();
        if (costmap_->needsRecenter(robot_x_, robot_y_)) {
          costmap_->recenter(robot_x_, robot_y_);
        }
        costmap_->update(*ground, *nonground, sensor_to_map, start);
        started_ = true;
        if (params_.ground_relative) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                               "costmap occupied=%zu ground_z=%+.2f m relative_z=%s",
                               costmap_->occupiedCells(), costmap_->groundZ(),
                               costmap_->hasGround() ? "on" : "off");
        }
      }
    } catch (const std::exception& exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "costmap update skipped: %s",
                           exception.what());
    }
  }

  void publishMap(bool force) {
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!started_) return;
      const uint64_t version = costmap_->version();
      if (!force && published_ && version == published_version_) return;
      costmap_->takeSnapshot(&snapshot_);
      published_version_ = version;
    }
    map_pub_->publish(RollingCostmap::toMessage(snapshot_, world_frame_, now()));
    published_ = true;
  }

  CostmapParams params_;
  std::unique_ptr<RollingCostmap> costmap_;
  std::string world_frame_ = "map";
  std::string ground_topic_ = "/ground_segmentation/ground";
  std::string nonground_topic_ = "/ground_segmentation/nonground";
  std::string map_topic_ = "/costmap_2d/grid";
  double stamp_tolerance_ = 0.05;
  int publish_period_ms_ = 200;
  int decay_period_ms_ = 200;
  int tf_timeout_ms_ = 100;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr decay_timer_;
  rclcpp::CallbackGroup::SharedPtr io_group_;
  rclcpp::CallbackGroup::SharedPtr output_group_;
  std::mutex input_mutex_;
  std::shared_ptr<PointCloudT> ground_;
  std::shared_ptr<PointCloudT> nonground_;
  rclcpp::Time ground_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time nonground_stamp_{0, 0, RCL_ROS_TIME};
  std::string nonground_frame_id_;
  std::mutex costmap_mutex_;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  bool started_ = false;

  CostmapSnapshot snapshot_;
  uint64_t published_version_ = 0;
  bool published_ = false;
};

}
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<costmap_2d::CostmapNode>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
