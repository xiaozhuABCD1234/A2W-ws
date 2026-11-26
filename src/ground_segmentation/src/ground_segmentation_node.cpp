#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "ground_segmentation/segmenter.hpp"

namespace ground_segmentation {
namespace {

class GroundSegmentationNode : public rclcpp::Node {
 public:
  GroundSegmentationNode() : Node("ground_segmentation") {
    loadParameters();
    segmenter_ = std::make_unique<PolarGroundSegmenter>(params_);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(50),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odomCallback(msg); });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { cloudCallback(msg); });
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ground_topic_, rclcpp::SensorDataQoS());
    obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        obstacles_topic_, rclcpp::SensorDataQoS());
    elevated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        elevated_topic_, rclcpp::SensorDataQoS());
    RCLCPP_INFO(get_logger(), "ground segmentation started with %d sectors and %.2f m rings",
                params_.sectors, params_.ring_size);
  }

 private:
  struct Frame {
    double stamp = 0.0;
    std_msgs::msg::Header header;
    Eigen::Isometry3d raw_pose = Eigen::Isometry3d::Identity();
    PointCloudPtr cloud;
  };

  template <typename T>
  T param(const std::string& name, const T& default_value) {
    return declare_parameter<T>(name, default_value);
  }

  void loadParameters() {
    odom_topic_ = param<std::string>("odom_topic", odom_topic_);
    cloud_topic_ = param<std::string>("cloud_topic", cloud_topic_);
    ground_topic_ = param<std::string>("ground_topic", ground_topic_);
    obstacles_topic_ = param<std::string>("obstacles_topic", obstacles_topic_);
    elevated_topic_ = param<std::string>("elevated_topic", elevated_topic_);
    stamp_tolerance_ = param<double>("runtime.stamp_tolerance", stamp_tolerance_);
    gravity_align_ = param<bool>("runtime.gravity_align", gravity_align_);
    log_period_ms_ = param<int>("runtime.log_period_ms", log_period_ms_);
    params_.min_range = param<double>("segmentation.min_range", params_.min_range);
    params_.max_range = param<double>("segmentation.max_range", params_.max_range);
    params_.ring_size = param<double>("segmentation.ring_size", params_.ring_size);
    params_.sectors = param<int>("segmentation.sectors", params_.sectors);
    params_.min_cell_points =
        param<int>("segmentation.min_cell_points", params_.min_cell_points);
    params_.seed_height_band =
        param<double>("segmentation.seed_height_band", params_.seed_height_band);
    params_.ground_height_threshold =
        param<double>("segmentation.ground_height_threshold", params_.ground_height_threshold);
    params_.max_ground_step =
        param<double>("segmentation.max_ground_step", params_.max_ground_step);
    params_.max_slope_deg =
        param<double>("segmentation.max_slope_deg", params_.max_slope_deg);
    params_.ground_alpha = param<double>("segmentation.ground_alpha", params_.ground_alpha);
    params_.initial_ground_tolerance =
        param<double>("segmentation.initial_ground_tolerance", params_.initial_ground_tolerance);
    params_.elevated_height =
        param<double>("segmentation.elevated_height", params_.elevated_height);
    params_.ground_estimation_range =
        param<double>("segmentation.ground_estimation_range", params_.ground_estimation_range);
  }

  Eigen::Isometry3d poseFromOdom(const nav_msgs::msg::Odometry& msg) const {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    const auto& p = msg.pose.pose.position;
    const auto& q = msg.pose.pose.orientation;
    pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    pose.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    return pose;
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    const auto pose = poseFromOdom(*msg);
    std::vector<Frame> ready;
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      odom_buffer_.emplace_back(stamp, pose);
      while (odom_buffer_.size() > 200) odom_buffer_.pop_front();
      for (auto it = waiting_clouds_.begin(); it != waiting_clouds_.end();) {
        if (std::abs(it->stamp - stamp) <= stamp_tolerance_) {
          it->raw_pose = pose;
          ready.push_back(std::move(*it));
          it = waiting_clouds_.erase(it);
        } else {
          ++it;
        }
      }
    }
    for (const auto& frame : ready) process(frame);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    Frame frame;
    frame.stamp = rclcpp::Time(msg->header.stamp).seconds();
    frame.header = msg->header;
    frame.cloud = PointCloudPtr(new PointCloudT());
    pcl::fromROSMsg(*msg, *frame.cloud);
    bool matched = false;
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      while (!waiting_clouds_.empty() &&
             frame.stamp - waiting_clouds_.front().stamp > std::max(0.5, 10.0 * stamp_tolerance_)) {
        waiting_clouds_.pop_front();
      }
      auto best = odom_buffer_.end();
      double best_dt = stamp_tolerance_;
      for (auto it = odom_buffer_.begin(); it != odom_buffer_.end(); ++it) {
        const double dt = std::abs(it->first - frame.stamp);
        if (dt <= best_dt) {
          best = it;
          best_dt = dt;
        }
      }
      if (best != odom_buffer_.end()) {
        frame.raw_pose = best->second;
        odom_buffer_.erase(best);
        matched = true;
      } else {
        waiting_clouds_.push_back(std::move(frame));
        while (waiting_clouds_.size() > 8) waiting_clouds_.pop_front();
      }
    }
    if (matched) process(frame);
  }

  void process(const Frame& frame) {
    try {
      Eigen::Matrix3d gravity_rotation = Eigen::Matrix3d::Identity();
      if (gravity_align_) {
        const auto& rotation = frame.raw_pose.linear();
        const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
        gravity_rotation = rotation *
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix().transpose();
      }
      const auto start = std::chrono::steady_clock::now();
      const auto result = segmenter_->segment(*frame.cloud, gravity_rotation);
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
      publish(ground_pub_, *result.ground, frame.header);
      publish(obstacles_pub_, *result.obstacles, frame.header);
      publish(elevated_pub_, *result.elevated, frame.header);
      if (log_period_ms_ > 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_period_ms_,
                             "ground=%zu obstacles=%zu elevated=%zu total=%zu time=%.2f ms",
                             result.ground->size(), result.obstacles->size(),
                             result.elevated->size(), result.finite_points, elapsed_ms);
      }
    } catch (const std::exception& exception) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "ground segmentation failed: %s", exception.what());
    }
  }

  void publish(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
               const PointCloudT& cloud, const std_msgs::msg::Header& header) {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header = header;
    publisher->publish(message);
  }

  SegmentationParams params_;
  std::unique_ptr<PolarGroundSegmenter> segmenter_;
  std::string odom_topic_ = "/aft_mapped_to_init";
  std::string cloud_topic_ = "/cloud_registered_body";
  std::string ground_topic_ = "/ground_segmentation/ground";
  std::string obstacles_topic_ = "/ground_segmentation/obstacles";
  std::string elevated_topic_ = "/ground_segmentation/elevated";
  double stamp_tolerance_ = 0.05;
  bool gravity_align_ = true;
  int log_period_ms_ = 5000;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr elevated_pub_;
  std::mutex buffer_mutex_;
  std::deque<std::pair<double, Eigen::Isometry3d>> odom_buffer_;
  std::deque<Frame> waiting_clouds_;
};

}
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ground_segmentation::GroundSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
