#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "relocalization/relocator.hpp"

namespace relocalization {
namespace {

struct Frame {
  double stamp = 0.0;
  int64_t stamp_ns = 0;
  Eigen::Isometry3d raw_pose = Eigen::Isometry3d::Identity();
  PointCloudPtr cloud;
};

class RelocalizationNode : public rclcpp::Node {
 public:
  RelocalizationNode() : Node("relocalization") {
    loadParameters();
    relocator_ = std::make_unique<PriorMapRelocator>(params_);
    std::string error;
    if (!relocator_->loadMap(params_.map_path, &error)) {
      RCLCPP_ERROR(get_logger(), "prior map load failed: %s", error.c_str());
      relocator_.reset();
      active_ = false;
    } else {
      RCLCPP_INFO(get_logger(), "loaded prior map %s with %zu points",
                  params_.map_path.c_str(), relocator_->mapSize());
    }
    createInterfaces();
    if (relocator_) worker_ = std::thread([this]() { workerLoop(); });
  }

  ~RelocalizationNode() override {
    stop_ = true;
    queue_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  template <typename T>
  T param(const std::string& name, const T& default_value) {
    return declare_parameter<T>(name, default_value);
  }

  void loadParameters() {
    world_frame_ = param<std::string>("world_frame", "map");
    odom_frame_ = param<std::string>("odom_frame", "camera_init");
    odom_topic_ = param<std::string>("odom_topic", "/aft_mapped_to_init");
    cloud_topic_ = param<std::string>("cloud_topic", "/cloud_registered_body");
    correction_topic_ = param<std::string>("correction_topic", "/relocalization/correction");
    params_.map_path = param<std::string>("map.path", params_.map_path);
    params_.auto_start = param<bool>("map.auto_start", params_.auto_start);
    params_.confirmation_frames =
        param<int>("confirmation.frames", params_.confirmation_frames);
    params_.confirmation_trans_tolerance =
        param<double>("confirmation.translation_tolerance", params_.confirmation_trans_tolerance);
    params_.confirmation_rot_tolerance_deg =
        param<double>("confirmation.rotation_tolerance_deg", params_.confirmation_rot_tolerance_deg);
    params_.max_attempts = param<int>("runtime.max_attempts", params_.max_attempts);
    params_.submap_frames = param<int>("runtime.submap_frames", params_.submap_frames);
    params_.submap_voxel = param<double>("runtime.submap_voxel", params_.submap_voxel);
    params_.min_frame_interval =
        param<double>("runtime.min_frame_interval", params_.min_frame_interval);
    params_.stamp_tolerance = param<double>("runtime.stamp_tolerance", params_.stamp_tolerance);
    params_.gravity_align = param<bool>("runtime.gravity_align", params_.gravity_align);
    params_.pyramid_voxels = param<std::vector<double>>("search.pyramid_voxels", params_.pyramid_voxels);
    params_.coarse_xy_step = param<double>("search.coarse_xy_step", params_.coarse_xy_step);
    params_.coarse_z_step = param<double>("search.coarse_z_step", params_.coarse_z_step);
    params_.coarse_yaw_step_deg =
        param<double>("search.coarse_yaw_step_deg", params_.coarse_yaw_step_deg);
    params_.search_z_min = param<double>("search.z_min", params_.search_z_min);
    params_.search_z_max = param<double>("search.z_max", params_.search_z_max);
    params_.max_candidates = param<int>("search.max_candidates", params_.max_candidates);
    params_.max_query_points = param<int>("search.max_query_points", params_.max_query_points);
    params_.filter_ground = param<bool>("gicp.filter_ground", params_.filter_ground);
    params_.ground_distance_threshold =
        param<double>("gicp.ground_distance_threshold", params_.ground_distance_threshold);
    params_.gicp_voxel = param<double>("gicp.voxel", params_.gicp_voxel);
    params_.gicp_max_points = param<int>("gicp.max_points", params_.gicp_max_points);
    params_.gicp_iterations = param<int>("gicp.max_iterations", params_.gicp_iterations);
    params_.gicp_max_correspondence =
        param<double>("gicp.max_correspondence", params_.gicp_max_correspondence);
    params_.validation_correspondence =
        param<double>("gicp.validation_correspondence", params_.validation_correspondence);
    params_.max_fitness = param<double>("gicp.max_fitness", params_.max_fitness);
    params_.min_inlier_ratio = param<double>("gicp.min_inlier_ratio", params_.min_inlier_ratio);
    params_.max_roll_pitch_deg =
        param<double>("validation.max_roll_pitch_deg", params_.max_roll_pitch_deg);
    params_.max_refinement_trans =
        param<double>("validation.max_refinement_translation", params_.max_refinement_trans);
    params_.max_refinement_rot_deg =
        param<double>("validation.max_refinement_rotation_deg", params_.max_refinement_rot_deg);
    params_.min_ambiguity_margin =
        param<double>("validation.min_ambiguity_margin", params_.min_ambiguity_margin);
    params_.ambiguity_translation =
        param<double>("validation.ambiguity_translation", params_.ambiguity_translation);
    params_.ambiguity_rotation_deg =
        param<double>("validation.ambiguity_rotation_deg", params_.ambiguity_rotation_deg);
    params_.confirmation_frames = std::max(1, params_.confirmation_frames);
    params_.submap_frames = std::max(1, params_.submap_frames);
    params_.max_candidates = std::max(1, params_.max_candidates);
    active_ = params_.auto_start;
  }

  void createInterfaces() {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(50),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odomCallback(msg); });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { cloudCallback(msg); });
    correction_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(correction_topic_, 10);
    trigger_service_ = create_service<std_srvs::srv::Trigger>(
        "~/trigger",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          if (!relocator_) {
            response->success = false;
            response->message = "prior map is not loaded";
            return;
          }
          trigger_ = true;
          response->success = true;
          response->message = "relocalization scheduled";
        });
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
        if (std::abs(it->stamp - stamp) <= params_.stamp_tolerance) {
          it->raw_pose = pose;
          ready.push_back(std::move(*it));
          it = waiting_clouds_.erase(it);
        } else {
          ++it;
        }
      }
    }
    for (auto& frame : ready) enqueue(std::move(frame));
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    PointCloudPtr cloud(new PointCloudT());
    pcl::fromROSMsg(*msg, *cloud);
    Frame frame;
    frame.stamp = rclcpp::Time(msg->header.stamp).seconds();
    frame.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    frame.cloud = cloud;
    bool matched = false;
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      auto best = odom_buffer_.end();
      double best_dt = params_.stamp_tolerance;
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
    if (matched) enqueue(std::move(frame));
  }

  void enqueue(Frame&& frame) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (queue_.size() >= 2) queue_.pop_front();
      queue_.push_back(std::move(frame));
    }
    queue_cv_.notify_one();
  }

  void workerLoop() {
    while (!stop_) {
      Frame frame;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() { return stop_ || !queue_.empty(); });
        if (stop_) return;
        frame = std::move(queue_.front());
        queue_.pop_front();
      }
      process(std::move(frame));
    }
  }

  PointCloudPtr buildSubmap(const Frame& latest) {
    PointCloudPtr merged(new PointCloudT());
    for (const auto& frame : frames_) {
      const Eigen::Isometry3d relative = latest.raw_pose.inverse() * frame.raw_pose;
      for (const auto& point : frame.cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
        const Eigen::Vector3d p = relative * Eigen::Vector3d(point.x, point.y, point.z);
        PointT transformed = point;
        transformed.x = static_cast<float>(p.x());
        transformed.y = static_cast<float>(p.y());
        transformed.z = static_cast<float>(p.z());
        merged->push_back(transformed);
      }
    }
    if (params_.submap_voxel > 0.0 && !merged->empty()) {
      PointCloudPtr filtered(new PointCloudT());
      pcl::VoxelGrid<PointT> voxel;
      const float leaf = static_cast<float>(params_.submap_voxel);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.setInputCloud(merged);
      voxel.filter(*filtered);
      merged.swap(filtered);
    }
    return merged;
  }

  void process(Frame&& frame) {
    frames_.push_back(std::move(frame));
    while (static_cast<int>(frames_.size()) > params_.submap_frames) frames_.pop_front();
    const Frame& latest = frames_.back();
    if (trigger_.exchange(false)) {
      active_ = true;
      attempts_ = 0;
      confirmations_.clear();
    }
    if (!active_) return;
    if (last_attempt_stamp_ >= 0.0 && latest.stamp >= last_attempt_stamp_ &&
        latest.stamp - last_attempt_stamp_ < params_.min_frame_interval) {
      return;
    }
    if (latest.stamp < last_attempt_stamp_) confirmations_.clear();
    last_attempt_stamp_ = latest.stamp;
    if (attempts_ >= params_.max_attempts) {
      active_ = false;
      confirmations_.clear();
      RCLCPP_WARN(get_logger(), "relocalization stopped after %d attempts", attempts_);
      return;
    }
    ++attempts_;
    const auto start = std::chrono::steady_clock::now();
    Eigen::Matrix3d level_rotation = Eigen::Matrix3d::Identity();
    if (params_.gravity_align) {
      const auto& rotation = latest.raw_pose.linear();
      const double raw_yaw = std::atan2(rotation(1, 0), rotation(0, 0));
      const Eigen::Matrix3d yaw_rotation =
          Eigen::AngleAxisd(raw_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      level_rotation = rotation * yaw_rotation.transpose();
    }
    PointCloudPtr body_submap = buildSubmap(latest);
    PointCloudPtr level_submap(new PointCloudT());
    level_submap->reserve(body_submap->size());
    for (const auto& point : body_submap->points) {
      const Eigen::Vector3d p =
          level_rotation * Eigen::Vector3d(point.x, point.y, point.z);
      PointT transformed = point;
      transformed.x = static_cast<float>(p.x());
      transformed.y = static_cast<float>(p.y());
      transformed.z = static_cast<float>(p.z());
      level_submap->push_back(transformed);
    }
    std::string rejection;
    const auto result = relocator_->localize(*level_submap, &rejection);
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    if (!result) {
      RCLCPP_WARN(get_logger(), "relocalization attempt %d rejected after %.2f s: %s",
                  attempts_, elapsed, rejection.c_str());
      return;
    }
    Eigen::Isometry3d map_body = result->T_map_body;
    map_body.linear() = result->T_map_body.linear() * level_rotation;
    const Eigen::Isometry3d correction = map_body * latest.raw_pose.inverse();
    if (!confirmations_.empty()) {
      const auto& previous = confirmations_.back();
      const double translation = (correction.translation() - previous.translation()).norm();
      const double rotation = Eigen::AngleAxisd(
          correction.linear() * previous.linear().transpose()).angle() * 180.0 / M_PI;
      if (translation > params_.confirmation_trans_tolerance ||
          rotation > params_.confirmation_rot_tolerance_deg) {
        confirmations_.clear();
      }
    }
    confirmations_.push_back(correction);
    while (confirmations_.size() > static_cast<size_t>(params_.confirmation_frames)) {
      confirmations_.erase(confirmations_.begin());
    }
    RCLCPP_INFO(get_logger(),
                "relocalization attempt %d accepted: fitness=%.4f inliers=%.3f confirmations=%zu/%d",
                attempts_, result->fitness, result->inlier_ratio, confirmations_.size(),
                params_.confirmation_frames);
    if (static_cast<int>(confirmations_.size()) < params_.confirmation_frames) return;

    size_t medoid = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < confirmations_.size(); ++i) {
      double cost = 0.0;
      for (size_t j = 0; j < confirmations_.size(); ++j) {
        cost += (confirmations_[i].translation() - confirmations_[j].translation()).norm();
        cost += Eigen::AngleAxisd(
            confirmations_[i].linear() * confirmations_[j].linear().transpose()).angle();
      }
      if (cost < best_cost) {
        best_cost = cost;
        medoid = i;
      }
    }
    const Eigen::Vector3d translation = confirmations_[medoid].translation();
    const double yaw = std::atan2(confirmations_[medoid].linear()(1, 0),
                                  confirmations_[medoid].linear()(0, 0));
    geometry_msgs::msg::TransformStamped output =
        tf2::eigenToTransform(confirmations_[medoid]);
    output.header.stamp = rclcpp::Time(latest.stamp_ns);
    output.header.frame_id = world_frame_;
    output.child_frame_id = odom_frame_;
    correction_pub_->publish(output);
    active_ = false;
    confirmations_.clear();
    RCLCPP_INFO(get_logger(),
                "relocalization confirmed in %.2f s: xyz=[%.3f %.3f %.3f] yaw=%.3f",
                elapsed, translation.x(), translation.y(), translation.z(), yaw);
  }

  RelocalizationParams params_;
  std::string world_frame_;
  std::string odom_frame_;
  std::string odom_topic_;
  std::string cloud_topic_;
  std::string correction_topic_;
  std::unique_ptr<PriorMapRelocator> relocator_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr correction_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
  std::mutex buffer_mutex_;
  std::deque<std::pair<double, Eigen::Isometry3d>> odom_buffer_;
  std::deque<Frame> waiting_clouds_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<Frame> queue_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> trigger_{false};
  bool active_ = true;
  int attempts_ = 0;
  double last_attempt_stamp_ = -1.0;
  std::deque<Frame> frames_;
  std::vector<Eigen::Isometry3d> confirmations_;
};

}
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<relocalization::RelocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
