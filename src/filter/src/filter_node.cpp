#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "filter/detection_gate.hpp"
#include "filter/pedestrian_filter.hpp"
#include "filter/self_filter.hpp"

namespace filter {
namespace {

struct PoseSample {
  double stamp = 0.0;
  Eigen::Vector3f translation = Eigen::Vector3f::Zero();
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
};

class DynamicFilterNode : public rclcpp::Node {
 public:
  DynamicFilterNode() : Node("filter") {
    loadParameters();
    voxel_params_.voxel_size = param<double>("filter.voxel_size", voxel_params_.voxel_size);
    voxel_params_.min_range = param<double>("filter.min_range", voxel_params_.min_range);
    voxel_params_.max_range = param<double>("filter.max_range", voxel_params_.max_range);
    voxel_params_.min_height = param<double>("filter.min_height", voxel_params_.min_height);
    voxel_params_.max_height = param<double>("filter.max_height", voxel_params_.max_height);
    voxel_params_.azimuth_step_deg =
        param<double>("filter.azimuth_step_deg", voxel_params_.azimuth_step_deg);
    voxel_params_.elevation_step_deg =
        param<double>("filter.elevation_step_deg", voxel_params_.elevation_step_deg);
    voxel_params_.max_rays_per_frame =
        param<int>("filter.max_rays_per_frame", voxel_params_.max_rays_per_frame);
    voxel_params_.free_frames_threshold =
        param<int>("filter.free_frames_threshold", voxel_params_.free_frames_threshold);
    voxel_params_.free_label_score =
        param<int>("filter.free_label_score", voxel_params_.free_label_score);
    voxel_params_.occupied_score = param<int>("filter.occupied_score", voxel_params_.occupied_score);
    voxel_params_.free_score = param<int>("filter.free_score", voxel_params_.free_score);
    voxel_params_.min_score = param<int>("filter.min_score", voxel_params_.min_score);
    voxel_params_.max_score = param<int>("filter.max_score", voxel_params_.max_score);
    voxel_params_.occupied_recent_gain =
        param<int>("filter.occupied_recent_gain", voxel_params_.occupied_recent_gain);
    voxel_params_.cluster_radius = param<double>("filter.cluster_radius", voxel_params_.cluster_radius);
    voxel_params_.relearn_frames = param<int>("filter.relearn_frames", voxel_params_.relearn_frames);
    voxel_params_.decay_period_frames =
        param<int>("filter.decay_period_frames", voxel_params_.decay_period_frames);
    voxel_params_.max_voxels = param<double>("filter.max_voxels", voxel_params_.max_voxels);
    pedestrian_params_.cell_size =
        param<double>("pedestrian.cell_size", pedestrian_params_.cell_size);
    pedestrian_params_.min_points =
        param<int>("pedestrian.min_points", pedestrian_params_.min_points);
    pedestrian_params_.max_width = param<double>("pedestrian.max_width", pedestrian_params_.max_width);
    pedestrian_params_.max_depth = param<double>("pedestrian.max_depth", pedestrian_params_.max_depth);
    pedestrian_params_.min_extent =
        param<double>("pedestrian.min_extent", pedestrian_params_.min_extent);
    pedestrian_params_.max_extent =
        param<double>("pedestrian.max_extent", pedestrian_params_.max_extent);
    pedestrian_params_.max_footprint =
        param<double>("pedestrian.max_footprint", pedestrian_params_.max_footprint);
    pedestrian_params_.min_aspect =
        param<double>("pedestrian.min_aspect", pedestrian_params_.min_aspect);
    pedestrian_params_.keep_unmatched =
        param<bool>("pedestrian.keep_unmatched", pedestrian_params_.keep_unmatched);
    self_params_.enabled = param<bool>("self.enabled", self_params_.enabled);
    self_params_.min_x = param<double>("self.min_x", self_params_.min_x);
    self_params_.max_x = param<double>("self.max_x", self_params_.max_x);
    self_params_.min_y = param<double>("self.min_y", self_params_.min_y);
    self_params_.max_y = param<double>("self.max_y", self_params_.max_y);
    self_params_.min_z = param<double>("self.min_z", self_params_.min_z);
    self_params_.max_z = param<double>("self.max_z", self_params_.max_z);
    self_params_.margin = param<double>("self.margin", self_params_.margin);
    self_params_.min_range = param<double>("self.min_range", self_params_.min_range);
    detection_params_.enabled = param<bool>("detection.enabled", detection_params_.enabled);
    detection_params_.expand = param<double>("detection.expand", detection_params_.expand);
    detection_params_.timeout = param<double>("detection.timeout", detection_params_.timeout);
    detection_params_.min_height =
        param<double>("detection.min_height", detection_params_.min_height);
    detection_params_.max_height =
        param<double>("detection.max_height", detection_params_.max_height);
    detection_params_.min_size = param<double>("detection.min_size", detection_params_.min_size);

    filter_ = std::make_unique<PedestrianFilter>(voxel_params_, pedestrian_params_);
    self_filter_ = std::make_unique<SelfFilter>(self_params_);
    detection_gate_ = std::make_unique<DetectionGate>(detection_params_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odomCallback(msg); });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { cloudCallback(msg); });
    detection_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        detection_topic_, rclcpp::QoS(10),
        [this](const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
          detectionCallback(msg);
        });
    static_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        static_topic_, rclcpp::SensorDataQoS());
    pedestrian_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        pedestrian_topic_, rclcpp::SensorDataQoS());
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "~/reset",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            filter_->reset();
            self_filter_->reset();
            detection_gate_->reset();
            poses_.clear();
          }
          response->success = true;
          response->message = "dynamic filter reset";
        });
    RCLCPP_INFO(get_logger(), "filter started: %s -> %s (self box %s, pedestrian gate on)",
                cloud_topic_.c_str(), static_topic_.c_str(),
                self_params_.enabled ? "on" : "off");
  }

 private:
  template <typename T>
  T param(const std::string& name, const T& default_value) {
    return declare_parameter<T>(name, default_value);
  }

  void loadParameters() {
    odom_topic_ = param<std::string>("runtime.odom_topic", odom_topic_);
    cloud_topic_ = param<std::string>("runtime.cloud_topic", cloud_topic_);
    cloud_frame_ = param<std::string>("runtime.cloud_frame", cloud_frame_);
    pose_frame_ = param<std::string>("runtime.pose_frame", pose_frame_);
    static_topic_ = param<std::string>("runtime.static_topic", static_topic_);
    pedestrian_topic_ = param<std::string>("runtime.pedestrian_topic", pedestrian_topic_);
    detection_topic_ = param<std::string>("detection.topic", detection_topic_);
    stamp_tolerance_ = param<double>("runtime.stamp_tolerance", stamp_tolerance_);
    log_period_frames_ = std::max(1, param<int>("runtime.log_period_frames", log_period_frames_));
    max_pose_buffer_ = std::max(1, param<int>("runtime.max_pose_buffer", max_pose_buffer_));
  }

  static double seconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    const auto& pose = msg->pose.pose;
    PoseSample sample;
    sample.stamp = seconds(msg->header.stamp);
    sample.translation = Eigen::Vector3f(static_cast<float>(pose.position.x),
                                         static_cast<float>(pose.position.y),
                                         static_cast<float>(pose.position.z));
    const Eigen::Quaternionf rotation(static_cast<float>(pose.orientation.w),
                                      static_cast<float>(pose.orientation.x),
                                      static_cast<float>(pose.orientation.y),
                                      static_cast<float>(pose.orientation.z));
    if (!sample.translation.allFinite() || !rotation.coeffs().allFinite() ||
        rotation.norm() < 1e-6f) {
      return;
    }
    sample.rotation = rotation.normalized().toRotationMatrix();
    std::lock_guard<std::mutex> lock(filter_mutex_);
    if (!poses_.empty() && sample.stamp <= poses_.back().stamp) {
      poses_.clear();
    }
    poses_.push_back(sample);
    while (static_cast<int>(poses_.size()) > max_pose_buffer_) poses_.pop_front();
  }

  void detectionCallback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
    if (msg->markers.empty()) {
      std::lock_guard<std::mutex> lock(filter_mutex_);
      detection_gate_->reset();
      return;
    }
    std::vector<DetectionBox> boxes;
    boxes.reserve(msg->markers.size());
    const std::string frame = msg->markers.front().header.frame_id;
    const double stamp = seconds(msg->markers.front().header.stamp);
    bool convertible = false;
    PoseSample pose;
    {
      std::lock_guard<std::mutex> lock(filter_mutex_);
      Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
      if (frame == pose_frame_) {
        convertible = true;
      } else if (frame == cloud_frame_) {
        convertible = lookupPose(stamp, &pose);
        transform.linear() = pose.rotation;
        transform.translation() = pose.translation;
      } else {
        convertible = false;
      }
      if (!convertible) {
        if (++unmapped_detections_ % static_cast<uint64_t>(log_period_frames_) == 1) {
          RCLCPP_WARN(get_logger(), "detection frame '%s' is neither '%s' nor '%s'; ignored",
                      frame.c_str(), pose_frame_.c_str(), cloud_frame_.c_str());
        }
        return;
      }
      for (const auto& marker : msg->markers) {
        if (marker.action == visualization_msgs::msg::Marker::DELETE ||
            marker.action == visualization_msgs::msg::Marker::DELETEALL) {
          continue;
        }
        if (marker.scale.x <= 0.0 || marker.scale.y <= 0.0 || marker.scale.z <= 0.0) continue;
        DetectionBox box;
        const Eigen::Quaternionf rotation(static_cast<float>(marker.pose.orientation.w),
                                          static_cast<float>(marker.pose.orientation.x),
                                          static_cast<float>(marker.pose.orientation.y),
                                          static_cast<float>(marker.pose.orientation.z));
        Eigen::Isometry3f marker_transform = Eigen::Isometry3f::Identity();
        marker_transform.linear() = rotation.normalized().toRotationMatrix();
        marker_transform.translation() = Eigen::Vector3f(
            static_cast<float>(marker.pose.position.x),
            static_cast<float>(marker.pose.position.y),
            static_cast<float>(marker.pose.position.z));
        const Eigen::Isometry3f transformed = transform * marker_transform;
        box.center = transformed.translation();
        box.rotation = Eigen::Quaternionf(transformed.linear()).normalized();
        box.half_size = Eigen::Vector3f(static_cast<float>(marker.scale.x) * 0.5f,
                                        static_cast<float>(marker.scale.y) * 0.5f,
                                        static_cast<float>(marker.scale.z) * 0.5f);
        boxes.push_back(box);
      }
      detection_gate_->set(boxes, stamp);
    }
  }

  bool lookupPose(double stamp, PoseSample* sample) {
    if (poses_.empty()) return false;
    auto best = poses_.begin();
    double best_delta = std::abs(best->stamp - stamp);
    for (auto it = poses_.begin(); it != poses_.end(); ++it) {
      const double delta = std::abs(it->stamp - stamp);
      if (delta < best_delta) {
        best_delta = delta;
        best = it;
      }
    }
    if (best_delta > stamp_tolerance_) return false;
    *sample = *best;
    return true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    const auto start = std::chrono::steady_clock::now();
    const size_t total = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    const size_t step = msg->point_step;
    if (total == 0 || step < sizeof(float) * 3) return;
    if (msg->data.size() < total * step) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "cloud data is truncated (%zu < %zu)", msg->data.size(),
                           total * step);
      return;
    }
    const auto* base = msg->data.data();
    std::vector<Eigen::Vector3f> points(total);
    for (size_t index = 0; index < total; ++index) {
      float xyz[3];
      std::memcpy(xyz, base + index * step, sizeof(xyz));
      points[index] = Eigen::Vector3f(xyz[0], xyz[1], xyz[2]);
    }

    PoseSample pose;
    bool has_pose = false;
    {
      std::lock_guard<std::mutex> lock(filter_mutex_);
      has_pose = lookupPose(seconds(msg->header.stamp), &pose);
    }
    if (!has_pose) {
      ++missing_pose_frames_;
      if (missing_pose_frames_ % static_cast<uint64_t>(log_period_frames_) == 1) {
        RCLCPP_WARN(get_logger(), "no odometry within %.3f s for cloud %.3f; publishing unfiltered",
                    stamp_tolerance_, seconds(msg->header.stamp));
      }
      publish(static_pub_, msg, nullptr, kKeep);
      return;
    }

    std::vector<uint8_t> self_mask;
    const SelfFilterStats self_stats = self_filter_->filter(points, &self_mask);

    std::vector<uint8_t> state(total, kKeep);
    {
      std::lock_guard<std::mutex> lock(filter_mutex_);
      for (size_t index = 0; index < total; ++index) {
        if (self_mask[index] != 0) state[index] = kSelf;
      }
    }

    std::vector<Eigen::Vector3f> world;
    std::vector<int32_t> kept;
    world.reserve(total - self_stats.self_points);
    kept.reserve(total - self_stats.self_points);
    for (size_t index = 0; index < total; ++index) {
      if (state[index] != kKeep) continue;
      world.push_back(pose.rotation * points[index] + pose.translation);
      kept.push_back(static_cast<int32_t>(index));
    }

    std::vector<uint8_t> detection_mask;
    size_t detected = 0;
    {
      std::lock_guard<std::mutex> lock(filter_mutex_);
      detected = detection_gate_->filter(world, seconds(msg->header.stamp), &detection_mask);
    }
    for (size_t slot = 0; slot < kept.size(); ++slot) {
      if (detection_mask[slot] != 0) state[static_cast<size_t>(kept[slot])] = kPedestrian;
    }

    std::vector<Eigen::Vector3f> visible;
    std::vector<int32_t> visible_index;
    visible.reserve(world.size() - detected);
    visible_index.reserve(world.size() - detected);
    for (size_t slot = 0; slot < world.size(); ++slot) {
      if (detection_mask[slot] != 0) continue;
      visible.push_back(world[slot]);
      visible_index.push_back(kept[slot]);
    }

    std::vector<uint8_t> mask;
    PedestrianStats stats;
    if (!visible.empty()) {
      std::lock_guard<std::mutex> lock(filter_mutex_);
      stats = filter_->filter(visible, pose.translation, &mask);
      for (size_t slot = 0; slot < visible_index.size(); ++slot) {
        if (mask[slot] != 0) state[static_cast<size_t>(visible_index[slot])] = kPedestrian;
      }
      stats.pedestrian_points += detected;
    }

    publish(static_pub_, msg, &state, kKeep);
    publish(pedestrian_pub_, msg, &state, kPedestrian);

    const auto elapsed = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - start)
                             .count();
    ++frame_count_;
    total_points_ += total;
    total_pedestrian_ += stats.pedestrian_points;
    total_self_ += self_stats.self_points;
    total_detected_ += detected;
    if (frame_count_ % static_cast<uint64_t>(log_period_frames_) == 0) {
      RCLCPP_INFO(get_logger(),
                  "frames=%zu points=%.0f self=%.0f detected=%.0f pedestrians=%.0f "
                  "clusters=%zu rays=%zu voxels=%zu rate=%.2f%% time=%.2f ms",
                  frame_count_, static_cast<double>(total_points_) / frame_count_,
                  static_cast<double>(total_self_) / frame_count_,
                  static_cast<double>(total_detected_) / frame_count_,
                  static_cast<double>(total_pedestrian_) / frame_count_, stats.clusters,
                  stats.voxel.rays, filter_->mapVoxels(),
                  100.0 * static_cast<double>(total_pedestrian_) /
                      std::max<double>(1.0, static_cast<double>(total_points_)),
                  elapsed);
    }
  }

  static constexpr uint8_t kKeep = 0;
  static constexpr uint8_t kPedestrian = 1;
  static constexpr uint8_t kSelf = 2;

  void publish(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
               const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
               const std::vector<uint8_t>* state, uint8_t wanted) {
    const size_t total = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    const size_t step = msg->point_step;
    size_t count = 0;
    for (size_t index = 0; index < total; ++index) {
      if (state != nullptr && (*state)[index] != wanted) continue;
      ++count;
    }
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = 1;
    out.width = static_cast<uint32_t>(count);
    out.fields = msg->fields;
    out.point_step = msg->point_step;
    out.row_step = static_cast<uint32_t>(count * step);
    out.is_bigendian = msg->is_bigendian;
    out.is_dense = msg->is_dense;
    out.data.resize(count * step);
    if (count == 0 || step == 0) {
      out.row_step = 0;
      publisher->publish(out);
      return;
    }
    const auto* source = msg->data.data();
    auto* target = out.data.data();
    size_t slot = 0;
    for (size_t index = 0; index < total; ++index) {
      if (state != nullptr && (*state)[index] != wanted) continue;
      std::memcpy(target + slot * step, source + index * step, step);
      ++slot;
    }
    publisher->publish(out);
  }

  std::string odom_topic_ = "/aft_mapped_to_init";
  std::string cloud_topic_ = "/cloud_registered_body";
  std::string static_topic_ = "/filter/cloud_static";
  std::string pedestrian_topic_ = "/filter/cloud_pedestrian";
  std::string detection_topic_ = "/filter/pedestrian_boxes";
  std::string pose_frame_ = "camera_init";
  std::string cloud_frame_ = "body";
  double stamp_tolerance_ = 0.05;
  int log_period_frames_ = 100;
  int max_pose_buffer_ = 200;

  FilterParams voxel_params_;
  PedestrianParams pedestrian_params_;
  SelfFilterParams self_params_;
  DetectionGateParams detection_params_;
  std::unique_ptr<PedestrianFilter> filter_;
  std::unique_ptr<SelfFilter> self_filter_;
  std::unique_ptr<DetectionGate> detection_gate_;
  std::deque<PoseSample> poses_;
  std::mutex filter_mutex_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pedestrian_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

  uint64_t frame_count_ = 0;
  uint64_t missing_pose_frames_ = 0;
  uint64_t total_points_ = 0;
  uint64_t total_pedestrian_ = 0;
  uint64_t total_self_ = 0;
  uint64_t total_detected_ = 0;
  uint64_t unmapped_detections_ = 0;
};

}

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<filter::DynamicFilterNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
