#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "point_lio_backend/keyframe_manager.hpp"
#include "point_lio_backend/loop_detector.hpp"
#include "point_lio_backend/pose_graph.hpp"
#include "point_lio_backend/scan_context.hpp"
#include "point_lio_backend/types.hpp"

namespace point_lio_backend {
namespace {

rclcpp::Time toRosTime(double seconds) {
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    return rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  const double whole = std::floor(seconds);
  const int32_t sec = static_cast<int32_t>(whole);
  const uint32_t nanosec =
      static_cast<uint32_t>(std::llround((seconds - whole) * 1.0e9));
  return rclcpp::Time(sec, nanosec, RCL_ROS_TIME);
}

Eigen::Isometry3d odomPoseToEigen(const nav_msgs::msg::Odometry& odom) {
  geometry_msgs::msg::Transform transform;
  transform.translation.x = odom.pose.pose.position.x;
  transform.translation.y = odom.pose.pose.position.y;
  transform.translation.z = odom.pose.pose.position.z;
  transform.rotation = odom.pose.pose.orientation;
  return tf2::transformToEigen(transform);
}

}  // namespace

struct PendingPair {
  double stamp = 0.0;
  int64_t stamp_ns = 0;
  Eigen::Isometry3d pose_raw = Eigen::Isometry3d::Identity();
  PointCloudPtr cloud;
};

struct WaitingCloud {
  double stamp = 0.0;
  int64_t stamp_ns = 0;
  PointCloudPtr cloud;
};

class BackendNode : public rclcpp::Node {
 public:
  BackendNode() : Node("point_lio_backend") {
    loadParameters();
    scan_context_ = std::make_unique<ScanContext>(params_.scan_context);
    loop_detector_ = std::make_unique<LoopDetector>(params_.scan_context, params_.icp);
    pose_graph_ = std::make_unique<PoseGraph>(params_.graph);
    keyframe_manager_ = std::make_unique<KeyframeManager>(params_.keyframe);
    createInterfaces();
    worker_ = std::thread([this]() { workerLoop(); });
    RCLCPP_INFO(get_logger(),
                "point_lio_backend started: odom=%s cloud=%s world=%s kf[max=%d] queue=%d",
                params_.odom_topic.c_str(), params_.cloud_topic.c_str(),
                params_.world_frame.c_str(), params_.keyframe.max_keyframes,
                params_.runtime.pending_clouds);
  }

  ~BackendNode() override {
    stop_.store(true);
    queue_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  // ── 参数 ────────────────────────────────────────────────────────────────
  template <typename T>
  T declareParam(const std::string& name, const T& default_value) {
    return this->declare_parameter<T>(name, default_value);
  }

  void loadParameters() {
    params_.world_frame = declareParam<std::string>("world_frame", params_.world_frame);
    params_.odom_frame = declareParam<std::string>("odom_frame", params_.odom_frame);
    params_.body_frame = declareParam<std::string>("body_frame", params_.body_frame);
    params_.odom_topic = declareParam<std::string>("odom_topic", params_.odom_topic);
    params_.cloud_topic = declareParam<std::string>("cloud_topic", params_.cloud_topic);
    params_.path_topic = declareParam<std::string>("path_topic", params_.path_topic);
    params_.odom_out_topic =
        declareParam<std::string>("odom_out_topic", params_.odom_out_topic);
    params_.map_topic = declareParam<std::string>("map_topic", params_.map_topic);
    params_.loop_marker_topic =
        declareParam<std::string>("loop_marker_topic", params_.loop_marker_topic);
    params_.status_topic = declareParam<std::string>("status_topic", params_.status_topic);

    auto& kf = params_.keyframe;
    kf.min_distance = declareParam<double>("keyframe.min_distance", kf.min_distance);
    kf.min_rotation_deg =
        declareParam<double>("keyframe.min_rotation_deg", kf.min_rotation_deg);
    kf.max_interval_sec =
        declareParam<double>("keyframe.max_interval_sec", kf.max_interval_sec);
    kf.min_interval_sec =
        declareParam<double>("keyframe.min_interval_sec", kf.min_interval_sec);
    kf.submap_frames = declareParam<int>("keyframe.submap_frames", kf.submap_frames);
    kf.submap_voxel = declareParam<double>("keyframe.submap_voxel", kf.submap_voxel);
    kf.max_keyframes = declareParam<int>("keyframe.max_keyframes", kf.max_keyframes);

    auto& sc = params_.scan_context;
    sc.rings = declareParam<int>("scan_context.rings", sc.rings);
    sc.sectors = declareParam<int>("scan_context.sectors", sc.sectors);
    sc.max_radius = declareParam<double>("scan_context.max_radius", sc.max_radius);
    sc.ring_key_threshold =
        declareParam<double>("scan_context.ring_key_threshold", sc.ring_key_threshold);
    sc.sc_distance_threshold =
        declareParam<double>("scan_context.sc_distance_threshold", sc.sc_distance_threshold);
    sc.top_candidates = declareParam<int>("scan_context.top_candidates", sc.top_candidates);
    sc.min_index_gap = declareParam<int>("scan_context.min_index_gap", sc.min_index_gap);
    sc.min_time_gap_sec =
        declareParam<double>("scan_context.min_time_gap_sec", sc.min_time_gap_sec);
    sc.min_travel_m = declareParam<double>("scan_context.min_travel_m", sc.min_travel_m);

    auto& icp = params_.icp;
    icp.coarse_correspondence =
        declareParam<double>("icp.coarse_correspondence", icp.coarse_correspondence);
    icp.max_correspondence =
        declareParam<double>("icp.max_correspondence", icp.max_correspondence);
    icp.max_iterations = declareParam<int>("icp.max_iterations", icp.max_iterations);
    icp.coarse_iterations =
        declareParam<int>("icp.coarse_iterations", icp.coarse_iterations);
    icp.match_voxel = declareParam<double>("icp.match_voxel", icp.match_voxel);
    icp.max_match_points =
        declareParam<int>("icp.max_match_points", icp.max_match_points);
    icp.stop_at_first = declareParam<bool>("icp.stop_at_first", icp.stop_at_first);
    icp.time_budget_ms = declareParam<double>("icp.time_budget_ms", icp.time_budget_ms);
    icp.max_fitness = declareParam<double>("icp.max_fitness", icp.max_fitness);
    icp.min_inlier_ratio = declareParam<double>("icp.min_inlier_ratio", icp.min_inlier_ratio);
    icp.max_correction_trans =
        declareParam<double>("icp.max_correction_trans", icp.max_correction_trans);
    icp.max_correction_rot_deg =
        declareParam<double>("icp.max_correction_rot_deg", icp.max_correction_rot_deg);

    auto& graph = params_.graph;
    graph.odom_trans_sigma =
        declareParam<double>("graph.odom_trans_sigma", graph.odom_trans_sigma);
    graph.odom_rot_sigma_deg =
        declareParam<double>("graph.odom_rot_sigma_deg", graph.odom_rot_sigma_deg);
    graph.loop_trans_sigma =
        declareParam<double>("graph.loop_trans_sigma", graph.loop_trans_sigma);
    graph.loop_rot_sigma_deg =
        declareParam<double>("graph.loop_rot_sigma_deg", graph.loop_rot_sigma_deg);
    graph.huber_delta = declareParam<double>("graph.huber_delta", graph.huber_delta);
    graph.max_iterations = declareParam<int>("graph.max_iterations", graph.max_iterations);
    graph.num_threads = declareParam<int>("graph.num_threads", graph.num_threads);
    graph.max_loop_residual_trans = declareParam<double>(
        "graph.max_loop_residual_trans", graph.max_loop_residual_trans);
    graph.max_loop_residual_rot_deg = declareParam<double>(
        "graph.max_loop_residual_rot_deg", graph.max_loop_residual_rot_deg);
    graph.remove_outlier_loops =
        declareParam<bool>("graph.remove_outlier_loops", graph.remove_outlier_loops);

    auto& out = params_.output;
    out.map_publish_period =
        declareParam<double>("output.map_publish_period", out.map_publish_period);
    out.map_voxel = declareParam<double>("output.map_voxel", out.map_voxel);
    out.correction_tau =
        declareParam<double>("output.correction_tau", out.correction_tau);
    out.publish_odom = declareParam<bool>("output.publish_odom", out.publish_odom);
    out.publish_raw_path =
        declareParam<bool>("output.publish_raw_path", out.publish_raw_path);

    auto& rt = params_.runtime;
    rt.pending_clouds =
        declareParam<int>("runtime.pending_clouds", rt.pending_clouds);
    rt.reset_on_jump_m =
        declareParam<double>("runtime.reset_on_jump_m", rt.reset_on_jump_m);
    rt.stamp_tolerance_sec =
        declareParam<double>("runtime.stamp_tolerance_sec", rt.stamp_tolerance_sec);
    rt.max_loop_solve_sec =
        declareParam<double>("runtime.max_loop_solve_sec", rt.max_loop_solve_sec);
    rt.profile = declareParam<bool>("runtime.profile", rt.profile);

    max_queue_ = static_cast<size_t>(std::max(1, rt.pending_clouds));
    if (rt.stamp_tolerance_sec <= 0.0) {
      rt.stamp_tolerance_sec = 0.05;
    }
    if (params_.keyframe.max_keyframes < 1) {
      params_.keyframe.max_keyframes = 1;
    }
  }

  void createInterfaces() {
    io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = io_group_;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        params_.odom_topic, rclcpp::QoS(20),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odomCallback(msg); },
        sub_options);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        params_.cloud_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          cloudCallback(msg);
        },
        sub_options);

    path_pub_ = create_publisher<nav_msgs::msg::Path>(params_.path_topic, rclcpp::QoS(10));
    path_raw_pub_ =
        create_publisher<nav_msgs::msg::Path>(params_.path_topic + "_raw", rclcpp::QoS(10));
    odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(params_.odom_out_topic, rclcpp::QoS(20));
    map_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(params_.map_topic, rclcpp::QoS(1));
    loop_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        params_.loop_marker_topic, rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        params_.status_topic, rclcpp::QoS(10));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() { tfTimerCallback(); }, timer_group_);
    status_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000),
        [this]() { statusTimerCallback(); }, timer_group_);
  }

  // ── 订阅回调：stamp 配对，重活全部丢给 worker ───────────────────────────
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    try {
      const double stamp = rclcpp::Time(msg->header.stamp).seconds();
      const Eigen::Isometry3d pose = odomPoseToEigen(*msg);

      bool jumped = false;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (has_prev_odom_ && params_.runtime.reset_on_jump_m > 0.0) {
          const double jump =
              (pose.translation() - prev_odom_pose_.translation()).norm();
          if (jump > params_.runtime.reset_on_jump_m) {
            jumped = true;
          }
        }
        latest_odom_pose_ = pose;
        latest_odom_stamp_ = stamp;
        has_odom_ = true;
        prev_odom_pose_ = pose;
        has_prev_odom_ = true;
      }

      if (jumped) {
        RCLCPP_WARN(get_logger(),
                    "odom translation jump > %.2f m, resetting backend graph",
                    params_.runtime.reset_on_jump_m);
        requestReset();
      }

      std::vector<PendingPair> ready;
      {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        odom_buffer_.emplace_back(stamp, pose);
        while (odom_buffer_.size() > kMaxOdomBuffer) {
          odom_buffer_.pop_front();
        }
        // 先到的点云在等这帧 odom。
        for (auto it = waiting_clouds_.begin(); it != waiting_clouds_.end();) {
          if (std::fabs(it->stamp - stamp) <= params_.runtime.stamp_tolerance_sec) {
            ready.push_back(PendingPair{it->stamp, it->stamp_ns, pose, it->cloud});
            it = waiting_clouds_.erase(it);
          } else {
            ++it;
          }
        }
      }
      for (auto& pair : ready) {
        enqueuePair(std::move(pair));
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "odom callback exception: %s", e.what());
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    try {
      PointCloudPtr cloud(new PointCloudT());
      pcl::fromROSMsg(*msg, *cloud);
      const rclcpp::Time stamp_ros(msg->header.stamp);
      const double stamp = stamp_ros.seconds();
      const int64_t stamp_ns = stamp_ros.nanoseconds();

      bool matched = false;
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      size_t dropped = 0;
      {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        // 过期的等待点云（没有匹配 odom）丢弃。
        const double ttl = std::max(0.5, 10.0 * params_.runtime.stamp_tolerance_sec);
        while (!waiting_clouds_.empty() &&
               stamp - waiting_clouds_.front().stamp > ttl) {
          waiting_clouds_.pop_front();
          ++dropped;
        }
        // 找 stamp 最近的 odom（一对一消费）。
        auto best = odom_buffer_.end();
        double best_dt = params_.runtime.stamp_tolerance_sec;
        for (auto it = odom_buffer_.begin(); it != odom_buffer_.end(); ++it) {
          const double dt = std::fabs(it->first - stamp);
          if (dt <= best_dt) {
            best_dt = dt;
            best = it;
          }
        }
        if (best != odom_buffer_.end()) {
          pose = best->second;
          odom_buffer_.erase(best);
          matched = true;
        } else {
          // point_lio 可能先发 cloud 再发 odom：短暂缓存等待配对。
          waiting_clouds_.emplace_back(WaitingCloud{stamp, stamp_ns, cloud});
          const size_t max_waiting = std::max<size_t>(8, 4 * max_queue_);
          while (waiting_clouds_.size() > max_waiting) {
            waiting_clouds_.pop_front();
            ++dropped;
          }
        }
      }
      if (dropped > 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "dropped %zu unmatched cloud(s): no odom within %.3f s",
                             dropped, params_.runtime.stamp_tolerance_sec);
      }
      if (matched) {
        enqueuePair(PendingPair{stamp, stamp_ns, pose, cloud});
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "cloud callback exception: %s", e.what());
    }
  }

  void enqueuePair(PendingPair&& pair) {
    bool dropped = false;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (queue_.size() >= max_queue_) {
        queue_.pop_front();
        dropped = true;
      }
      queue_.push_back(std::move(pair));
    }
    if (dropped) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "worker queue full (max %zu), dropped oldest cloud", max_queue_);
    }
    queue_cv_.notify_one();
  }

  void requestReset() {
    reset_requested_.store(true);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_.clear();
    }
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      waiting_clouds_.clear();
      odom_buffer_.clear();
    }
    queue_cv_.notify_all();
  }

  // ── worker ──────────────────────────────────────────────────────────────
  size_t queueSize() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return queue_.size();
  }

  void workerLoop() {
    while (true) {
      PendingPair job;
      bool has_job = false;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {
          return stop_.load() || reset_requested_.load() || !queue_.empty();
        });
        if (stop_.load()) {
          return;
        }
        if (reset_requested_.exchange(false)) {
          lock.unlock();
          handleReset();
          continue;
        }
        if (queue_.empty()) {
          continue;
        }
        job = std::move(queue_.front());
        queue_.pop_front();
        has_job = true;
      }
      if (has_job) {
        processPair(job);
      }
    }
  }

  void handleReset() {
    try {
      keyframe_manager_->reset();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        keyframes_.clear();
        keyframe_stamp_ns_.clear();
        loops_.clear();
        correction_.setIdentity();
        target_correction_.setIdentity();
        last_smooth_stamp_ = -1.0;
        last_solve_ms_ = 0.0;
        last_fitness_ = 0.0;
        map_accum_.reset();
        map_keyframes_applied_ = 0;
        map_dirty_ = false;
        last_rebuild_correction_.setIdentity();
      }
      RCLCPP_WARN(get_logger(), "backend reset: graph/keyframes cleared, re-anchoring");
      publishPath();
      if (params_.output.publish_raw_path) {
        publishRawPath();
      }
      publishMapPoints();
      publishLoopMarkers();
      last_map_publish_ = std::chrono::steady_clock::now();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "reset exception: %s", e.what());
    }
  }

  void processPair(const PendingPair& job) {
    try {
      if (static_cast<int>(keyframes_.size()) >= params_.keyframe.max_keyframes) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "max_keyframes (%d) reached, no new keyframes",
                             params_.keyframe.max_keyframes);
        return;
      }
      keyframe_manager_->addFrame(job.stamp, job.pose_raw, job.cloud);
      if (!keyframe_manager_->shouldCreate(job.stamp, job.pose_raw)) {
        return;
      }

      Keyframe keyframe;
      if (!keyframe_manager_->buildNextKeyframe(&keyframe)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "keyframe build rejected (max_keyframes=%d)",
                             params_.keyframe.max_keyframes);
        return;
      }
      if (keyframe.cloud && !keyframe.cloud->empty()) {
        keyframe.descriptor = scan_context_->describe(*keyframe.cloud);
        keyframe.ring_key = scan_context_->ringKey(keyframe.descriptor);
      } else {
        keyframe.descriptor = Eigen::MatrixXd::Zero(params_.scan_context.rings,
                                                    params_.scan_context.sectors);
        keyframe.ring_key = Eigen::VectorXd::Zero(params_.scan_context.rings);
      }

      // 优化尚未覆盖的新关键帧：按前端相对运动从上一关键帧推 map 系位姿。
      if (!keyframes_.empty()) {
        const Keyframe& last = keyframes_.back();
        keyframe.pose_opt = last.pose_opt * last.pose_raw.inverse() * keyframe.pose_raw;
      } else {
        keyframe.pose_opt = keyframe.pose_raw;
      }

      const auto t_detect0 = std::chrono::steady_clock::now();
      const std::optional<LoopConstraint> loop =
          loop_detector_->detect(keyframe, keyframes_);
      const double detect_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t_detect0).count();

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        keyframes_.push_back(keyframe);
        keyframe_stamp_ns_.push_back(job.stamp_ns);
        if (loop.has_value()) {
          loops_.push_back(*loop);
          last_fitness_ = loop->fitness;
        }
        updateTargetCorrectionLocked();
      }

      if (loop.has_value()) {
        RCLCPP_INFO(get_logger(),
                    "loop detected: from=%d to=%d fitness=%.3f inliers=%.2f",
                    loop->from, loop->to, loop->fitness, loop->inlier_ratio);
      }

      // 契约：每个新关键帧都发一次 map 系 path；优化后再重发。
      const auto t_pub0 = std::chrono::steady_clock::now();
      publishPath();
      if (params_.output.publish_raw_path) {
        publishRawPath();
      }
      publishLoopMarkers();
      publishMapPointsIfDue(false);
      const double pub_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t_pub0).count();

      if (loop.has_value()) {
        runOptimization();
      }
      if (params_.runtime.profile) {
        RCLCPP_INFO(get_logger(),
                    "[profile] kf=%d pts=%zu detect=%.1fms publish=%.1fms queue=%zu",
                    keyframe.id, keyframe.cloud ? keyframe.cloud->size() : 0u, detect_ms,
                    pub_ms, queueSize());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "worker keyframe exception: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "worker keyframe unknown exception");
    }
  }

  void runOptimization() {
    try {
      const auto start = std::chrono::steady_clock::now();
      std::vector<bool> active;
      bool ok = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ok = pose_graph_->optimize(keyframes_, loops_, &active);
        const auto end = std::chrono::steady_clock::now();
        last_solve_ms_ =
            std::chrono::duration<double, std::milli>(end - start).count();
        if (active.size() == loops_.size()) {
          for (size_t i = 0; i < loops_.size(); ++i) {
            loops_[i].active = active[i];
          }
        }
        updateTargetCorrectionLocked();
      }
      if (!ok) {
        RCLCPP_WARN(get_logger(), "pose graph optimization failed; keeping last solution");
        return;
      }
      RCLCPP_INFO(get_logger(), "pose graph optimized %zu keyframes, %zu loops in %.1f ms",
                  keyframes_.size(), loops_.size(), last_solve_ms_);
      publishPath();
      publishLoopMarkers();
      // 回环常成串到达（连续几个关键帧各触发一次优化），且每条新回环对历史位姿的
      // 修正量通常越来越小。整图重建是 O(全部关键帧)（实测 100~1400 ms，会阻塞 worker），
      // 所以只有纠偏量真的变了才置脏，再由 map_publish_period 节流重建。
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const Eigen::Isometry3d delta = correction_ * last_rebuild_correction_.inverse();
        const double dt = delta.translation().norm();
        const double da = Eigen::AngleAxisd(delta.linear()).angle();
        if (dt > 0.03 || da > 0.3 * M_PI / 180.0) {
          map_dirty_ = true;
          last_rebuild_correction_ = correction_;
        }
      }
      publishMapPointsIfDue(false);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "optimization exception: %s", e.what());
    }
  }

  // ── 发布 ────────────────────────────────────────────────────────────────
  struct TimedPose {
    double stamp = 0.0;
    int64_t stamp_ns = 0;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  };

  rclcpp::Time poseStamp(const TimedPose& pose) const {
    if (pose.stamp_ns > 0) {
      return rclcpp::Time(pose.stamp_ns);
    }
    return toRosTime(pose.stamp);
  }

  void publishPath() {
    std::vector<TimedPose> poses;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      poses.reserve(keyframes_.size());
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        TimedPose tp;
        tp.stamp = keyframes_[i].stamp;
        tp.stamp_ns =
            i < keyframe_stamp_ns_.size() ? keyframe_stamp_ns_[i] : 0;
        tp.pose = keyframes_[i].pose_opt;
        poses.push_back(tp);
      }
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = params_.world_frame;
    path.header.stamp = poses.empty() ? now() : poseStamp(poses.back());
    path.poses.reserve(poses.size());
    for (const auto& item : poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = params_.world_frame;
      ps.header.stamp = poseStamp(item);
      ps.pose = tf2::toMsg(item.pose);
      path.poses.push_back(std::move(ps));
    }
    path_pub_->publish(path);
  }

  void publishRawPath() {
    std::vector<TimedPose> poses;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      poses.reserve(keyframes_.size());
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        TimedPose tp;
        tp.stamp = keyframes_[i].stamp;
        tp.stamp_ns =
            i < keyframe_stamp_ns_.size() ? keyframe_stamp_ns_[i] : 0;
        tp.pose = keyframes_[i].pose_raw;
        poses.push_back(tp);
      }
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = params_.odom_frame;
    path.header.stamp = poses.empty() ? now() : poseStamp(poses.back());
    path.poses.reserve(poses.size());
    for (const auto& item : poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = params_.odom_frame;
      ps.header.stamp = poseStamp(item);
      ps.pose = tf2::toMsg(item.pose);
      path.poses.push_back(std::move(ps));
    }
    path_raw_pub_->publish(path);
  }

  void publishMapPointsIfDue(bool force) {
    const auto now_tp = std::chrono::steady_clock::now();
    if (!force && params_.output.map_publish_period > 0.0) {
      const double elapsed =
          std::chrono::duration<double>(now_tp - last_map_publish_).count();
      if (elapsed < params_.output.map_publish_period) {
        return;
      }
    }
    publishMapPoints(map_dirty_);
    map_dirty_ = false;
    last_map_publish_ = now_tp;
  }

  // 把 keyframes_[from, end) 的点按 pose_opt 变到 map 系后追加到 out。
  void appendKeyframesToMap(size_t from, PointCloudT* out) const {
    for (size_t i = from; i < keyframes_.size(); ++i) {
      const Keyframe& kf = keyframes_[i];
      if (!kf.cloud) {
        continue;
      }
      const Eigen::Isometry3d& T = kf.pose_opt;
      const Eigen::Matrix3d R = T.linear();
      const Eigen::Vector3d t = T.translation();
      out->reserve(out->size() + kf.cloud->size());
      for (const auto& p : kf.cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          continue;
        }
        const Eigen::Vector3d q = R * Eigen::Vector3d(p.x, p.y, p.z) + t;
        PointT pt = p;
        pt.x = static_cast<float>(q.x());
        pt.y = static_cast<float>(q.y());
        pt.z = static_cast<float>(q.z());
        out->push_back(pt);
      }
    }
  }

  PointCloudT::Ptr voxelize(const PointCloudT::Ptr& in) const {
    if (params_.output.map_voxel <= 0.0 || in->empty()) {
      return in;
    }
    PointCloudT::Ptr filtered(new PointCloudT());
    pcl::VoxelGrid<PointT> voxel;
    const float leaf = static_cast<float>(params_.output.map_voxel);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(in);
    voxel.filter(*filtered);
    return filtered;
  }

  // 地图点云是增量维护的：每来一个关键帧只把"新增的那几帧"变换+体素化后并进
  // map_accum_，而不是每次重建全图。全量重建只在回环优化改变了历史位姿之后做。
  // 否则长时建图会退化成 O(关键帧数) 的重复变换（2000 帧 × 5k 点/2 s 就能吃满一个核）。
  void publishMapPoints(bool rebuild_all = false) {
    // 只在 worker 线程调用；keyframes_ 的写者也是 worker，读安全。
    if (rebuild_all || !map_accum_ ||
        map_keyframes_applied_ > keyframes_.size()) {
      map_accum_.reset(new PointCloudT());
      map_keyframes_applied_ = 0;
      appendKeyframesToMap(0, map_accum_.get());
      map_accum_ = voxelize(map_accum_);
      map_keyframes_applied_ = keyframes_.size();
    } else if (map_keyframes_applied_ < keyframes_.size()) {
      PointCloudT::Ptr fresh(new PointCloudT());
      appendKeyframesToMap(map_keyframes_applied_, fresh.get());
      fresh = voxelize(fresh);
      *map_accum_ += *fresh;
      map_keyframes_applied_ = keyframes_.size();
      // 增量合并会让重叠区域的点重复累积；超过阈值时整体压一次，保持内存与带宽有界。
      if (map_accum_->size() > kMapCompressThreshold) {
        map_accum_ = voxelize(map_accum_);
      }
    }
    PointCloudT::Ptr merged = map_accum_ ? map_accum_ : PointCloudT::Ptr(new PointCloudT());

    double last_stamp = 0.0;
    int64_t last_stamp_ns = 0;
    if (!keyframes_.empty()) {
      last_stamp = keyframes_.back().stamp;
      if (!keyframe_stamp_ns_.empty() && keyframe_stamp_ns_.back() > 0) {
        last_stamp_ns = keyframe_stamp_ns_.back();
      }
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*merged, msg);
    msg.header.frame_id = params_.world_frame;
    msg.header.stamp = keyframes_.empty()
                            ? now()
                            : (last_stamp_ns > 0 ? rclcpp::Time(last_stamp_ns)
                                                 : toRosTime(last_stamp));
    map_pub_->publish(msg);
  }

  void publishLoopMarkers() {
    // 只在 worker 线程调用。
    visualization_msgs::msg::MarkerArray array;
    const rclcpp::Time stamp =
        keyframes_.empty()
            ? now()
            : (!keyframe_stamp_ns_.empty() && keyframe_stamp_ns_.back() > 0
                   ? rclcpp::Time(keyframe_stamp_ns_.back())
                   : toRosTime(keyframes_.back().stamp));

    visualization_msgs::msg::Marker loops;
    loops.header.frame_id = params_.world_frame;
    loops.header.stamp = stamp;
    loops.ns = "loops";
    loops.id = 0;
    loops.type = visualization_msgs::msg::Marker::LINE_LIST;
    loops.action = visualization_msgs::msg::Marker::ADD;
    loops.scale.x = 0.05;
    loops.color.r = 0.1f;
    loops.color.g = 0.9f;
    loops.color.b = 0.1f;
    loops.color.a = 0.9f;
    loops.pose.orientation.w = 1.0;
    for (const auto& loop : loops_) {
      if (loop.from < 0 || loop.to < 0 ||
          loop.from >= static_cast<int>(keyframes_.size()) ||
          loop.to >= static_cast<int>(keyframes_.size())) {
        continue;
      }
      const Eigen::Vector3d a = keyframes_[loop.from].pose_opt.translation();
      const Eigen::Vector3d b = keyframes_[loop.to].pose_opt.translation();
      geometry_msgs::msg::Point pa;
      pa.x = a.x();
      pa.y = a.y();
      pa.z = a.z();
      geometry_msgs::msg::Point pb;
      pb.x = b.x();
      pb.y = b.y();
      pb.z = b.z();
      loops.points.push_back(pa);
      loops.points.push_back(pb);
    }

    visualization_msgs::msg::Marker keyframes;
    keyframes.header.frame_id = params_.world_frame;
    keyframes.header.stamp = stamp;
    keyframes.ns = "keyframes";
    keyframes.id = 1;
    keyframes.type = visualization_msgs::msg::Marker::POINTS;
    keyframes.action = visualization_msgs::msg::Marker::ADD;
    keyframes.scale.x = 0.15;
    keyframes.scale.y = 0.15;
    keyframes.color.r = 0.1f;
    keyframes.color.g = 0.6f;
    keyframes.color.b = 1.0f;
    keyframes.color.a = 0.9f;
    keyframes.pose.orientation.w = 1.0;
    keyframes.points.reserve(keyframes_.size());
    for (const auto& kf : keyframes_) {
      const Eigen::Vector3d t = kf.pose_opt.translation();
      geometry_msgs::msg::Point p;
      p.x = t.x();
      p.y = t.y();
      p.z = t.z();
      keyframes.points.push_back(p);
    }

    array.markers.push_back(std::move(loops));
    array.markers.push_back(std::move(keyframes));
    loop_marker_pub_->publish(array);
  }

  // 20 Hz：TF map->camera_init + /backend/odom（map->body）。
  void tfTimerCallback() {
    try {
      double stamp = 0.0;
      Eigen::Isometry3d raw = Eigen::Isometry3d::Identity();
      bool has_odom = false;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        stamp = latest_odom_stamp_;
        raw = latest_odom_pose_;
        has_odom = has_odom_;
      }
      if (!has_odom) {
        return;
      }

      Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        updateCorrectionLocked(stamp);
        correction = correction_;
      }

      geometry_msgs::msg::TransformStamped tf_msg = tf2::eigenToTransform(correction);
      tf_msg.header.stamp = toRosTime(stamp);
      tf_msg.header.frame_id = params_.world_frame;
      tf_msg.child_frame_id = params_.odom_frame;
      tf_broadcaster_->sendTransform(tf_msg);

      if (params_.output.publish_odom) {
        const Eigen::Isometry3d map_body = correction * raw;
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = toRosTime(stamp);
        odom.header.frame_id = params_.world_frame;
        odom.child_frame_id = params_.body_frame;
        odom.pose.pose = tf2::toMsg(map_body);
        odom_pub_->publish(odom);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "tf timer exception: %s", e.what());
    }
  }

  void statusTimerCallback() {
    try {
      size_t keyframes = 0;
      size_t loops = 0;
      double last_solve_ms = 0.0;
      double last_fitness = 0.0;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        keyframes = keyframes_.size();
        loops = loops_.size();
        last_solve_ms = last_solve_ms_;
        last_fitness = last_fitness_;
      }

      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = now();
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.name = "point_lio_backend";
      status.hardware_id = "point_lio_backend";
      status.message = "running";
      auto addValue = [&status](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(std::move(kv));
      };
      addValue("keyframes", std::to_string(keyframes));
      addValue("loops", std::to_string(loops));
      addValue("last_solve_ms", std::to_string(last_solve_ms));
      addValue("last_fitness", std::to_string(last_fitness));
      array.status.push_back(std::move(status));
      status_pub_->publish(array);
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "status timer exception: %s", e.what());
    }
  }

  // ── 纠偏平滑 ────────────────────────────────────────────────────────────
  // 调用时须持有 state_mutex_。
  void updateTargetCorrectionLocked() {
    if (keyframes_.empty()) {
      target_correction_.setIdentity();
      return;
    }
    const Keyframe& kf = keyframes_.back();
    target_correction_ = kf.pose_opt * kf.pose_raw.inverse();
  }

  // 调用时须持有 state_mutex_。
  void updateCorrectionLocked(double stamp) {
    if (last_smooth_stamp_ < 0.0 || stamp < last_smooth_stamp_) {
      correction_ = target_correction_;
      last_smooth_stamp_ = stamp;
      return;
    }
    const double dt = stamp - last_smooth_stamp_;
    last_smooth_stamp_ = stamp;
    if (params_.output.correction_tau <= 0.0) {
      correction_ = target_correction_;
      return;
    }
    if (dt <= 0.0) {
      return;
    }
    const double alpha = 1.0 - std::exp(-dt / params_.output.correction_tau);
    const Eigen::Quaterniond q_from(correction_.linear());
    const Eigen::Quaterniond q_to(target_correction_.linear());
    const Eigen::Quaterniond q = q_from.slerp(std::clamp(alpha, 0.0, 1.0), q_to);
    correction_.linear() = q.normalized().toRotationMatrix();
    correction_.translation() =
        (1.0 - alpha) * correction_.translation() + alpha * target_correction_.translation();
  }

  // ── 成员 ────────────────────────────────────────────────────────────────
  static constexpr size_t kMaxOdomBuffer = 200;
  static constexpr size_t kMapCompressThreshold = 1'500'000;  // 增量合并后超过就整体压一次

  BackendParams params_;
  size_t max_queue_ = 2;

  std::unique_ptr<ScanContext> scan_context_;
  std::unique_ptr<LoopDetector> loop_detector_;
  std::unique_ptr<PoseGraph> pose_graph_;
  std::unique_ptr<KeyframeManager> keyframe_manager_;

  rclcpp::CallbackGroup::SharedPtr io_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_raw_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<PendingPair> queue_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> reset_requested_{false};
  std::thread worker_;

  std::mutex buffer_mutex_;
  std::deque<std::pair<double, Eigen::Isometry3d>> odom_buffer_;
  std::deque<WaitingCloud> waiting_clouds_;

  std::mutex odom_mutex_;
  Eigen::Isometry3d latest_odom_pose_ = Eigen::Isometry3d::Identity();
  double latest_odom_stamp_ = -1.0;
  bool has_odom_ = false;
  Eigen::Isometry3d prev_odom_pose_ = Eigen::Isometry3d::Identity();
  bool has_prev_odom_ = false;

  std::mutex state_mutex_;
  std::vector<Keyframe> keyframes_;
  std::vector<int64_t> keyframe_stamp_ns_;
  std::vector<LoopConstraint> loops_;
  Eigen::Isometry3d correction_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_correction_ = Eigen::Isometry3d::Identity();
  double last_smooth_stamp_ = -1.0;
  double last_solve_ms_ = 0.0;
  double last_fitness_ = 0.0;
  std::chrono::steady_clock::time_point last_map_publish_{};
  PointCloudT::Ptr map_accum_;          // map 系增量累积点云（worker 线程独占）
  size_t map_keyframes_applied_ = 0;    // map_accum_ 已并入到第几个关键帧
  bool map_dirty_ = false;              // 历史位姿被优化改过，下次发布需全量重建
  Eigen::Isometry3d last_rebuild_correction_ = Eigen::Isometry3d::Identity();
};

}  // namespace point_lio_backend

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<point_lio_backend::BackendNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  }
  rclcpp::shutdown();
  return 0;
}
