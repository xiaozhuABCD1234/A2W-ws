#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <cctype>
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
    segmenter_ = createSegmenter();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(50),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odomCallback(msg); });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { cloudCallback(msg); });
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ground_topic_, rclcpp::SensorDataQoS());
    nonground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        nonground_topic_, rclcpp::SensorDataQoS());
    RCLCPP_INFO(get_logger(), "ground segmentation: algorithm=%s ground=%s nonground=%s",
                segmenter_->name().c_str(), ground_topic_.c_str(), nonground_topic_.c_str());
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

  std::unique_ptr<GroundSegmenter> createSegmenter() {
    if (algorithm_ == "polar") return std::make_unique<PolarGroundSegmenter>(polar_params_);
    if (algorithm_ == "linefit") {
      return std::make_unique<LineFitGroundSegmenter>(linefit_params_);
    }
    if (algorithm_ != "patchwork") {
      RCLCPP_WARN(get_logger(),
                  "unknown segmentation.algorithm '%s', use polar|linefit|patchwork; "
                  "falling back to patchwork",
                  algorithm_.c_str());
      algorithm_ = "patchwork";
    }
    return std::make_unique<PatchworkGroundSegmenter>(patchwork_params_);
  }

  void loadParameters() {
    odom_topic_ = param<std::string>("odom_topic", odom_topic_);
    cloud_topic_ = param<std::string>("cloud_topic", cloud_topic_);
    ground_topic_ = param<std::string>("ground_topic", ground_topic_);
    nonground_topic_ = param<std::string>("nonground_topic", nonground_topic_);
    stamp_tolerance_ = param<double>("runtime.stamp_tolerance", stamp_tolerance_);
    gravity_align_ = param<bool>("runtime.gravity_align", gravity_align_);

    algorithm_ = param<std::string>("segmentation.algorithm", algorithm_);
    std::transform(algorithm_.begin(), algorithm_.end(), algorithm_.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    min_range_ = param<double>("segmentation.min_range", min_range_);
    max_range_ = param<double>("segmentation.max_range", max_range_);

    polar_params_.min_range = min_range_;
    polar_params_.max_range = max_range_;
    polar_params_.ring_size = param<double>("segmentation.polar.ring_size", polar_params_.ring_size);
    polar_params_.sectors = param<int>("segmentation.polar.sectors", polar_params_.sectors);
    polar_params_.min_cell_points =
        param<int>("segmentation.polar.min_cell_points", polar_params_.min_cell_points);
    polar_params_.seed_height_band =
        param<double>("segmentation.polar.seed_height_band", polar_params_.seed_height_band);
    polar_params_.ground_height_threshold = param<double>(
        "segmentation.polar.ground_height_threshold", polar_params_.ground_height_threshold);
    polar_params_.max_ground_step =
        param<double>("segmentation.polar.max_ground_step", polar_params_.max_ground_step);
    polar_params_.max_slope_deg =
        param<double>("segmentation.polar.max_slope_deg", polar_params_.max_slope_deg);
    polar_params_.ground_alpha =
        param<double>("segmentation.polar.ground_alpha", polar_params_.ground_alpha);
    polar_params_.initial_ground_tolerance = param<double>(
        "segmentation.polar.initial_ground_tolerance", polar_params_.initial_ground_tolerance);
    polar_params_.ground_estimation_range = param<double>(
        "segmentation.polar.ground_estimation_range", polar_params_.ground_estimation_range);

    linefit_params_.min_range = min_range_;
    linefit_params_.max_range = max_range_;
    linefit_params_.ring_size =
        param<double>("segmentation.linefit.ring_size", linefit_params_.ring_size);
    linefit_params_.sectors = param<int>("segmentation.linefit.sectors", linefit_params_.sectors);
    linefit_params_.num_lpr = param<int>("segmentation.linefit.num_lpr", linefit_params_.num_lpr);
    linefit_params_.num_iter =
        param<int>("segmentation.linefit.num_iter", linefit_params_.num_iter);
    linefit_params_.min_seed_points =
        param<int>("segmentation.linefit.min_seed_points", linefit_params_.min_seed_points);
    linefit_params_.th_seeds =
        param<double>("segmentation.linefit.th_seeds", linefit_params_.th_seeds);
    linefit_params_.th_dist =
        param<double>("segmentation.linefit.th_dist", linefit_params_.th_dist);
    linefit_params_.uprightness_thr =
        param<double>("segmentation.linefit.uprightness_thr", linefit_params_.uprightness_thr);
    linefit_params_.max_slope_deg =
        param<double>("segmentation.linefit.max_slope_deg", linefit_params_.max_slope_deg);
    linefit_params_.max_ground_step =
        param<double>("segmentation.linefit.max_ground_step", linefit_params_.max_ground_step);
    linefit_params_.initial_ground_tolerance = param<double>(
        "segmentation.linefit.initial_ground_tolerance",
        linefit_params_.initial_ground_tolerance);
    linefit_params_.ground_bottom_percentile = param<double>(
        "segmentation.linefit.ground_bottom_percentile",
        linefit_params_.ground_bottom_percentile);

    patchwork_params_.min_range = min_range_;
    patchwork_params_.max_range = max_range_;
    patchwork_params_.zone_ranges =
        param<std::vector<double>>("segmentation.patchwork.zone_ranges",
                                   patchwork_params_.zone_ranges);
    patchwork_params_.sectors_per_zone =
        param<std::vector<int64_t>>("segmentation.patchwork.sectors_per_zone",
                                    patchwork_params_.sectors_per_zone);
    patchwork_params_.rings_per_zone =
        param<std::vector<int64_t>>("segmentation.patchwork.rings_per_zone",
                                    patchwork_params_.rings_per_zone);
    patchwork_params_.num_iter =
        param<int>("segmentation.patchwork.num_iter", patchwork_params_.num_iter);
    patchwork_params_.num_lpr =
        param<int>("segmentation.patchwork.num_lpr", patchwork_params_.num_lpr);
    patchwork_params_.min_bucket_points = param<int>(
        "segmentation.patchwork.min_bucket_points", patchwork_params_.min_bucket_points);
    patchwork_params_.th_seeds =
        param<double>("segmentation.patchwork.th_seeds", patchwork_params_.th_seeds);
    patchwork_params_.th_dist =
        param<double>("segmentation.patchwork.th_dist", patchwork_params_.th_dist);
    patchwork_params_.uprightness_thr = param<double>(
        "segmentation.patchwork.uprightness_thr", patchwork_params_.uprightness_thr);
    patchwork_params_.flatness_thr =
        param<double>("segmentation.patchwork.flatness_thr", patchwork_params_.flatness_thr);
    patchwork_params_.max_ground_height = param<double>(
        "segmentation.patchwork.max_ground_height", patchwork_params_.max_ground_height);
    patchwork_params_.ground_bottom_percentile = param<double>(
        "segmentation.patchwork.ground_bottom_percentile",
        patchwork_params_.ground_bottom_percentile);
    patchwork_params_.enable_temporal_revert = param<bool>(
        "segmentation.patchwork.enable_temporal_revert",
        patchwork_params_.enable_temporal_revert);
    patchwork_params_.temporal_revert_ratio = param<double>(
        "segmentation.patchwork.temporal_revert_ratio",
        patchwork_params_.temporal_revert_ratio);
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
        gravity_rotation =
            Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * rotation;
      }
      const auto result = segmenter_->segment(*frame.cloud, gravity_rotation);
      publish(ground_pub_, *result.ground, frame.header);
      publish(nonground_pub_, *result.nonground, frame.header);
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

  PolarParams polar_params_;
  LineFitParams linefit_params_;
  PatchworkParams patchwork_params_;
  std::unique_ptr<GroundSegmenter> segmenter_;
  std::string algorithm_ = "patchwork";
  double min_range_ = 0.5;
  double max_range_ = 30.0;
  std::string odom_topic_ = "/aft_mapped_to_init";
  std::string cloud_topic_ = "/cloud_registered_body";
  std::string ground_topic_ = "/ground_segmentation/ground";
  std::string nonground_topic_ = "/ground_segmentation/nonground";
  double stamp_tolerance_ = 0.05;
  bool gravity_align_ = true;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_pub_;
  std::mutex buffer_mutex_;
  std::deque<std::pair<double, Eigen::Isometry3d>> odom_buffer_;
  std::deque<Frame> waiting_clouds_;
};

}  // namespace
}  // namespace ground_segmentation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ground_segmentation::GroundSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
