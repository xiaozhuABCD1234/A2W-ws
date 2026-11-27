#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include "esdf/edt.hpp"

namespace esdf {
namespace {

class EsdfNode : public rclcpp::Node {
 public:
  EsdfNode() : Node("esdf") {
    loadParameters();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        distance_topic_, rclcpp::QoS(1).transient_local().reliable());
    meta_pub_ = create_publisher<nav_msgs::msg::MapMetaData>(
        meta_topic_, rclcpp::QoS(1).transient_local().reliable());
    distance_pub_ = create_publisher<std_msgs::msg::Float32>(query_response_topic_, rclcpp::QoS(10));
    if (publish_points_) {
      points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          points_topic_, rclcpp::QoS(1).reliable());
    }
    source_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        source_topic_, rclcpp::QoS(1).reliable(),
        [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { sourceCallback(msg); });
    query_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        query_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) { queryCallback(msg); });

    RCLCPP_INFO(get_logger(), "esdf started: %s -> %s (max %.1f m, points %s)", source_topic_.c_str(),
                distance_topic_.c_str(), params_.max_distance, publish_points_ ? "on" : "off");
  }

 private:
  template <typename T>
  T param(const std::string& name, const T& default_value) {
    return declare_parameter<T>(name, default_value);
  }

  void loadParameters() {
    source_topic_ = param<std::string>("source_topic", source_topic_);
    distance_topic_ = param<std::string>("distance_topic", distance_topic_);
    meta_topic_ = param<std::string>("meta_topic", meta_topic_);
    query_topic_ = param<std::string>("query_topic", query_topic_);
    query_response_topic_ = param<std::string>("query_response_topic", query_response_topic_);
    points_topic_ = param<std::string>("points_topic", points_topic_);
    publish_points_ = param<bool>("publish_points", publish_points_);
    point_stride_ = param<int>("point_stride", point_stride_);
    params_.resolution = param<double>("esdf.resolution", params_.resolution);
    params_.max_distance = param<double>("esdf.max_distance", params_.max_distance);
    params_.occupied_threshold = param<int>("esdf.occupied_threshold", params_.occupied_threshold);
    params_.unknown_is_obstacle =
        param<bool>("esdf.unknown_is_obstacle", params_.unknown_is_obstacle);
    downsample_ = param<int>("esdf.downsample", downsample_);
    points_period_ms_ = param<int>("points_period_ms", points_period_ms_);
    points_max_ = param<int>("points_max", points_max_);
    downsample_ = std::max(1, downsample_);
    points_period_ms_ = std::max(0, points_period_ms_);
    points_max_ = std::max(1000, points_max_);
    point_stride_ = std::max(1, point_stride_);
    params_.max_distance = std::max(0.1, params_.max_distance);
  }

  void sourceCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
    const int source_width = static_cast<int>(msg->info.width);
    const int source_height = static_cast<int>(msg->info.height);
    if (source_width <= 0 || source_height <= 0 ||
        msg->data.size() < static_cast<size_t>(source_width) * static_cast<size_t>(source_height)) {
      return;
    }

    EdtParams params = params_;
    const double source_resolution =
        msg->info.resolution > 0.0f ? static_cast<double>(msg->info.resolution) : params.resolution;
    const int step = downsample_;
    const int width = (source_width + step - 1) / step;
    const int height = (source_height + step - 1) / step;
    params.resolution = source_resolution * step;

    const auto start = std::chrono::steady_clock::now();
    std::vector<int8_t> reduced(static_cast<size_t>(width) * static_cast<size_t>(height), -1);
    size_t obstacles = 0;
    size_t free_cells = 0;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int8_t value = -1;
        for (int oy = 0; oy < step; ++oy) {
          const int sy = y * step + oy;
          if (sy >= source_height) break;
          for (int ox = 0; ox < step; ++ox) {
            const int sx = x * step + ox;
            if (sx >= source_width) break;
            const int8_t cell = msg->data[static_cast<size_t>(sy) *
                                              static_cast<size_t>(source_width) +
                                          static_cast<size_t>(sx)];
            if (cell >= params.occupied_threshold) {
              value = cell;
              break;
            }
            if (cell >= 0 && value < 0) value = 0;
          }
          if (value >= params.occupied_threshold) break;
        }
        reduced[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)] =
            value;
        if (value >= params.occupied_threshold) {
          ++obstacles;
        } else if (value >= 0) {
          ++free_cells;
        }
      }
    }

    std::vector<float> distance;
    distanceTransform(reduced, width, height, params, &distance);
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

    geometry_msgs::msg::Pose origin = msg->info.origin;
    origin.position.z = 0.0;
    origin.orientation.w = 1.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      distance_ = std::move(distance);
      width_ = width;
      height_ = height;
      resolution_ = params.resolution;
      origin_x_ = origin.position.x;
      origin_y_ = origin.position.y;
      frame_id_ = msg->header.frame_id;
      stamp_ = rclcpp::Time(msg->header.stamp);
    }
    publish(origin, obstacles, free_cells, elapsed_ms);
  }

  void publish(const geometry_msgs::msg::Pose& origin, size_t obstacles, size_t free_cells,
               double elapsed_ms) {
    sensor_msgs::msg::Image image;
    nav_msgs::msg::MapMetaData meta;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (width_ <= 0 || height_ <= 0) return;
      const size_t bytes = static_cast<size_t>(width_) * static_cast<size_t>(height_) *
                           sizeof(float);
      image.header.frame_id = frame_id_;
      image.header.stamp = stamp_;
      image.height = static_cast<uint32_t>(height_);
      image.width = static_cast<uint32_t>(width_);
      image.encoding = "32FC1";
      image.is_bigendian = false;
      image.step = static_cast<uint32_t>(width_ * sizeof(float));
      image.data.resize(bytes);
      std::memcpy(image.data.data(), distance_.data(), bytes);
      meta.map_load_time = stamp_;
      meta.resolution = static_cast<float>(resolution_);
      meta.width = static_cast<uint32_t>(width_);
      meta.height = static_cast<uint32_t>(height_);
      meta.origin = origin;
      meta.origin.position.z = 0.0;
      meta.origin.orientation.w = 1.0;
    }
    image_pub_->publish(image);
    meta_pub_->publish(meta);
    if (publish_points_ && points_pub_) {
      const double now_sec = this->now().seconds();
      if (points_period_ms_ == 0 || now_sec - last_points_sec_ >= points_period_ms_ / 1000.0) {
        last_points_sec_ = now_sec;
        publishPoints(image.header);
      }
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "esdf %dx%d res=%.3f obstacles=%zu free=%zu time=%.1f ms",
                         static_cast<int>(meta.width), static_cast<int>(meta.height),
                         static_cast<double>(meta.resolution), obstacles, free_cells, elapsed_ms);
  }

  void publishPoints(const std_msgs::msg::Header& header) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      int stride = point_stride_;
      while (stride < width_ && stride < height_ &&
             (width_ / stride + 1) * (height_ / stride + 1) > points_max_) {
        ++stride;
      }
      for (int y = 0; y < height_; y += stride) {
        for (int x = 0; x < width_; x += stride) {
          const size_t p = static_cast<size_t>(y) * static_cast<size_t>(width_) +
                           static_cast<size_t>(x);
          if (distance_[p] >= static_cast<float>(params_.max_distance)) continue;
          pcl::PointXYZI point;
          point.x = static_cast<float>(origin_x_ + (x + 0.5) * resolution_);
          point.y = static_cast<float>(origin_y_ + (y + 0.5) * resolution_);
          point.z = 0.0f;
          point.intensity = distance_[p];
          cloud.push_back(point);
        }
      }
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header = header;
    points_pub_->publish(message);
  }

  void queryCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg) {
    std_msgs::msg::Float32 response;
    response.data = static_cast<float>(params_.max_distance);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (width_ > 0 && height_ > 0) {
        const double cell_x = (msg->point.x - origin_x_) / resolution_ - 0.5;
        const double cell_y = (msg->point.y - origin_y_) / resolution_ - 0.5;
        response.data = bilinear(distance_, width_, height_, cell_x, cell_y,
                                 static_cast<float>(params_.max_distance));
      }
    }
    distance_pub_->publish(response);
  }

  EdtParams params_;
  std::string source_topic_ = "/costmap_2d/grid";
  std::string distance_topic_ = "/esdf/local/distance";
  std::string meta_topic_ = "/esdf/local/meta";
  std::string query_topic_ = "/esdf/local/query";
  std::string query_response_topic_ = "/esdf/local/query_distance";
  std::string points_topic_ = "/esdf/local/points";
  bool publish_points_ = true;
  int point_stride_ = 2;
  int downsample_ = 1;
  int points_period_ms_ = 500;
  int points_max_ = 60000;
  double last_points_sec_ = 0.0;

  std::mutex mutex_;
  std::vector<float> distance_;
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.05;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  std::string frame_id_ = "map";
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr source_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr query_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr meta_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
};

}
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<esdf::EsdfNode>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
