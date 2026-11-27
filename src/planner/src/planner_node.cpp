#include "planner/footprint_checker.hpp"
#include "planner/grid.hpp"
#include "planner/search.hpp"
#include "robot_model/msg/robot_profile.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace planner {

class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode() : rclcpp::Node("planner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    loadParameters();

    rclcpp::QoS latched(1);
    latched.transient_local().reliable();

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    cost_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        cost_topic_, latched,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { costCallback(msg); });
    esdf_sub_ = create_subscription<sensor_msgs::msg::Image>(
        esdf_topic_, latched,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { esdfCallback(msg); });
    meta_sub_ = create_subscription<nav_msgs::msg::MapMetaData>(
        esdf_meta_topic_, latched,
        [this](nav_msgs::msg::MapMetaData::ConstSharedPtr msg) { metaCallback(msg); });
    profile_sub_ = create_subscription<robot_model::msg::RobotProfile>(
        profile_topic_, latched,
        [this](robot_model::msg::RobotProfile::ConstSharedPtr msg) { profileCallback(msg); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { goalCallback(msg); });

    RCLCPP_INFO(get_logger(), "planner: cost=%s esdf=%s goal=%s path=%s world=%s robot=%s",
                cost_topic_.c_str(), esdf_topic_.c_str(), goal_topic_.c_str(),
                path_topic_.c_str(), world_frame_.c_str(), robot_frame_.c_str());
  }

 private:
  template <typename T>
  T param(const std::string& name, const T& default_value) {
    return declare_parameter<T>(name, default_value);
  }

  void loadParameters() {
    cost_topic_ = param<std::string>("topics.cost", cost_topic_);
    esdf_topic_ = param<std::string>("topics.esdf", esdf_topic_);
    esdf_meta_topic_ = param<std::string>("topics.esdf_meta", esdf_meta_topic_);
    profile_topic_ = param<std::string>("topics.robot_profile", profile_topic_);
    goal_topic_ = param<std::string>("topics.goal", goal_topic_);
    path_topic_ = param<std::string>("topics.path", path_topic_);
    world_frame_ = param<std::string>("frames.world", world_frame_);
    robot_frame_ = param<std::string>("frames.robot", robot_frame_);
    tf_timeout_ms_ = param<int>("frames.tf_timeout_ms", tf_timeout_ms_);

    options_.angle_search = param<bool>("search.angle_search", options_.angle_search);
    options_.cost_weight = param<double>("search.cost_weight", options_.cost_weight);
    options_.max_expansions = param<int>("search.max_expansions", options_.max_expansions);
    options_.sample_step = param<double>("search.sample_step", options_.sample_step);
    options_.time_budget_ms = param<double>("search.time_budget_ms", options_.time_budget_ms);
    options_.region_margin = param<double>("search.region_margin", options_.region_margin);
    goal_tolerance_ = param<double>("search.goal_tolerance", goal_tolerance_);
    min_clearance_ = param<double>("collision.min_clearance", min_clearance_);
    sample_step_ = param<double>("collision.sample_step", sample_step_);
    allow_unknown_ = param<bool>("collision.allow_unknown", allow_unknown_);
    min_goal_distance_ = param<double>("collision.min_goal_distance", min_goal_distance_);
  }

  void profileCallback(robot_model::msg::RobotProfile::ConstSharedPtr msg) {
    robot_model::FootprintParams params;
    for (const auto& point : msg->footprint.points) params.points.push_back({point.x, point.y});
    const robot_model::Footprint footprint = robot_model::makeFootprint(params);
    checker_.configure(footprint, min_clearance_, sample_step_);
    const bool was_ready = profile_ready_;
    profile_ready_ = checker_.valid();
    if (!was_ready) {
      RCLCPP_INFO(get_logger(), "planner: footprint %zu points inscribed=%.3f circumscribed=%.3f",
                  footprint.points.size(), checker_.inscribedRadius(),
                  checker_.circumscribedRadius());
    }
  }

  void costCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
    const auto width = static_cast<int64_t>(msg->info.width);
    const auto height = static_cast<int64_t>(msg->info.height);
    if (width <= 0 || height <= 0 || !(msg->info.resolution > 0.0f)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "planner: ignoring invalid costmap %ldx%ld res=%.3f",
                           static_cast<long>(width), static_cast<long>(height),
                           static_cast<double>(msg->info.resolution));
      return;
    }
    const size_t total = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (msg->data.size() < total) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "planner: truncated costmap data (%zu < %zu), keeping previous grid",
                           msg->data.size(), total);
      return;
    }

    grid_.width = static_cast<int>(width);
    grid_.height = static_cast<int>(height);
    grid_.resolution = msg->info.resolution;
    grid_.origin_x = msg->info.origin.position.x;
    grid_.origin_y = msg->info.origin.position.y;
    grid_.cost.assign(total, 1.0f);
    grid_.blocked.assign(total, 0);
    for (size_t i = 0; i < total; ++i) {
      const int8_t value = msg->data[i];
      if (value >= 65 || (value < 0 && !allow_unknown_)) {
        grid_.blocked[i] = 1;
      } else if (value > 0) {
        grid_.cost[i] = 1.0f + options_.cost_weight * static_cast<float>(value) / 100.0f;
      }
    }
  }

  void metaCallback(nav_msgs::msg::MapMetaData::ConstSharedPtr msg) {
    if (msg->width == 0 || msg->height == 0 || !(msg->resolution > 0.0f)) return;
    field_.width = static_cast<int>(msg->width);
    field_.height = static_cast<int>(msg->height);
    field_.resolution = msg->resolution;
    field_.origin_x = msg->origin.position.x;
    field_.origin_y = msg->origin.position.y;
  }

  void esdfCallback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    if (msg->encoding != "32FC1" || msg->width == 0 || msg->height == 0) return;
    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    const size_t total = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (msg->data.size() < total * sizeof(float)) return;
    field_.width = width;
    field_.height = height;
    field_.data.resize(total);
    std::memcpy(field_.data.data(), msg->data.data(), total * sizeof(float));
    field_.max_distance = 0.0f;
    for (const float value : field_.data) {
      field_.max_distance = std::max(field_.max_distance, value);
    }
    field_ready_ = true;
  }

  void goalCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    if (!profile_ready_) {
      RCLCPP_WARN(get_logger(), "planner: robot profile not received yet");
      return;
    }
    if (!field_ready_) {
      RCLCPP_WARN(get_logger(), "planner: esdf not received yet");
      return;
    }
    if (grid_.empty()) {
      RCLCPP_WARN(get_logger(), "planner: cost grid empty");
      return;
    }

    if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y)) {
      RCLCPP_WARN(get_logger(), "planner: ignoring non-finite goal");
      return;
    }

    geometry_msgs::msg::TransformStamped robot;
    try {
      const auto timeout = tf2::durationFromSec(
          tf_timeout_ms_ > 0 ? static_cast<double>(tf_timeout_ms_) / 1000.0 : 0.0);
      robot = tf_buffer_.lookupTransform(world_frame_, robot_frame_, tf2::TimePointZero,
                                         timeout);
    } catch (const std::exception& exception) {
      RCLCPP_WARN(get_logger(), "planner: %s->%s failed: %s", world_frame_.c_str(),
                  robot_frame_.c_str(), exception.what());
      return;
    }

    Point2d start;
    start.x = robot.transform.translation.x;
    start.y = robot.transform.translation.y;
    tf2::Quaternion quaternion(robot.transform.rotation.x, robot.transform.rotation.y,
                               robot.transform.rotation.z, robot.transform.rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    Point2d goal;
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;
    const double goal_distance = std::hypot(goal.x - start.x, goal.y - start.y);
    if (goal_distance < min_goal_distance_) {
      RCLCPP_WARN(get_logger(), "planner: goal too close (%.2f m)", goal_distance);
      return;
    }

    const auto begin = std::chrono::steady_clock::now();
    applyFootprintInflation();

    if (!snapToFree(&start) || !snapToFree(&goal)) {
      RCLCPP_WARN(
          get_logger(),
          "planner: start (%.2f %.2f) or goal (%.2f %.2f) inside inflated obstacle",
          start.x, start.y, goal.x, goal.y);
      return;
    }
    const FreeFn is_free = [this](double x, double y, double heading) {
      return checker_.freeAt(field_, x, y, heading);
    };

    SearchResult result = plan(grid_, start, goal, options_, is_free);
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();

    if (!result.success) {
      RCLCPP_WARN(get_logger(), "planner: no path (expanded=%zu, %.1f ms, %s)", result.expanded,
                  elapsed_ms, result.timed_out ? "budget" : "exhausted");
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = world_frame_;
    path.poses.resize(result.path.size());
    for (size_t i = 0; i < result.path.size(); ++i) {
      path.poses[i].header = path.header;
      path.poses[i].pose.position.x = result.path[i].x;
      path.poses[i].pose.position.y = result.path[i].y;
      path.poses[i].pose.orientation.w = 1.0;
    }
    path_pub_->publish(path);

    RCLCPP_INFO(
        get_logger(),
        "planner: %zu waypoints expanded=%zu length=%.2f m start=(%.2f %.2f %.1f deg) "
        "%.1f ms",
        result.path.size(), result.expanded, result.cost, start.x, start.y,
        yaw * 180.0 / M_PI, elapsed_ms);
  }

  bool snapToFree(Point2d* point) const {
    int cx = 0;
    int cy = 0;
    if (!grid_.worldToCell(point->x, point->y, &cx, &cy)) return false;
    if (!grid_.blockedAt(cx, cy)) {
      grid_.cellToWorld(cx, cy, &point->x, &point->y);
      return true;
    }
    const int radius = std::max(1, static_cast<int>(std::ceil(goal_tolerance_ / grid_.resolution)));
    for (int r = 1; r <= radius; ++r) {
      double best = 1e9;
      int best_x = -1;
      int best_y = -1;
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) continue;
          const int nx = cx + dx;
          const int ny = cy + dy;
          if (grid_.blockedAt(nx, ny)) continue;
          const double distance = std::hypot(dx, dy);
          if (distance < best) {
            best = distance;
            best_x = nx;
            best_y = ny;
          }
        }
      }
      if (best_x >= 0) {
        grid_.cellToWorld(best_x, best_y, &point->x, &point->y);
        return true;
      }
    }
    return false;
  }

  void applyFootprintInflation() {
    if (!checker_.valid() || field_.empty()) return;
    const size_t total = grid_.blocked.size();
    for (size_t i = 0; i < total; ++i) {
      if (grid_.blocked[i] != 0) continue;
      const int cx = static_cast<int>(i % static_cast<size_t>(grid_.width));
      const int cy = static_cast<int>(i / static_cast<size_t>(grid_.width));
      double x = 0.0;
      double y = 0.0;
      grid_.cellToWorld(cx, cy, &x, &y);
      if (!checker_.freeCircle(field_, x, y)) grid_.blocked[i] = 1;
    }
  }

  std::string cost_topic_ = "/occupancy_map_2d/map";
  std::string esdf_topic_ = "/esdf/global/distance";
  std::string esdf_meta_topic_ = "/esdf/global/meta";
  std::string profile_topic_ = "/robot_profile";
  std::string goal_topic_ = "/goal_pose";
  std::string path_topic_ = "/planner/path";
  std::string world_frame_ = "map";
  std::string robot_frame_ = "base_footprint";
  int tf_timeout_ms_ = 100;
  double min_clearance_ = 0.05;
  double sample_step_ = 0.05;
  bool allow_unknown_ = true;
  double min_goal_distance_ = 0.20;
  double goal_tolerance_ = 0.50;

  GridMap grid_;
  EsdfField field_;
  FootprintChecker checker_;
  SearchOptions options_;
  bool profile_ready_ = false;
  bool field_ready_ = false;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr esdf_sub_;
  rclcpp::Subscription<nav_msgs::msg::MapMetaData>::SharedPtr meta_sub_;
  rclcpp::Subscription<robot_model::msg::RobotProfile>::SharedPtr profile_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planner::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
