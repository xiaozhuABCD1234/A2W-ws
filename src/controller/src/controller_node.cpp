#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_model/limits.hpp"
#include "robot_model/msg/robot_profile.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "controller/mppi.hpp"
#include "controller/path.hpp"
#include "controller/pure_pursuit.hpp"
#include "controller/smoother.hpp"

namespace controller {

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode() : Node("controller") {
    world_frame_ = declare_parameter<std::string>("frames.world", "map");
    robot_frame_ = declare_parameter<std::string>("frames.robot", "base_footprint");
    tf_timeout_ms_ = declare_parameter<int>("frames.tf_timeout_ms", 100);
    image_topic_ = declare_parameter<std::string>("topics.esdf", "/esdf/local/distance");
    meta_topic_ = declare_parameter<std::string>("topics.esdf_meta", "/esdf/local/meta");
    path_topic_ = declare_parameter<std::string>("topics.path", "/planner/path");
    profile_topic_ = declare_parameter<std::string>("topics.robot_profile", "/robot_profile");
    cmd_topic_ = declare_parameter<std::string>("topics.cmd_vel", "/cmd_vel");
    smoothed_topic_ =
        declare_parameter<std::string>("topics.smoothed_path", "/controller/smoothed_path");
    trajectory_topic_ =
        declare_parameter<std::string>("topics.trajectory", "/controller/trajectory");

    period_ms_ = declare_parameter<int>("runtime.period_ms", 50);
    field_max_distance_ = declare_parameter<double>("runtime.field_max_distance", 3.0);
    publish_markers_ = declare_parameter<bool>("runtime.publish_markers", true);
    goal_tolerance_ = declare_parameter<double>("controller.goal_tolerance", 0.20);
    stuck_timeout_ = declare_parameter<double>("controller.stuck_timeout", 3.0);

    smoother_params_.iterations = declare_parameter<int>("smoother.iterations", 600);
    smoother_params_.tolerance = declare_parameter<double>("smoother.tolerance", 5.0e-4);
    smoother_params_.w_data = declare_parameter<double>("smoother.w_data", 0.25);
    smoother_params_.w_smooth = declare_parameter<double>("smoother.w_smooth", 0.40);
    smoother_params_.w_obstacle = declare_parameter<double>("smoother.w_obstacle", 0.60);
    smoother_params_.alpha = declare_parameter<double>("smoother.alpha", 0.60);
    smoother_params_.max_step = declare_parameter<double>("smoother.max_step", 0.05);
    smoother_params_.min_clearance = declare_parameter<double>("smoother.min_clearance", 0.22);
    smoother_params_.time_budget_ms = declare_parameter<double>("smoother.time_budget_ms", 100.0);
    smoother_params_.refinement = declare_parameter<int>("smoother.refinement", 1);
    smoother_enabled_ = declare_parameter<bool>("smoother.enabled", true);
    resample_step_ = declare_parameter<double>("smoother.resample_step", 0.05);

    mppi_params_.batch = declare_parameter<int>("mppi.batch", 400);
    mppi_params_.time_steps = declare_parameter<int>("mppi.time_steps", 40);
    mppi_params_.model_dt = declare_parameter<double>("mppi.model_dt", 0.05);
    mppi_params_.iterations = declare_parameter<int>("mppi.iterations", 1);
    mppi_params_.temperature = declare_parameter<double>("mppi.temperature", 0.30);
    mppi_params_.gamma = declare_parameter<double>("mppi.gamma", 0.015);
    mppi_params_.sigma_linear = declare_parameter<double>("mppi.sigma_linear", 0.20);
    mppi_params_.sigma_angular = declare_parameter<double>("mppi.sigma_angular", 0.35);
    mppi_params_.shift_steps = declare_parameter<int>("mppi.shift_steps", 1);
    mppi_params_.w_path = declare_parameter<double>("mppi.w_path", 1.0);
    mppi_params_.w_goal = declare_parameter<double>("mppi.w_goal", 1.6);
    mppi_params_.w_obstacle = declare_parameter<double>("mppi.w_obstacle", 0.9);
    mppi_params_.w_heading = declare_parameter<double>("mppi.w_heading", 0.35);
    mppi_params_.w_prefer_forward = declare_parameter<double>("mppi.w_prefer_forward", 0.05);
    mppi_params_.w_constraint = declare_parameter<double>("mppi.w_constraint", 3.0);
    mppi_params_.collision_penalty = declare_parameter<double>("mppi.collision_penalty", 1.0e5);
    mppi_params_.safe_distance = declare_parameter<double>("mppi.safe_distance", 0.40);
    mppi_params_.critical_clearance = declare_parameter<double>("mppi.critical_clearance", 0.25);
    mppi_params_.near_goal_distance = declare_parameter<double>("mppi.near_goal_distance", 0.60);
    mppi_params_.path_step = declare_parameter<int>("mppi.path_step", 3);
    mppi_params_.path_window = declare_parameter<int>("mppi.path_window", 40);
    mppi_params_.max_linear = declare_parameter<double>("mppi.max_linear", 0.80);
    mppi_params_.min_linear = declare_parameter<double>("mppi.min_linear", 0.0);
    mppi_params_.max_angular = declare_parameter<double>("mppi.max_angular", 1.20);
    mppi_enabled_ = declare_parameter<bool>("mppi.enabled", true);

    pure_pursuit_params_.desired_linear =
        declare_parameter<double>("pure_pursuit.desired_linear", 0.60);
    pure_pursuit_params_.lookahead_time =
        declare_parameter<double>("pure_pursuit.lookahead_time", 1.10);
    pure_pursuit_params_.min_lookahead =
        declare_parameter<double>("pure_pursuit.min_lookahead", 0.35);
    pure_pursuit_params_.max_lookahead =
        declare_parameter<double>("pure_pursuit.max_lookahead", 1.10);
    pure_pursuit_params_.min_radius = declare_parameter<double>("pure_pursuit.min_radius", 0.35);
    pure_pursuit_params_.approach_distance =
        declare_parameter<double>("pure_pursuit.approach_distance", 0.70);
    pure_pursuit_params_.min_approach_speed =
        declare_parameter<double>("pure_pursuit.min_approach_speed", 0.06);
    pure_pursuit_params_.rotate_angle =
        declare_parameter<double>("pure_pursuit.rotate_angle", 1.20);
    pure_pursuit_params_.rotate_angular =
        declare_parameter<double>("pure_pursuit.rotate_angular", 0.90);
    pure_pursuit_params_.goal_tolerance = goal_tolerance_;
    pure_pursuit_enabled_ = declare_parameter<bool>("pure_pursuit.enabled", true);

    limits_.max_linear_velocity = declare_parameter<double>("limits.max_linear_velocity", 0.80);
    limits_.max_angular_velocity = declare_parameter<double>("limits.max_angular_velocity", 1.20);
    limits_.max_linear_acceleration =
        declare_parameter<double>("limits.max_linear_acceleration", 0.80);
    limits_.max_angular_acceleration =
        declare_parameter<double>("limits.max_angular_acceleration", 2.00);

    mppi_.configure(mppi_params_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    smoothed_pub_ = create_publisher<nav_msgs::msg::Path>(smoothed_topic_, 1);
    trajectory_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(trajectory_topic_, 1);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, 1,
        [this](const nav_msgs::msg::Path::SharedPtr msg) { pathCallback(msg); });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, 1,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { imageCallback(msg); });
    meta_sub_ = create_subscription<nav_msgs::msg::MapMetaData>(
        meta_topic_, rclcpp::QoS(1).transient_local(),
        [this](const nav_msgs::msg::MapMetaData::SharedPtr msg) { metaCallback(msg); });
    profile_sub_ = create_subscription<robot_model::msg::RobotProfile>(
        profile_topic_, 1,
        [this](const robot_model::msg::RobotProfile::SharedPtr msg) { profileCallback(msg); });

    timer_ = create_wall_timer(std::chrono::milliseconds(std::max(5, period_ms_)),
                               [this]() { timerCallback(); });
  }

 private:
  void metaCallback(const nav_msgs::msg::MapMetaData::SharedPtr msg) {
    metas_.push_back(*msg);
    while (metas_.size() > 8) metas_.pop_front();
  }

  const nav_msgs::msg::MapMetaData* matchMeta(const builtin_interfaces::msg::Time& stamp) {
    for (auto it = metas_.rbegin(); it != metas_.rend(); ++it) {
      if (it->map_load_time.sec == stamp.sec && it->map_load_time.nanosec == stamp.nanosec) {
        return &(*it);
      }
    }
    if (metas_.empty()) return nullptr;
    return &metas_.back();
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->encoding != "32FC1" || msg->width == 0 || msg->height == 0) return;
    const std::size_t cells = static_cast<std::size_t>(msg->width) *
                              static_cast<std::size_t>(msg->height);
    if (msg->data.size() < cells * sizeof(float)) return;
    const nav_msgs::msg::MapMetaData* meta = matchMeta(msg->header.stamp);
    if (meta == nullptr) return;
    std::vector<float> distance(cells);
    std::memcpy(distance.data(), msg->data.data(), cells * sizeof(float));
    field_.update(distance, static_cast<int>(msg->width), static_cast<int>(msg->height),
                  static_cast<double>(meta->resolution), meta->origin.position.x,
                  meta->origin.position.y, field_max_distance_);
  }

  void profileCallback(const robot_model::msg::RobotProfile::SharedPtr msg) {
    if (msg->max_linear_velocity > 0.0) {
      limits_.max_linear_velocity = msg->max_linear_velocity;
      mppi_params_.max_linear = msg->max_linear_velocity;
      pure_pursuit_params_.desired_linear =
          std::min(mppi_params_.max_linear, 0.75 * msg->max_linear_velocity);
    }
    if (msg->max_angular_velocity > 0.0) {
      limits_.max_angular_velocity = msg->max_angular_velocity;
      mppi_params_.max_angular = msg->max_angular_velocity;
    }
    if (msg->max_linear_acceleration > 0.0) {
      limits_.max_linear_acceleration = msg->max_linear_acceleration;
    }
    if (msg->max_angular_acceleration > 0.0) {
      limits_.max_angular_acceleration = msg->max_angular_acceleration;
    }
    mppi_.configure(mppi_params_);
    if (has_path_) {
      mppi_.setPath(path_);
    }
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    std::vector<Point> raw;
    raw.reserve(msg->poses.size());
    for (const auto& pose : msg->poses) {
      Point point;
      point.x = pose.pose.position.x;
      point.y = pose.pose.position.y;
      raw.push_back(point);
    }
    if (raw.size() < 2) {
      has_path_ = false;
      return;
    }

    std::vector<Point> resampled;
    resamplePath(raw, resample_step_, &resampled);
    if (resampled.size() < 2) {
      has_path_ = false;
      return;
    }

    std::vector<Point> smoothed = resampled;
    SmootherStats stats;
    if (smoother_enabled_) {
      smoothPath(resampled, field_, smoother_params_, &smoothed, &stats);
    }
    path_ = makePath(smoothed);
    mppi_.setPath(path_);
    mppi_.reset();
    has_path_ = true;
    finished_ = false;
    stuck_time_ = 0.0;

    publishSmoothedPath(msg->header.frame_id.empty() ? world_frame_ : msg->header.frame_id);
    RCLCPP_INFO(get_logger(),
                "path %zu -> %zu points, deviation %.3f m, clearance %.3f m, %d iterations%s",
                raw.size(), path_.points.size(), stats.max_deviation, stats.min_clearance,
                stats.iterations, stats.used_field ? "" : " (no esdf)");
  }

  void publishSmoothedPath(const std::string& frame) {
    nav_msgs::msg::Path message;
    message.header.stamp = now();
    message.header.frame_id = frame;
    message.poses.reserve(path_.points.size());
    for (std::size_t i = 0; i < path_.points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = path_.points[i].x;
      pose.pose.position.y = path_.points[i].y;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, path_.yaw[i]);
      pose.pose.orientation.x = orientation.x();
      pose.pose.orientation.y = orientation.y();
      pose.pose.orientation.z = orientation.z();
      pose.pose.orientation.w = orientation.w();
      message.poses.push_back(pose);
    }
    smoothed_pub_->publish(message);
  }

  bool lookupPose(Pose2D* pose) {
    try {
      const geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
          world_frame_, robot_frame_, tf2::TimePointZero,
          tf2::durationFromSec(static_cast<double>(tf_timeout_ms_) / 1000.0));
      pose->x = transform.transform.translation.x;
      pose->y = transform.transform.translation.y;
      const auto& rotation = transform.transform.rotation;
      pose->yaw = std::atan2(2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                             1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
      tf_failures_ = 0;
      return true;
    } catch (const std::exception&) {
      ++tf_failures_;
      return false;
    }
  }

  void publishCommand(const Command& command) {
    geometry_msgs::msg::Twist message;
    message.linear.x = command.linear;
    message.angular.z = command.angular;
    cmd_pub_->publish(message);
  }

  void publishMarkers(const MppiOutput& output, const Point& carrot) {
    if (!publish_markers_) return;
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    if (output.best_trajectory.size() >= 2) {
      visualization_msgs::msg::Marker line;
      line.header.stamp = now();
      line.header.frame_id = world_frame_;
      line.ns = "controller";
      line.id = 1;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.03;
      line.color.r = 0.20f;
      line.color.g = 0.95f;
      line.color.b = 0.35f;
      line.color.a = 0.95f;
      line.pose.orientation.w = 1.0;
      line.points.reserve(output.best_trajectory.size());
      for (const Point& point : output.best_trajectory) {
        geometry_msgs::msg::Point marker_point;
        marker_point.x = point.x;
        marker_point.y = point.y;
        line.points.push_back(marker_point);
      }
      array.markers.push_back(line);
    }

    visualization_msgs::msg::Marker carrot_marker;
    carrot_marker.header.stamp = now();
    carrot_marker.header.frame_id = world_frame_;
    carrot_marker.ns = "controller";
    carrot_marker.id = 2;
    carrot_marker.type = visualization_msgs::msg::Marker::SPHERE;
    carrot_marker.action = visualization_msgs::msg::Marker::ADD;
    carrot_marker.scale.x = 0.12;
    carrot_marker.scale.y = 0.12;
    carrot_marker.scale.z = 0.12;
    carrot_marker.color.r = 0.98f;
    carrot_marker.color.g = 0.85f;
    carrot_marker.color.b = 0.15f;
    carrot_marker.color.a = 0.95f;
    carrot_marker.pose.position.x = carrot.x;
    carrot_marker.pose.position.y = carrot.y;
    carrot_marker.pose.orientation.w = 1.0;
    array.markers.push_back(carrot_marker);

    trajectory_pub_->publish(array);
  }

  double obstacleScale(const Pose2D& pose) const {
    if (field_.empty()) return 1.0;
    const double clearance = field_.distance(pose.x, pose.y);
    const double span =
        std::max(1e-3, mppi_params_.safe_distance - mppi_params_.critical_clearance);
    return std::clamp((clearance - mppi_params_.critical_clearance) / span, 0.0, 1.0);
  }

  void timerCallback() {
    if (!has_path_) {
      publishCommand(Command{});
      current_ = Command{};
      return;
    }

    Pose2D pose;
    if (!lookupPose(&pose)) {
      if (tf_failures_ > 2) {
        publishCommand(Command{});
        current_ = Command{};
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "no transform %s -> %s, skipping control cycle", world_frame_.c_str(),
                           robot_frame_.c_str());
      return;
    }

    const Point goal = path_.points.back();
    const double goal_distance = std::hypot(pose.x - goal.x, pose.y - goal.y);

    if (finished_ || goal_distance <= goal_tolerance_) {
      finished_ = true;
      publishCommand(Command{});
      current_ = Command{};
      MppiOutput empty;
      publishMarkers(empty, goal);
      return;
    }

    const double scale = obstacleScale(pose);
    Command desired;
    MppiOutput mppi_output;
    bool mppi_used = false;
    Point carrot = goal;

    if (mppi_enabled_) {
      bool idle = true;
      for (double value : mppi_.controlLinear()) {
        if (std::fabs(value) > 1e-6) {
          idle = false;
          break;
        }
      }
      if (idle) {
        const PurePursuitOutput seed =
            purePursuit(path_, pose, std::max(current_.linear, 0.2), scale, pure_pursuit_params_);
        if (!seed.reached) mppi_.warmStart(seed.command);
      }
      mppi_output = mppi_.optimize(pose, field_, goal);
      if (mppi_output.feasible) {
        desired = mppi_output.command;
        mppi_used = true;
      }
    }

    if (!mppi_used && pure_pursuit_enabled_) {
      const PurePursuitOutput pursuit =
          purePursuit(path_, pose, std::max(current_.linear, 0.2), scale, pure_pursuit_params_);
      desired = pursuit.command;
      carrot = pursuit.carrot;
      if (pursuit.reached) {
        finished_ = true;
      }
    }

    const double dt = static_cast<double>(std::max(5, period_ms_)) / 1000.0;
    const robot_model::Twist2D limited = robot_model::applyLimits(
        robot_model::Twist2D{current_.linear, current_.angular},
        robot_model::Twist2D{desired.linear, desired.angular}, dt, limits_);
    const Command applied{limited.linear, limited.angular};

    if (std::fabs(applied.linear) < 1e-3 && std::fabs(applied.angular) < 1e-3) {
      stuck_time_ += dt;
    } else {
      stuck_time_ = 0.0;
    }

    publishCommand(applied);
    publishMarkers(mppi_output, carrot);
    current_ = applied;

    const double clearance =
        mppi_used ? mppi_output.min_clearance
                  : (field_.empty() ? 0.0 : field_.distance(pose.x, pose.y));
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "cmd v=%.2f w=%.2f %s clearance=%.2f goal=%.2f scale=%.2f",
                         applied.linear, applied.angular, mppi_used ? "mppi" : "pursuit",
                         clearance, goal_distance, scale);

    if (stuck_time_ > stuck_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "controller stuck for %.1f s",
                           stuck_time_);
    }
  }

  std::string world_frame_;
  std::string robot_frame_;
  int tf_timeout_ms_ = 100;
  std::string image_topic_;
  std::string meta_topic_;
  std::string path_topic_;
  std::string profile_topic_;
  std::string cmd_topic_;
  std::string smoothed_topic_;
  std::string trajectory_topic_;
  int period_ms_ = 50;
  double field_max_distance_ = 3.0;
  bool publish_markers_ = true;
  double goal_tolerance_ = 0.20;
  double stuck_timeout_ = 3.0;
  bool smoother_enabled_ = true;
  double resample_step_ = 0.05;
  bool mppi_enabled_ = true;
  bool pure_pursuit_enabled_ = true;

  SmootherParams smoother_params_;
  MppiParams mppi_params_;
  PurePursuitParams pure_pursuit_params_;
  robot_model::VelocityLimits limits_;

  Mppi mppi_;
  DistanceField field_;
  Path2D path_;
  bool has_path_ = false;
  bool finished_ = false;
  Command current_;
  double stuck_time_ = 0.0;
  int tf_failures_ = 0;
  std::deque<nav_msgs::msg::MapMetaData> metas_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::MapMetaData>::SharedPtr meta_sub_;
  rclcpp::Subscription<robot_model::msg::RobotProfile>::SharedPtr profile_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
