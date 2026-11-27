#include "robot_model/footprint.hpp"
#include "robot_model/limits.hpp"
#include "robot_model/msg/robot_profile.hpp"

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace robot_model {

class RobotModelNode : public rclcpp::Node {
 public:
  RobotModelNode() : rclcpp::Node("robot_model") {
    loadParameters();

    rclcpp::QoS latched(1);
    latched.transient_local().reliable();
    description_pub_ = create_publisher<std_msgs::msg::String>("robot_description", latched);
    profile_pub_ = create_publisher<msg::RobotProfile>("robot_profile", latched);
    footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("robot_footprint", 10);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    if (publish_description_) {
      std_msgs::msg::String description_msg;
      description_msg.data = loadDescription();
      description_pub_->publish(description_msg);
    }

    broadcastStaticTransform();
    broadcastLidarTransform();
    publishProfile();

    publish_period_ms_ = std::max(50, publish_period_ms_);
    timer_ = create_wall_timer(std::chrono::milliseconds(publish_period_ms_), [this]() {
      publishProfile();
    });

    RCLCPP_INFO(get_logger(),
                "robot model: %zu footprint points inscribed=%.3f circumscribed=%.3f base=%s",
                footprint_.points.size(), footprint_.inscribed_radius,
                footprint_.circumscribed_radius, base_frame_.c_str());
  }

 private:
  template <typename T>
  T param(const std::string& name, const T& default_value) {
    return declare_parameter<T>(name, default_value);
  }

  void loadParameters() {
    odom_frame_ = param<std::string>("frames.odom", odom_frame_);
    base_frame_ = param<std::string>("frames.base", base_frame_);
    offset_x_ = param<double>("frames.offset_x", offset_x_);
    offset_y_ = param<double>("frames.offset_y", offset_y_);
    offset_z_ = param<double>("frames.offset_z", offset_z_);
    offset_yaw_ = param<double>("frames.offset_yaw", offset_yaw_);

    const std::vector<double> flat = param<std::vector<double>>("footprint.points", {});
    if (flat.size() % 2 != 0) {
      RCLCPP_WARN(get_logger(), "robot model: footprint.points has an odd number of values");
    }
    FootprintParams footprint_params;
    for (size_t i = 0; i + 1 < flat.size(); i += 2) {
      footprint_params.points.push_back({flat[i], flat[i + 1]});
    }
    footprint_params.padding = param<double>("footprint.padding", footprint_params.padding);
    height_ = std::max(0.01, param<double>("footprint.height", height_));
    footprint_ = makeFootprint(footprint_params);
    if (!footprint_.valid) {
      RCLCPP_WARN(get_logger(),
                  "robot model: footprint is invalid (%zu points); planner will reject goals",
                  footprint_params.points.size());
    }

    limits_.max_linear_velocity =
        param<double>("limits.max_linear_velocity", limits_.max_linear_velocity);
    limits_.max_linear_acceleration =
        param<double>("limits.max_linear_acceleration", limits_.max_linear_acceleration);
    limits_.max_angular_velocity =
        param<double>("limits.max_angular_velocity", limits_.max_angular_velocity);
    limits_.max_angular_acceleration =
        param<double>("limits.max_angular_acceleration", limits_.max_angular_acceleration);
    limits_.max_linear_velocity = std::max(0.0, limits_.max_linear_velocity);
    limits_.max_angular_velocity = std::max(0.0, limits_.max_angular_velocity);
    limits_.max_linear_acceleration = std::max(0.0, limits_.max_linear_acceleration);
    limits_.max_angular_acceleration = std::max(0.0, limits_.max_angular_acceleration);
    publish_period_ms_ = param<int>("publish_period_ms", publish_period_ms_);
    description_file_ = param<std::string>("description.file", description_file_);
    lidar_frame_ = param<std::string>("frames.lidar", lidar_frame_);
    lidar_x_ = param<double>("frames.lidar_x", lidar_x_);
    lidar_y_ = param<double>("frames.lidar_y", lidar_y_);
    lidar_z_ = param<double>("frames.lidar_z", lidar_z_);
    lidar_roll_ = param<double>("frames.lidar_roll", lidar_roll_);
    lidar_pitch_ = param<double>("frames.lidar_pitch", lidar_pitch_);
    lidar_yaw_ = param<double>("frames.lidar_yaw", lidar_yaw_);
    publish_description_ = param<bool>("description.publish", publish_description_);
  }

  bool publish_description_ = true;
  std::string description_file_;

  std::string loadDescription() const {
    if (!description_file_.empty()) {
      std::ifstream stream(description_file_);
      if (stream) {
        std::ostringstream buffer;
        buffer << stream.rdbuf();
        return buffer.str();
      }
      RCLCPP_WARN(get_logger(), "robot model: cannot read %s, falling back to generated box",
                  description_file_.c_str());
    }
    return buildDescription();
  }

  std::string buildDescription() const {
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    if (!footprint_.points.empty()) {
      min_x = footprint_.points.front()[0];
      max_x = min_x;
      min_y = footprint_.points.front()[1];
      max_y = min_y;
      for (const Point2& point : footprint_.points) {
        min_x = std::min(min_x, point[0]);
        max_x = std::max(max_x, point[0]);
        min_y = std::min(min_y, point[1]);
        max_y = std::max(max_y, point[1]);
      }
    }
    const double length = std::max(0.05, max_x - min_x);
    const double width = std::max(0.05, max_y - min_y);
    const double center_x = 0.5 * (min_x + max_x);
    const double center_y = 0.5 * (min_y + max_y);

    std::ostringstream xml;
    xml << "<?xml version=\"1.0\"?>\n<robot name=\"base\">\n";
    xml << "  <link name=\"" << base_frame_ << "\"/>\n";
    xml << "  <link name=\"base_link\">\n";
    xml << "    <visual>\n      <origin xyz=\"" << center_x << " " << center_y << " "
        << 0.5 * height_ << "\" rpy=\"0 0 0\"/>\n";
    xml << "      <geometry><box size=\"" << length << " " << width << " " << height_
        << "\"/></geometry>\n";
    xml << "      <material name=\"chassis\"><color rgba=\"0.7 0.7 0.72 1.0\"/></material>\n";
    xml << "    </visual>\n";
    xml << "    <collision>\n      <origin xyz=\"" << center_x << " " << center_y << " "
        << 0.5 * height_ << "\" rpy=\"0 0 0\"/>\n";
    xml << "      <geometry><box size=\"" << length << " " << width << " " << height_
        << "\"/></geometry>\n";
    xml << "    </collision>\n";
    xml << "    <inertial>\n      <mass value=\"5.0\"/>\n";
    xml << "      <inertia ixx=\"0.2\" ixy=\"0\" ixz=\"0\" iyy=\"0.2\" iyz=\"0\" izz=\"0.2\"/>\n";
    xml << "    </inertial>\n";
    xml << "  </link>\n";
    xml << "  <joint name=\"base_link_joint\" type=\"fixed\">\n";
    xml << "    <parent link=\"" << base_frame_ << "\"/>\n";
    xml << "    <child link=\"base_link\"/>\n";
    xml << "  </joint>\n</robot>\n";
    return xml.str();
  }

  void broadcastStaticTransform() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = offset_x_;
    transform.transform.translation.y = offset_y_;
    transform.transform.translation.z = offset_z_;
    transform.transform.rotation.z = std::sin(0.5 * offset_yaw_);
    transform.transform.rotation.w = std::cos(0.5 * offset_yaw_);
    static_broadcaster_->sendTransform(transform);
  }

  void broadcastLidarTransform() {
    if (lidar_frame_.empty()) return;
    const double cr = std::cos(lidar_roll_ * 0.5);
    const double sr = std::sin(lidar_roll_ * 0.5);
    const double cp = std::cos(lidar_pitch_ * 0.5);
    const double sp = std::sin(lidar_pitch_ * 0.5);
    const double cy = std::cos(lidar_yaw_ * 0.5);
    const double sy = std::sin(lidar_yaw_ * 0.5);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    transform.header.frame_id = base_frame_;
    transform.child_frame_id = lidar_frame_;
    transform.transform.translation.x = lidar_x_;
    transform.transform.translation.y = lidar_y_;
    transform.transform.translation.z = lidar_z_;
    transform.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    transform.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    transform.transform.rotation.z = cr * cp * sy - sr * sp * cy;
    transform.transform.rotation.w = cr * cp * cy + sr * sp * sy;
    static_broadcaster_->sendTransform(transform);
  }

  void publishProfile() {
    msg::RobotProfile profile;
    profile.header.stamp = now();
    profile.header.frame_id = base_frame_;
    profile.footprint.points.resize(footprint_.points.size());
    for (size_t i = 0; i < footprint_.points.size(); ++i) {
      profile.footprint.points[i].x = footprint_.points[i][0];
      profile.footprint.points[i].y = footprint_.points[i][1];
      profile.footprint.points[i].z = 0.0;
    }
    profile.inscribed_radius = footprint_.inscribed_radius;
    profile.circumscribed_radius = footprint_.circumscribed_radius;
    profile.height = height_;
    profile.max_linear_velocity = limits_.max_linear_velocity;
    profile.max_linear_acceleration = limits_.max_linear_acceleration;
    profile.max_angular_velocity = limits_.max_angular_velocity;
    profile.max_angular_acceleration = limits_.max_angular_acceleration;
    profile_pub_->publish(profile);

    geometry_msgs::msg::PolygonStamped footprint_msg;
    footprint_msg.header = profile.header;
    footprint_msg.polygon = profile.footprint;
    footprint_pub_->publish(footprint_msg);
  }

  std::string odom_frame_ = "body";
  std::string base_frame_ = "base_footprint";
  std::string lidar_frame_ = "livox";
  double lidar_x_ = 0.0;
  double lidar_y_ = 0.0;
  double lidar_z_ = 0.0;
  double lidar_roll_ = 0.0;
  double lidar_pitch_ = 0.0;
  double lidar_yaw_ = 0.0;
  double offset_x_ = 0.0;
  double offset_y_ = 0.0;
  double offset_z_ = 0.0;
  double offset_yaw_ = 0.0;
  double height_ = 0.6;
  int publish_period_ms_ = 1000;
  Footprint footprint_;
  VelocityLimits limits_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
  rclcpp::Publisher<msg::RobotProfile>::SharedPtr profile_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_model::RobotModelNode>());
  rclcpp::shutdown();
  return 0;
}
