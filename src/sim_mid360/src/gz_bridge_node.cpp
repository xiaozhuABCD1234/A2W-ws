#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gz/msgs/clock.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

namespace sim_mid360 {
namespace {

constexpr size_t kOffsetX = 0;
constexpr size_t kOffsetY = 4;
constexpr size_t kOffsetZ = 8;
constexpr size_t kOffsetIntensity = 12;
constexpr size_t kPointStep = 24;
constexpr size_t kOffsetTimestamp = 16;

bool readFloat(const std::string& data, size_t offset, float* value) {
  if (value == nullptr || offset > data.size() || data.size() - offset < sizeof(float)) {
    return false;
  }
  std::memcpy(value, data.data() + offset, sizeof(float));
  return true;
}

sensor_msgs::msg::PointField makeField(const std::string& name, uint32_t offset, uint8_t datatype) {
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

}

class BridgeNode : public rclcpp::Node {
 public:
  BridgeNode() : rclcpp::Node("sim_mid360_bridge") {
    lidar_topic_ = declare_parameter<std::string>("topics.lidar", lidar_topic_);
    imu_topic_ = declare_parameter<std::string>("topics.imu", imu_topic_);
    clock_topic_ = declare_parameter<std::string>("topics.clock", clock_topic_);
    cmd_vel_topic_ = declare_parameter<std::string>("topics.cmd_vel", cmd_vel_topic_);
    gz_lidar_topic_ = declare_parameter<std::string>("gz.lidar", gz_lidar_topic_);
    gz_imu_topic_ = declare_parameter<std::string>("gz.imu", gz_imu_topic_);
    gz_clock_topic_ = declare_parameter<std::string>("gz.clock", gz_clock_topic_);
    gz_cmd_vel_topic_ = declare_parameter<std::string>("gz.cmd_vel", gz_cmd_vel_topic_);
    frame_id_ = declare_parameter<std::string>("frames.lidar", frame_id_);
    scan_period_ = declare_parameter<double>("lidar.scan_period", scan_period_);
    publish_clock_ = declare_parameter<bool>("clock.publish", publish_clock_);
    imu_median_window_ = declare_parameter<int>("imu.median_window", imu_median_window_);
    mount_roll_ = declare_parameter<double>("mount.roll", mount_roll_);
    mount_pitch_ = declare_parameter<double>("mount.pitch", mount_pitch_);
    mount_yaw_ = declare_parameter<double>("mount.yaw", mount_yaw_);
    mount_x_ = declare_parameter<double>("mount.x", mount_x_);
    mount_y_ = declare_parameter<double>("mount.y", mount_y_);
    mount_z_ = declare_parameter<double>("mount.z", mount_z_);
    buildMountRotation();
    imu_median_window_ = std::clamp(imu_median_window_, 1, 999);
    scan_period_ = std::max(1e-3, scan_period_);

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic_, 5);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 200);
    if (publish_clock_) {
      clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(clock_topic_, 50);
      gz_node_.Subscribe(gz_clock_topic_, &BridgeNode::onClock, this);
    }
    gz_node_.Subscribe(gz_lidar_topic_, &BridgeNode::onCloud, this);
    gz_node_.Subscribe(gz_imu_topic_, &BridgeNode::onImu, this);

    cmd_vel_pub_ = gz_node_.Advertise<gz::msgs::Twist>(gz_cmd_vel_topic_);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          gz::msgs::Twist twist;
          twist.mutable_linear()->set_x(msg->linear.x);
          twist.mutable_linear()->set_y(msg->linear.y);
          twist.mutable_angular()->set_z(msg->angular.z);
          cmd_vel_pub_.Publish(twist);
        });

    RCLCPP_INFO(get_logger(),
                "sim bridge: %s -> %s, %s -> %s, clock=%s", gz_lidar_topic_.c_str(),
                lidar_topic_.c_str(), gz_imu_topic_.c_str(), imu_topic_.c_str(),
                publish_clock_ ? "on" : "off");
  }

 private:
  void buildMountRotation() {
    if (!std::isfinite(mount_roll_) || !std::isfinite(mount_pitch_) ||
        !std::isfinite(mount_yaw_) || !std::isfinite(mount_x_) ||
        !std::isfinite(mount_y_) || !std::isfinite(mount_z_)) {
      RCLCPP_WARN(get_logger(), "sim bridge: non-finite lidar mount, using identity");
      mount_roll_ = mount_pitch_ = mount_yaw_ = 0.0;
      mount_x_ = mount_y_ = mount_z_ = 0.0;
    }
    const double cr = std::cos(mount_roll_);
    const double sr = std::sin(mount_roll_);
    const double cp = std::cos(mount_pitch_);
    const double sp = std::sin(mount_pitch_);
    const double cy = std::cos(mount_yaw_);
    const double sy = std::sin(mount_yaw_);
    rotation_ << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                 sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                 -sp,     cp * sr,                cp * cr;
    const bool identity = std::abs(mount_roll_) < 1e-9 && std::abs(mount_pitch_) < 1e-9 &&
                          std::abs(mount_yaw_) < 1e-9 && std::abs(mount_x_) < 1e-9 &&
                          std::abs(mount_y_) < 1e-9 && std::abs(mount_z_) < 1e-9;
    RCLCPP_INFO(get_logger(),
                "sim bridge: lidar mount rpy=(%.3f %.3f %.3f) xyz=(%.3f %.3f %.3f) %s",
                mount_roll_, mount_pitch_, mount_yaw_, mount_x_, mount_y_, mount_z_,
                identity ? "(identity)" : "(rotating points into body frame)");
  }

  void onClock(const gz::msgs::Clock& msg) {
    if (!clock_pub_) return;
    const double stamp = static_cast<double>(msg.sim().sec()) +
                         static_cast<double>(msg.sim().nsec()) * 1e-9;
    if (stamp - last_clock_ < 1e-3) return;
    last_clock_ = stamp;
    rosgraph_msgs::msg::Clock clock;
    clock.clock.sec = static_cast<int32_t>(msg.sim().sec());
    clock.clock.nanosec = msg.sim().nsec();
    clock_pub_->publish(clock);
  }

  double filter(double sample, std::deque<double>* window) const {
    if (imu_median_window_ <= 1) return sample;
    window->push_back(sample);
    while (static_cast<int>(window->size()) > imu_median_window_) window->pop_front();
    std::vector<double> scratch(window->begin(), window->end());
    const auto middle = scratch.begin() + static_cast<long>(scratch.size() / 2);
    std::nth_element(scratch.begin(), middle, scratch.end());
    return *middle;
  }

  void onImu(const gz::msgs::IMU& msg) {
    double stamp = static_cast<double>(msg.header().stamp().sec()) +
                   static_cast<double>(msg.header().stamp().nsec()) * 1e-9;
    if (stamp <= last_imu_) stamp = last_imu_ + 1e-4;
    last_imu_ = stamp;
    sensor_msgs::msg::Imu imu;
    imu.header.stamp.sec = static_cast<int32_t>(stamp);
    imu.header.stamp.nanosec = static_cast<uint32_t>((stamp - std::floor(stamp)) * 1e9);
    imu.header.frame_id = frame_id_;
    imu.orientation.x = msg.orientation().x();
    imu.orientation.y = msg.orientation().y();
    imu.orientation.z = msg.orientation().z();
    imu.orientation.w = msg.orientation().w();
    imu.angular_velocity.x = filter(msg.angular_velocity().x(), &gyro_x_);
    imu.angular_velocity.y = filter(msg.angular_velocity().y(), &gyro_y_);
    imu.angular_velocity.z = filter(msg.angular_velocity().z(), &gyro_z_);
    imu.linear_acceleration.x = filter(msg.linear_acceleration().x(), &acc_x_);
    imu.linear_acceleration.y = filter(msg.linear_acceleration().y(), &acc_y_);
    imu.linear_acceleration.z = filter(msg.linear_acceleration().z(), &acc_z_);
    imu_pub_->publish(imu);
  }

  void onCloud(const gz::msgs::PointCloudPacked& msg) {
    const size_t width = static_cast<size_t>(msg.width());
    const size_t height = static_cast<size_t>(msg.height());
    const size_t step = static_cast<size_t>(msg.point_step());
    const size_t total = width * height;
    if (total == 0) return;

    int32_t offset_x = -1;
    int32_t offset_y = -1;
    int32_t offset_z = -1;
    int32_t offset_intensity = -1;
    int32_t offset_time = -1;
    for (const auto& field : msg.field()) {
      if (field.datatype() != gz::msgs::PointCloudPacked_Field_DataType_FLOAT32) continue;
      if (field.name() == "x") offset_x = field.offset();
      else if (field.name() == "y") offset_y = field.offset();
      else if (field.name() == "z") offset_z = field.offset();
      else if (field.name() == "intensity") offset_intensity = field.offset();
      else if (field.name() == "time" || field.name() == "t" || field.name() == "offset_time")
        offset_time = field.offset();
    }
    const auto offset_valid = [step](int32_t offset) {
      return offset >= 0 && static_cast<size_t>(offset) + sizeof(float) <= step;
    };
    if (!offset_valid(offset_x) || !offset_valid(offset_y) || !offset_valid(offset_z)) return;
    if (!offset_valid(offset_intensity)) offset_intensity = -1;
    if (!offset_valid(offset_time)) offset_time = -1;

    double head = static_cast<double>(msg.header().stamp().sec()) +
                  static_cast<double>(msg.header().stamp().nsec()) * 1e-9;
    if (head <= last_cloud_) head = last_cloud_ + 1e-3;
    last_cloud_ = head;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp.sec = static_cast<int32_t>(head);
    cloud.header.stamp.nanosec = static_cast<uint32_t>((head - std::floor(head)) * 1e9);
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    cloud.point_step = kPointStep;
    cloud.fields.push_back(makeField("x", kOffsetX, sensor_msgs::msg::PointField::FLOAT32));
    cloud.fields.push_back(makeField("y", kOffsetY, sensor_msgs::msg::PointField::FLOAT32));
    cloud.fields.push_back(makeField("z", kOffsetZ, sensor_msgs::msg::PointField::FLOAT32));
    cloud.fields.push_back(
        makeField("intensity", kOffsetIntensity, sensor_msgs::msg::PointField::FLOAT32));
    cloud.fields.push_back(
        makeField("timestamp", kOffsetTimestamp, sensor_msgs::msg::PointField::FLOAT64));

    const double head_sec = head;
    const std::string& data = msg.data();
    std::vector<uint8_t> buffer;
    buffer.reserve(total * kPointStep);
    uint32_t kept = 0;

    for (size_t i = 0; i < total; ++i) {
      const size_t base = i * step;
      if (base + step > data.size()) break;
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      if (!readFloat(data, base + static_cast<size_t>(offset_x), &x) ||
          !readFloat(data, base + static_cast<size_t>(offset_y), &y) ||
          !readFloat(data, base + static_cast<size_t>(offset_z), &z)) {
        continue;
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      if (x == 0.0f && y == 0.0f && z == 0.0f) continue;
      float intensity = 0.0f;
      if (offset_intensity >= 0) {
        readFloat(data, base + static_cast<size_t>(offset_intensity), &intensity);
      }

      double stamp = 0.0;
      float relative_time = 0.0f;
      if (offset_time >= 0 &&
          readFloat(data, base + static_cast<size_t>(offset_time), &relative_time)) {
        stamp = head_sec + static_cast<double>(relative_time);
      } else {
        const float fraction = width > 1
                                   ? static_cast<float>(i % width) / static_cast<float>(width)
                                   : 0.0f;
        stamp = head_sec + static_cast<double>(fraction) * scan_period_;
      }

      Eigen::Vector3d point(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
      point = rotation_ * point;
      const float bx = static_cast<float>(point.x());
      const float by = static_cast<float>(point.y());
      const float bz = static_cast<float>(point.z());

      const size_t out = buffer.size();
      buffer.resize(out + kPointStep);
      std::memcpy(&buffer[out + kOffsetX], &bx, sizeof(float));
      std::memcpy(&buffer[out + kOffsetY], &by, sizeof(float));
      std::memcpy(&buffer[out + kOffsetZ], &bz, sizeof(float));
      std::memcpy(&buffer[out + kOffsetIntensity], &intensity, sizeof(float));
      std::memcpy(&buffer[out + kOffsetTimestamp], &stamp, sizeof(double));
      ++kept;
    }

    if (kept == 0) return;
    cloud.width = kept;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data = std::move(buffer);
    cloud_pub_->publish(cloud);
  }

  std::string lidar_topic_ = "/livox/lidar";
  std::string imu_topic_ = "/livox/imu";
  std::string clock_topic_ = "/clock";
  std::string cmd_vel_topic_ = "/cmd_vel";
  std::string gz_lidar_topic_ = "/lidar/points";
  std::string gz_imu_topic_ = "/imu";
  std::string gz_clock_topic_ = "/clock";
  std::string gz_cmd_vel_topic_ = "/cmd_vel";
  std::string frame_id_ = "livox";
  double scan_period_ = 0.1;
  bool publish_clock_ = true;
  int imu_median_window_ = 5;
  std::deque<double> acc_x_;
  std::deque<double> acc_y_;
  std::deque<double> acc_z_;
  std::deque<double> gyro_x_;
  std::deque<double> gyro_y_;
  std::deque<double> gyro_z_;
  double mount_roll_ = 0.0;
  double mount_pitch_ = 0.0;
  double mount_yaw_ = 0.0;
  double mount_x_ = 0.0;
  double mount_y_ = 0.0;
  double mount_z_ = 0.0;
  Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
  double last_cloud_ = 0.0;
  double last_imu_ = 0.0;
  double last_clock_ = 0.0;

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher cmd_vel_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sim_mid360::BridgeNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
