#include <gz/msgs/pointcloud_packed.pb.h>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Sphere.hh>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace sim_mid360 {
namespace {

struct Ray {
  double azimuth = 0.0;
  double zenith = 0.0;
};

enum class ShapeKind { kBox, kCylinder, kSphere, kPlane };

struct Occluder {
  ShapeKind kind = ShapeKind::kBox;
  gz::math::Vector3d size{0.0, 0.0, 0.0};
  double radius = 0.0;
  double length = 0.0;
  gz::math::Pose3d pose;
};

double intersectBox(const gz::math::Vector3d& origin, const gz::math::Vector3d& direction,
                    const gz::math::Vector3d& half) {
  double t_min = -std::numeric_limits<double>::infinity();
  double t_max = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 3; ++axis) {
    const double o = origin[axis];
    const double d = direction[axis];
    const double h = half[axis];
    if (std::abs(d) < 1e-12) {
      if (o < -h || o > h) return -1.0;
      continue;
    }
    double t1 = (-h - o) / d;
    double t2 = (h - o) / d;
    if (t1 > t2) std::swap(t1, t2);
    t_min = std::max(t_min, t1);
    t_max = std::min(t_max, t2);
    if (t_min > t_max) return -1.0;
  }
  if (t_max < 0.0) return -1.0;
  return t_min > 0.0 ? t_min : t_max;
}

double intersectCylinder(const gz::math::Vector3d& origin, const gz::math::Vector3d& direction,
                         double radius, double half_length) {
  const double a = direction.X() * direction.X() + direction.Y() * direction.Y();
  const double b = 2.0 * (origin.X() * direction.X() + origin.Y() * direction.Y());
  const double c = origin.X() * origin.X() + origin.Y() * origin.Y() - radius * radius;
  double best = -1.0;
  if (a > 1e-12) {
    const double disc = b * b - 4.0 * a * c;
    if (disc >= 0.0) {
      const double root = std::sqrt(disc);
      for (const double t : {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)}) {
        if (t <= 0.0) continue;
        const double z = origin.Z() + t * direction.Z();
        if (std::abs(z) <= half_length) {
          best = (best < 0.0) ? t : std::min(best, t);
        }
      }
    }
  }
  if (std::abs(direction.Z()) > 1e-12) {
    for (const double cap : {half_length, -half_length}) {
      const double t = (cap - origin.Z()) / direction.Z();
      if (t <= 0.0) continue;
      const double x = origin.X() + t * direction.X();
      const double y = origin.Y() + t * direction.Y();
      if (x * x + y * y <= radius * radius) best = (best < 0.0) ? t : std::min(best, t);
    }
  }
  return best;
}

double intersectSphere(const gz::math::Vector3d& origin, const gz::math::Vector3d& direction,
                       double radius) {
  const double a = direction.SquaredLength();
  const double b = 2.0 * origin.Dot(direction);
  const double c = origin.SquaredLength() - radius * radius;
  const double disc = b * b - 4.0 * a * c;
  if (disc < 0.0 || a < 1e-12) return -1.0;
  const double root = std::sqrt(disc);
  double t = (-b - root) / (2.0 * a);
  if (t <= 0.0) t = (-b + root) / (2.0 * a);
  return t > 0.0 ? t : -1.0;
}

double intersectPlane(const gz::math::Vector3d& origin, const gz::math::Vector3d& direction,
                      const gz::math::Vector3d& normal) {
  const double denominator = normal.Dot(direction);
  if (std::abs(denominator) < 1e-12) return -1.0;
  const double t = -normal.Dot(origin) / denominator;
  return t > 0.0 ? t : -1.0;
}

}

class Mid360Pattern : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager&, gz::sim::EventManager&) override {
    model_ = gz::sim::Model(entity);
    link_name_ = sdf->Get<std::string>("parent_link", "lidar_link").first;
    csv_path_ = sdf->Get<std::string>("csv_file", "").first;
    topic_ = sdf->Get<std::string>("topic", "/mid360/points").first;
    frame_id_ = sdf->Get<std::string>("frame_id", link_name_).first;
    samples_per_second_ = std::max(1, sdf->Get<int>("samples_per_second", 100000).first);
    update_rate_ = std::max(1e-3, sdf->Get<double>("update_rate", 10.0).first);
    min_range_ = std::max(0.0, sdf->Get<double>("min_range", 0.1).first);
    max_range_ = std::max(min_range_, sdf->Get<double>("max_range", 40.0).first);
    if (csv_path_.empty()) {
      gzerr << "[mid360] csv_file is empty, no pattern loaded\n";
      return;
    }
    const double mx = sdf->Get<double>("mount_x", 0.0).first;
    const double my = sdf->Get<double>("mount_y", 0.0).first;
    const double mz = sdf->Get<double>("mount_z", 0.0).first;
    const double mroll = sdf->Get<double>("mount_roll", 0.0).first;
    const double mpitch = sdf->Get<double>("mount_pitch", 0.0).first;
    const double myaw = sdf->Get<double>("mount_yaw", 0.0).first;
    mount_pose_ = gz::math::Pose3d(mx, my, mz, mroll, mpitch, myaw);
    if (!loadPattern()) return;

    publisher_ = node_.Advertise<gz::msgs::PointCloudPacked>(topic_);
    gzmsg << "[mid360] advertising on " << topic_ << "\n";

    gzmsg << "[mid360] pattern rays=" << pattern_.size() << " samples/s=" << samples_per_second_
          << " topic=" << topic_ << " frame=" << frame_id_ << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override {
    if (pattern_.empty() || info.paused) return;
    const double period = 1.0 / std::max(1.0, update_rate_);
    const double elapsed = std::chrono::duration<double>(info.simTime).count();
    if (elapsed - last_scan_ < period - 1e-6) return;
    last_scan_ = elapsed;

    gz::sim::Entity link = model_.LinkByName(ecm, link_name_);
    if (link == gz::sim::kNullEntity) {
      gzerr << "[mid360] link not found: " << link_name_ << "\n";
      return;
    }
    if (occluders_.empty()) {
      collectOccluders(ecm, link);
      gzmsg << "[mid360] occluders=" << occluders_.size() << "\n";
    }
    if (occluders_.empty()) return;

    const gz::math::Pose3d link_pose = gz::sim::worldPose(link, ecm);
    const gz::math::Pose3d sensor_pose = link_pose * mount_pose_;

    const size_t per_scan = std::max<size_t>(
        1, static_cast<size_t>(samples_per_second_ * period));
    const size_t start = cursor_ % pattern_.size();

    std::vector<float> xs;
    std::vector<float> ys;
    std::vector<float> zs;
    std::vector<float> ts;
    xs.reserve(per_scan);
    ys.reserve(per_scan);
    zs.reserve(per_scan);
    ts.reserve(per_scan);

    for (size_t i = 0; i < per_scan; ++i) {
      const Ray& ray = pattern_[(start + i) % pattern_.size()];
      const gz::math::Quaterniond rotation(0.0, -ray.zenith, ray.azimuth);
      const gz::math::Vector3d local = rotation * gz::math::Vector3d::UnitX;
      const gz::math::Vector3d direction = sensor_pose.Rot() * local;
      const double range = cast(sensor_pose.Pos(), direction);
      if (range < min_range_ || range > max_range_) continue;
      const gz::math::Vector3d point = sensor_pose.Pos() + direction * range;
      const gz::math::Vector3d local_point =
          sensor_pose.Rot().RotateVectorReverse(point - sensor_pose.Pos());
      xs.push_back(static_cast<float>(local_point.X()));
      ys.push_back(static_cast<float>(local_point.Y()));
      zs.push_back(static_cast<float>(local_point.Z()));
      ts.push_back(static_cast<float>(i) * static_cast<float>(period) /
                   static_cast<float>(per_scan));
    }
    cursor_ += per_scan;
    if (xs.empty()) return;

    const double scan_start = elapsed - period;
    gz::msgs::PointCloudPacked cloud;
    auto* header = cloud.mutable_header();
    header->mutable_stamp()->set_sec(static_cast<int64_t>(scan_start));
    header->mutable_stamp()->set_nsec(
        static_cast<int64_t>((scan_start - std::floor(scan_start)) * 1e9));
    cloud.set_height(1);
    cloud.set_width(static_cast<uint32_t>(xs.size()));
    cloud.set_is_dense(false);
    cloud.set_point_step(20);
    cloud.set_row_step(20 * cloud.width());

    const char* names[5] = {"x", "y", "z", "intensity", "time"};
    const uint32_t offsets[5] = {0, 4, 8, 12, 16};
    const auto datatype = gz::msgs::PointCloudPacked_Field_DataType_FLOAT32;
    for (int i = 0; i < 5; ++i) {
      auto* field = cloud.add_field();
      field->set_name(names[i]);
      field->set_offset(offsets[i]);
      field->set_datatype(datatype);
      field->set_count(1);
    }

    std::string buffer(xs.size() * 20, '\0');
    for (size_t i = 0; i < xs.size(); ++i) {
      const double squared = xs[i] * xs[i] + ys[i] * ys[i] + zs[i] * zs[i];
      const float intensity = 200.0f - std::min(200.0f, static_cast<float>(squared));
      char* slot = &buffer[i * 20];
      std::memcpy(slot + 0, &xs[i], 4);
      std::memcpy(slot + 4, &ys[i], 4);
      std::memcpy(slot + 8, &zs[i], 4);
      std::memcpy(slot + 12, &intensity, 4);
      std::memcpy(slot + 16, &ts[i], 4);
    }
    cloud.set_data(buffer);
    publisher_.Publish(cloud);
  }

 private:
  bool loadPattern() {
    std::ifstream stream(csv_path_);
    if (!stream) {
      gzerr << "[mid360] cannot open pattern csv: " << csv_path_ << "\n";
      return false;
    }
    pattern_.clear();
    std::string line;
    std::getline(stream, line);
    while (std::getline(stream, line)) {
      if (line.empty()) continue;
      std::stringstream row(line);
      std::string cell;
      std::vector<double> values;
      while (std::getline(row, cell, ',')) {
        try {
          values.push_back(std::stod(cell));
        } catch (const std::exception&) {
          values.clear();
          break;
        }
      }
      if (values.size() < 3 || !std::isfinite(values[1]) || !std::isfinite(values[2])) {
        continue;
      }
      Ray ray;
      ray.azimuth = values[1] * M_PI / 180.0;
      ray.zenith = (90.0 - values[2]) * M_PI / 180.0;
      pattern_.push_back(ray);
    }
    return !pattern_.empty();
  }

  void collectOccluders(gz::sim::EntityComponentManager& ecm, gz::sim::Entity self_link) {
    occluders_.clear();
    ecm.Each<gz::sim::components::Geometry>(
        [&](const gz::sim::Entity& entity, const gz::sim::components::Geometry* geometry) -> bool {
          if (ecm.ParentEntity(entity) == self_link) return true;
          const auto& shape = geometry->Data();
          Occluder occluder;
          if (shape.Type() == sdf::GeometryType::BOX) {
            occluder.kind = ShapeKind::kBox;
            occluder.size = gz::math::Vector3d(shape.BoxShape()->Size().X() * 0.5,
                                               shape.BoxShape()->Size().Y() * 0.5,
                                               shape.BoxShape()->Size().Z() * 0.5);
          } else if (shape.Type() == sdf::GeometryType::CYLINDER) {
            occluder.kind = ShapeKind::kCylinder;
            occluder.radius = shape.CylinderShape()->Radius();
            occluder.length = shape.CylinderShape()->Length() * 0.5;
          } else if (shape.Type() == sdf::GeometryType::SPHERE) {
            occluder.kind = ShapeKind::kSphere;
            occluder.radius = shape.SphereShape()->Radius();
          } else if (shape.Type() == sdf::GeometryType::PLANE) {
            occluder.kind = ShapeKind::kPlane;
          } else {
            return true;
          }
          occluder.pose = gz::sim::worldPose(entity, ecm);
          occluders_.push_back(occluder);
          return true;
        });
  }

  double cast(const gz::math::Vector3d& origin, const gz::math::Vector3d& direction) const {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& occluder : occluders_) {
      const gz::math::Vector3d local_origin =
          occluder.pose.Rot().RotateVectorReverse(origin - occluder.pose.Pos());
      const gz::math::Vector3d local_direction =
          occluder.pose.Rot().RotateVectorReverse(direction);
      double t = -1.0;
      switch (occluder.kind) {
        case ShapeKind::kBox:
          t = intersectBox(local_origin, local_direction, occluder.size);
          break;
        case ShapeKind::kCylinder:
          t = intersectCylinder(local_origin, local_direction, occluder.radius, occluder.length);
          break;
        case ShapeKind::kSphere:
          t = intersectSphere(local_origin, local_direction, occluder.radius);
          break;
        case ShapeKind::kPlane:
          t = intersectPlane(local_origin, local_direction, gz::math::Vector3d::UnitZ);
          break;
      }
      if (t > 0.0 && t < best) best = t;
    }
    return best;
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::string link_name_ = "lidar_link";
  std::string csv_path_;
  std::string topic_ = "/mid360/points";
  std::string frame_id_ = "lidar_link";
  int samples_per_second_ = 100000;
  double update_rate_ = 10.0;
  double min_range_ = 0.1;
  double max_range_ = 40.0;
  gz::math::Pose3d mount_pose_{0, 0, 0, 0, 0, 0};
  std::vector<Ray> pattern_;
  std::vector<Occluder> occluders_;
  size_t cursor_ = 0;
  double last_scan_ = -1.0;
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
};

}

GZ_ADD_PLUGIN(sim_mid360::Mid360Pattern, gz::sim::System,
              sim_mid360::Mid360Pattern::ISystemConfigure,
              sim_mid360::Mid360Pattern::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(sim_mid360::Mid360Pattern, "sim_mid360::Mid360Pattern")
