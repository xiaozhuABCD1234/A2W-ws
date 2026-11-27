#ifndef FILTER__DETECTION_GATE_HPP_
#define FILTER__DETECTION_GATE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <vector>

namespace filter {

struct DetectionBox {
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
  Eigen::Vector3f half_size = Eigen::Vector3f::Zero();
};

struct DetectionGateParams {
  bool enabled = true;
  double expand = 0.20;
  double timeout = 0.30;
  double min_height = -0.5;
  double max_height = 2.5;
  double min_size = 0.05;
};

class DetectionGate {
 public:
  explicit DetectionGate(const DetectionGateParams& params);

  void reset();

  void set(const std::vector<DetectionBox>& boxes, double stamp);

  size_t filter(const std::vector<Eigen::Vector3f>& points, double stamp,
                std::vector<uint8_t>* mask) const;

  size_t boxes() const { return boxes_.size(); }

  bool inside(const DetectionBox& box, const Eigen::Vector3f& point) const;

  const DetectionGateParams& params() const { return params_; }

 private:
  const DetectionBox* match(const Eigen::Vector3f& point) const;

  DetectionGateParams params_;
  std::vector<DetectionBox> boxes_;
  double stamp_ = -1.0;
  float expand_ = 0.2f;
  float min_z_ = 0.0f;
  float max_z_ = 0.0f;
  float min_half_ = 0.025f;
};

}

#endif
