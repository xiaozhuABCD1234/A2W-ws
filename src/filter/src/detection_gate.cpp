#include "filter/detection_gate.hpp"

#include <algorithm>
#include <cmath>

namespace filter {

DetectionGate::DetectionGate(const DetectionGateParams& params) : params_(params) {
  params_.expand = std::max(0.0, params_.expand);
  params_.timeout = std::max(0.0, params_.timeout);
  params_.min_size = std::max(0.0, params_.min_size);
  expand_ = static_cast<float>(params_.expand);
  min_z_ = static_cast<float>(params_.min_height);
  max_z_ = static_cast<float>(params_.max_height);
  min_half_ = static_cast<float>(params_.min_size * 0.5);
}

void DetectionGate::reset() {
  boxes_.clear();
  stamp_ = -1.0;
}

void DetectionGate::set(const std::vector<DetectionBox>& boxes, double stamp) {
  if (!params_.enabled) return;
  boxes_.clear();
  boxes_.reserve(boxes.size());
  for (const DetectionBox& box : boxes) {
    if (box.half_size.x() < min_half_ || box.half_size.y() < min_half_ ||
        box.half_size.z() < min_half_) {
      continue;
    }
    if (box.center.z() + box.half_size.z() < min_z_ || box.center.z() - box.half_size.z() > max_z_) {
      continue;
    }
    DetectionBox filtered = box;
    filtered.rotation.normalize();
    boxes_.push_back(filtered);
  }
  stamp_ = stamp;
}

bool DetectionGate::inside(const DetectionBox& box, const Eigen::Vector3f& point) const {
  const Eigen::Vector3f local = box.rotation.conjugate() * (point - box.center);
  const Eigen::Vector3f limit = box.half_size + Eigen::Vector3f::Constant(expand_);
  return std::abs(local.x()) <= limit.x() && std::abs(local.y()) <= limit.y() &&
         std::abs(local.z()) <= limit.z();
}

const DetectionBox* DetectionGate::match(const Eigen::Vector3f& point) const {
  for (const DetectionBox& box : boxes_) {
    if (inside(box, point)) return &box;
  }
  return nullptr;
}

size_t DetectionGate::filter(const std::vector<Eigen::Vector3f>& points, double stamp,
                             std::vector<uint8_t>* mask) const {
  mask->assign(points.size(), 0);
  if (!params_.enabled || boxes_.empty()) return 0;
  if (stamp_ < 0.0 || stamp - stamp_ > params_.timeout) return 0;
  size_t count = 0;
  for (size_t index = 0; index < points.size(); ++index) {
    if (match(points[index]) == nullptr) continue;
    (*mask)[index] = 1;
    ++count;
  }
  return count;
}

}
