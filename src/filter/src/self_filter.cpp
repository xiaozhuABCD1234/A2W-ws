#include "filter/self_filter.hpp"

#include <algorithm>

namespace filter {

SelfFilter::SelfFilter(const SelfFilterParams& params) : params_(params) {
  params_.margin = std::max(0.0, params_.margin);
  params_.min_range = std::max(0.0, params_.min_range);
  const float margin = static_cast<float>(params_.margin);
  min_x_ = static_cast<float>(std::min(params_.min_x, params_.max_x)) - margin;
  max_x_ = static_cast<float>(std::max(params_.min_x, params_.max_x)) + margin;
  min_y_ = static_cast<float>(std::min(params_.min_y, params_.max_y)) - margin;
  max_y_ = static_cast<float>(std::max(params_.min_y, params_.max_y)) + margin;
  min_z_ = static_cast<float>(std::min(params_.min_z, params_.max_z)) - margin;
  max_z_ = static_cast<float>(std::max(params_.min_z, params_.max_z)) + margin;
  min_range2_ = static_cast<float>(params_.min_range * params_.min_range);
}

bool SelfFilter::isSelf(const Eigen::Vector3f& point) const {
  if (!params_.enabled) return false;
  if (min_range2_ > 0.0f && point.squaredNorm() < min_range2_) return true;
  if (point.x() < min_x_ || point.x() > max_x_) return false;
  if (point.y() < min_y_ || point.y() > max_y_) return false;
  if (point.z() < min_z_ || point.z() > max_z_) return false;
  return true;
}

SelfFilterStats SelfFilter::filter(const std::vector<Eigen::Vector3f>& points,
                                   std::vector<uint8_t>* self_mask) const {
  SelfFilterStats stats;
  stats.input_points = points.size();
  self_mask->assign(points.size(), 0);
  if (!params_.enabled) return stats;
  for (size_t index = 0; index < points.size(); ++index) {
    if (!isSelf(points[index])) continue;
    (*self_mask)[index] = 1;
    ++stats.self_points;
  }
  return stats;
}

}
