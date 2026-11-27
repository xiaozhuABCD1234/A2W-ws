#ifndef FILTER__SELF_FILTER_HPP_
#define FILTER__SELF_FILTER_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace filter {

struct SelfFilterParams {
  bool enabled = true;
  double min_x = -0.45;
  double max_x = 0.45;
  double min_y = -0.45;
  double max_y = 0.45;
  double min_z = -1.20;
  double max_z = 0.10;
  double margin = 0.0;
  double min_range = 0.0;
};

struct SelfFilterStats {
  size_t input_points = 0;
  size_t self_points = 0;
};

class SelfFilter {
 public:
  explicit SelfFilter(const SelfFilterParams& params);

  void reset() {}

  bool isSelf(const Eigen::Vector3f& point) const;

  SelfFilterStats filter(const std::vector<Eigen::Vector3f>& points,
                         std::vector<uint8_t>* self_mask) const;

  const SelfFilterParams& params() const { return params_; }

 private:
  SelfFilterParams params_;
  float min_x_ = 0.0f;
  float max_x_ = 0.0f;
  float min_y_ = 0.0f;
  float max_y_ = 0.0f;
  float min_z_ = 0.0f;
  float max_z_ = 0.0f;
  float min_range2_ = 0.0f;
};

}

#endif
