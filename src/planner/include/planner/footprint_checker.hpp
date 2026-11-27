#pragma once

#include "planner/grid.hpp"
#include "robot_model/footprint.hpp"

#include <vector>

namespace planner {

class FootprintChecker {
 public:
  void configure(const robot_model::Footprint& footprint, double min_clearance, double sample_step);

  bool freeAt(const EsdfField& field, double x, double y, double yaw) const;

  bool freeCircle(const EsdfField& field, double x, double y) const;

  bool valid() const { return valid_; }

  double inscribedRadius() const { return inscribed_radius_; }

  double circumscribedRadius() const { return circumscribed_radius_; }

  const std::vector<robot_model::Point2>& bodySamples() const { return body_samples_; }

 private:
  bool valid_ = false;
  double inscribed_radius_ = 0.0;
  double circumscribed_radius_ = 0.0;
  double min_clearance_ = 0.0;
  robot_model::Footprint footprint_;
  std::vector<robot_model::Point2> body_samples_;
};

}
