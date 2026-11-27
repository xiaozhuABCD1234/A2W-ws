#include "planner/footprint_checker.hpp"

#include <algorithm>
#include <cmath>

namespace planner {

void FootprintChecker::configure(const robot_model::Footprint& footprint, double min_clearance,
                                 double sample_step) {
  footprint_ = footprint;
  valid_ = footprint.valid;
  min_clearance_ = std::max(0.0, min_clearance);
  inscribed_radius_ = footprint.inscribed_radius + min_clearance_;
  circumscribed_radius_ = footprint.circumscribed_radius + min_clearance_;

  body_samples_.clear();
  if (!valid_) return;
  const double spacing = sample_step > 0.0 ? sample_step : 0.05;
  body_samples_ = robot_model::sampleFootprint(footprint, 0.0, 0.0, 0.0, spacing);
}

bool FootprintChecker::freeCircle(const EsdfField& field, double x, double y) const {
  if (!valid_) return false;
  return static_cast<double>(field.at(x, y)) >= inscribed_radius_;
}

bool FootprintChecker::freeAt(const EsdfField& field, double x, double y, double yaw) const {
  if (!valid_) return false;
  const double center = static_cast<double>(field.at(x, y));
  if (center <= inscribed_radius_) return false;
  if (center > circumscribed_radius_) return true;

  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  for (const robot_model::Point2& sample : body_samples_) {
    const double sx = x + cosine * sample[0] - sine * sample[1];
    const double sy = y + sine * sample[0] + cosine * sample[1];
    if (static_cast<double>(field.at(sx, sy)) <= min_clearance_) return false;
  }
  return true;
}

}
