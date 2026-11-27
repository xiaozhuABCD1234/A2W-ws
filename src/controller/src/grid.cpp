#include "controller/grid.hpp"

#include <algorithm>
#include <cmath>

#include "esdf/edt.hpp"

namespace controller {

void DistanceField::update(const std::vector<float>& distance, int width, int height,
                           double resolution, double origin_x, double origin_y,
                           double max_distance) {
  if (width <= 0 || height <= 0 || distance.size() != static_cast<std::size_t>(width) *
                                                     static_cast<std::size_t>(height)) {
    reset();
    return;
  }
  distance_ = distance;
  width_ = width;
  height_ = height;
  resolution_ = resolution > 0.0 ? resolution : 0.05;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  max_distance_ = max_distance > 0.0 ? max_distance : 3.0;
  esdf::gradient(distance_, width_, height_, resolution_, max_distance_, &gradient_x_,
                 &gradient_y_);
}

void DistanceField::reset() {
  distance_.clear();
  gradient_x_.clear();
  gradient_y_.clear();
  width_ = 0;
  height_ = 0;
}

bool DistanceField::empty() const { return width_ <= 0 || height_ <= 0 || distance_.empty(); }

int DistanceField::width() const { return width_; }

int DistanceField::height() const { return height_; }

double DistanceField::resolution() const { return resolution_; }

double DistanceField::maxDistance() const { return max_distance_; }

double DistanceField::originX() const { return origin_x_; }

double DistanceField::originY() const { return origin_y_; }

bool DistanceField::inside(double x, double y) const {
  if (empty()) return false;
  const double gx = (x - origin_x_) / resolution_;
  const double gy = (y - origin_y_) / resolution_;
  return gx >= 0.0 && gy >= 0.0 && gx <= static_cast<double>(width_ - 1) &&
         gy <= static_cast<double>(height_ - 1);
}

double DistanceField::distance(double x, double y) const {
  if (empty()) return max_distance_;
  if (!inside(x, y)) return max_distance_;
  const double gx = (x - origin_x_) / resolution_;
  const double gy = (y - origin_y_) / resolution_;
  return esdf::bilinear(distance_, width_, height_, gx, gy, static_cast<float>(max_distance_));
}

bool DistanceField::gradient(double x, double y, double* gradient_x,
                             double* gradient_y) const {
  if (gradient_x == nullptr || gradient_y == nullptr) return false;
  if (empty() || !inside(x, y)) {
    *gradient_x = 0.0;
    *gradient_y = 0.0;
    return false;
  }
  const double gx = (x - origin_x_) / resolution_;
  const double gy = (y - origin_y_) / resolution_;
  *gradient_x = esdf::bilinear(gradient_x_, width_, height_, gx, gy, 0.0f);
  *gradient_y = esdf::bilinear(gradient_y_, width_, height_, gx, gy, 0.0f);
  return true;
}

bool DistanceField::clear(double x, double y, double clearance) const {
  return distance(x, y) >= clearance;
}

}
