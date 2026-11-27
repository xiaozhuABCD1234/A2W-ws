#pragma once

#include <vector>

namespace controller {

class DistanceField {
 public:
  void update(const std::vector<float>& distance, int width, int height, double resolution,
              double origin_x, double origin_y, double max_distance);

  void reset();

  bool empty() const;

  int width() const;

  int height() const;

  double resolution() const;

  double maxDistance() const;

  double originX() const;

  double originY() const;

  bool inside(double x, double y) const;

  double distance(double x, double y) const;

  bool gradient(double x, double y, double* gradient_x, double* gradient_y) const;

  bool clear(double x, double y, double clearance) const;

 private:
  std::vector<float> distance_;
  std::vector<float> gradient_x_;
  std::vector<float> gradient_y_;
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.05;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double max_distance_ = 3.0;
};

}
