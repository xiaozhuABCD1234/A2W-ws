#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace robot_model {

using Point2 = std::array<double, 2>;

struct FootprintParams {
  std::vector<Point2> points;
  double padding = 0.0;
};

struct Footprint {
  std::vector<Point2> points;
  double inscribed_radius = 0.0;
  double circumscribed_radius = 0.0;
  bool valid = false;
};

Footprint makeFootprint(const FootprintParams& params);

std::vector<Point2> transformPolygon(const std::vector<Point2>& points, double x, double y,
                                    double yaw);

std::vector<Point2> sampleFootprint(const Footprint& footprint, double x, double y, double yaw,
                                   double step);

bool pointInPolygon(const std::vector<Point2>& polygon, double x, double y);

double polygonArea(const std::vector<Point2>& polygon);

}
