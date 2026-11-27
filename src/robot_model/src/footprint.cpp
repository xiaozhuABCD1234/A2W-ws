#include "robot_model/footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_model {
namespace {

double norm(double x, double y) { return std::hypot(x, y); }

double segmentDistance(double ax, double ay, double bx, double by) {
  const double dx = bx - ax;
  const double dy = by - ay;
  const double length2 = dx * dx + dy * dy;
  if (length2 <= 1e-12) return norm(ax, ay);
  double t = -(ax * dx + ay * dy) / length2;
  t = std::min(1.0, std::max(0.0, t));
  return norm(ax + t * dx, ay + t * dy);
}

std::vector<Point2> offsetPolygon(const std::vector<Point2>& points, double padding) {
  const size_t count = points.size();
  std::vector<Point2> lines_a(count);
  std::vector<Point2> lines_b(count);
  for (size_t i = 0; i < count; ++i) {
    const Point2& from = points[i];
    const Point2& to = points[(i + 1) % count];
    const double dx = to[0] - from[0];
    const double dy = to[1] - from[1];
    const double length = norm(dx, dy);
    if (length <= 1e-9) {
      lines_a[i] = from;
      lines_b[i] = to;
      continue;
    }
    const double nx = dy / length;
    const double ny = -dx / length;
    lines_a[i] = {from[0] + padding * nx, from[1] + padding * ny};
    lines_b[i] = {to[0] + padding * nx, to[1] + padding * ny};
  }

  std::vector<Point2> result(count);
  for (size_t i = 0; i < count; ++i) {
    const Point2& p1 = lines_a[(i + count - 1) % count];
    const Point2& p2 = lines_b[(i + count - 1) % count];
    const Point2& p3 = lines_a[i];
    const Point2& p4 = lines_b[i];
    const double denominator =
        (p1[0] - p2[0]) * (p3[1] - p4[1]) - (p1[1] - p2[1]) * (p3[0] - p4[0]);
    if (std::abs(denominator) < 1e-9) {
      result[i] = p3;
      continue;
    }
    const double a = p1[0] * p2[1] - p1[1] * p2[0];
    const double b = p3[0] * p4[1] - p3[1] * p4[0];
    result[i] = {(a * (p3[0] - p4[0]) - (p1[0] - p2[0]) * b) / denominator,
                 (a * (p3[1] - p4[1]) - (p1[1] - p2[1]) * b) / denominator};
  }
  return result;
}

}  // namespace

double polygonArea(const std::vector<Point2>& polygon) {
  if (polygon.size() < 3) return 0.0;
  double sum = 0.0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const Point2& a = polygon[i];
    const Point2& b = polygon[(i + 1) % polygon.size()];
    sum += a[0] * b[1] - b[0] * a[1];
  }
  return 0.5 * sum;
}

Footprint makeFootprint(const FootprintParams& params) {
  Footprint footprint;
  if (params.points.size() < 3) return footprint;
  for (const Point2& point : params.points) {
    if (!std::isfinite(point[0]) || !std::isfinite(point[1])) return footprint;
  }

  std::vector<Point2> points = params.points;
  if (polygonArea(points) < 0.0) std::reverse(points.begin(), points.end());
  if (params.padding > 0.0 && std::isfinite(params.padding)) {
    points = offsetPolygon(points, params.padding);
  }
  for (const Point2& point : points) {
    if (!std::isfinite(point[0]) || !std::isfinite(point[1])) return footprint;
  }

  footprint.points = points;
  footprint.circumscribed_radius = 0.0;
  footprint.inscribed_radius = std::numeric_limits<double>::max();
  bool has_segment = false;
  for (size_t i = 0; i < points.size(); ++i) {
    const Point2& a = points[i];
    const Point2& b = points[(i + 1) % points.size()];
    footprint.circumscribed_radius = std::max(footprint.circumscribed_radius, norm(a[0], a[1]));
    footprint.circumscribed_radius = std::max(footprint.circumscribed_radius, norm(b[0], b[1]));
    if (norm(b[0] - a[0], b[1] - a[1]) <= 1e-9) continue;
    footprint.inscribed_radius =
        std::min(footprint.inscribed_radius, segmentDistance(a[0], a[1], b[0], b[1]));
    has_segment = true;
  }
  if (!has_segment) footprint.inscribed_radius = 0.0;
  footprint.inscribed_radius = std::max(0.0, footprint.inscribed_radius);
  footprint.valid = has_segment && polygonArea(points) > 1e-6 &&
                    std::isfinite(footprint.inscribed_radius) &&
                    std::isfinite(footprint.circumscribed_radius);
  return footprint;
}

std::vector<Point2> transformPolygon(const std::vector<Point2>& points, double x, double y,
                                     double yaw) {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  std::vector<Point2> result;
  result.reserve(points.size());
  for (const Point2& point : points) {
    result.push_back({x + cosine * point[0] - sine * point[1],
                      y + sine * point[0] + cosine * point[1]});
  }
  return result;
}

bool pointInPolygon(const std::vector<Point2>& polygon, double x, double y) {
  if (polygon.size() < 3) return false;
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const double yi = polygon[i][1];
    const double yj = polygon[j][1];
    const bool crosses = (yi > y) != (yj > y);
    if (!crosses) continue;
    const double xi = polygon[i][0];
    const double xj = polygon[j][0];
    const double x_cross = xi + (y - yi) * (xj - xi) / (yj - yi);
    if (x < x_cross) inside = !inside;
  }
  return inside;
}

std::vector<Point2> sampleFootprint(const Footprint& footprint, double x, double y, double yaw,
                                    double step) {
  std::vector<Point2> samples;
  if (!footprint.valid) return samples;
  const double spacing = std::max(0.01, step);
  const std::vector<Point2> world = transformPolygon(footprint.points, x, y, yaw);
  samples = world;

  for (size_t i = 0; i < world.size(); ++i) {
    const Point2& a = world[i];
    const Point2& b = world[(i + 1) % world.size()];
    const double length = norm(b[0] - a[0], b[1] - a[1]);
    const int count = static_cast<int>(std::ceil(length / spacing));
    for (int k = 1; k < count; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(count);
      samples.push_back({a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])});
    }
  }

  double min_x = footprint.points[0][0];
  double max_x = footprint.points[0][0];
  double min_y = footprint.points[0][1];
  double max_y = footprint.points[0][1];
  for (const Point2& point : footprint.points) {
    min_x = std::min(min_x, point[0]);
    max_x = std::max(max_x, point[0]);
    min_y = std::min(min_y, point[1]);
    max_y = std::max(max_y, point[1]);
  }

  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  for (double bx = min_x; bx <= max_x + 1e-9; bx += spacing) {
    for (double by = min_y; by <= max_y + 1e-9; by += spacing) {
      if (norm(bx, by) <= footprint.inscribed_radius) continue;
      if (!pointInPolygon(footprint.points, bx, by)) continue;
      samples.push_back({x + cosine * bx - sine * by, y + sine * bx + cosine * by});
    }
  }
  return samples;
}

}
