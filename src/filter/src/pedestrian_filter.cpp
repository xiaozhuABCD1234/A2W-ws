#include "filter/pedestrian_filter.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace filter {

PedestrianFilter::PedestrianFilter(const FilterParams& filter_params,
                                   const PedestrianParams& pedestrian_params)
    : voxels_(filter_params), params_(pedestrian_params) {
  params_.cell_size = std::max(0.05, params_.cell_size);
  params_.min_points = std::max(1, params_.min_points);
  params_.max_width = std::max(params_.cell_size, params_.max_width);
  params_.max_depth = std::max(params_.cell_size, params_.max_depth);
  params_.max_extent = std::max(params_.min_extent, params_.max_extent);
  params_.min_aspect = std::max(0.0, params_.min_aspect);
}

void PedestrianFilter::reset() {
  voxels_.reset();
  candidates_.clear();
  clusters_.clear();
  parent_.clear();
  cell_head_.clear();
  candidate_mask_.clear();
}

bool PedestrianFilter::accepts(const Cluster& cluster) const {
  const double width = cluster.max_x - cluster.min_x;
  const double depth = cluster.max_y - cluster.min_y;
  const double extent = cluster.max_z - cluster.min_z;
  if (static_cast<int>(cluster.points) < params_.min_points) return false;
  if (width > params_.max_width || depth > params_.max_depth) return false;
  if (extent < params_.min_extent || extent > params_.max_extent) return false;
  if (width * depth > params_.max_footprint) return false;
  if (extent < params_.min_aspect * std::max(width, depth)) return false;
  return true;
}

PedestrianStats PedestrianFilter::filter(const std::vector<Eigen::Vector3f>& points,
                                         const Eigen::Vector3f& origin,
                                         std::vector<uint8_t>* pedestrian_mask) {
  PedestrianStats stats;
  stats.voxel = voxels_.filter(points, origin, &candidate_mask_);
  pedestrian_mask->assign(points.size(), 0);

  candidates_.clear();
  for (size_t index = 0; index < candidate_mask_.size(); ++index) {
    if (candidate_mask_[index] != 0) candidates_.push_back(static_cast<int32_t>(index));
  }
  if (candidates_.empty()) {
    stats.voxel.dynamic_points = 0;
    return stats;
  }

  const double cell = params_.cell_size;
  double min_x = points[static_cast<size_t>(candidates_.front())].x();
  double max_x = min_x;
  double min_y = points[static_cast<size_t>(candidates_.front())].y();
  double max_y = min_y;
  for (const int32_t index : candidates_) {
    const Eigen::Vector3f& point = points[static_cast<size_t>(index)];
    min_x = std::min(min_x, static_cast<double>(point.x()));
    max_x = std::max(max_x, static_cast<double>(point.x()));
    min_y = std::min(min_y, static_cast<double>(point.y()));
    max_y = std::max(max_y, static_cast<double>(point.y()));
  }

  const int grid_w = static_cast<int>(std::floor((max_x - min_x) / cell)) + 1;
  const int grid_h = static_cast<int>(std::floor((max_y - min_y) / cell)) + 1;
  if (static_cast<double>(grid_w) * static_cast<double>(grid_h) > 400000.0) {
    if (!params_.keep_unmatched) {
      for (const int32_t index : candidates_) {
        (*pedestrian_mask)[static_cast<size_t>(index)] = 1;
      }
      stats.pedestrian_points = candidates_.size();
    }
    stats.voxel.dynamic_points = 0;
    return stats;
  }

  const size_t count = candidates_.size();
  parent_.assign(count, -1);
  cell_head_.assign(static_cast<size_t>(grid_w) * static_cast<size_t>(grid_h), -1);

  auto find_root = [this](int32_t node) {
    int32_t root = node;
    while (parent_[static_cast<size_t>(root)] >= 0) root = parent_[static_cast<size_t>(root)];
    while (node != root) {
      const int32_t next = parent_[static_cast<size_t>(node)];
      parent_[static_cast<size_t>(node)] = root;
      node = next;
    }
    return root;
  };

  auto unite = [this, &find_root](int32_t a, int32_t b) {
    int32_t root_a = find_root(a);
    int32_t root_b = find_root(b);
    if (root_a == root_b) return;
    if (parent_[static_cast<size_t>(root_a)] > parent_[static_cast<size_t>(root_b)]) {
      const int32_t swap = root_a;
      root_a = root_b;
      root_b = swap;
    }
    const int32_t size_a = -parent_[static_cast<size_t>(root_a)];
    const int32_t size_b = -parent_[static_cast<size_t>(root_b)];
    parent_[static_cast<size_t>(root_a)] = -(size_a + size_b);
    parent_[static_cast<size_t>(root_b)] = root_a;
  };

  for (size_t slot = 0; slot < count; ++slot) {
    const Eigen::Vector3f& point = points[static_cast<size_t>(candidates_[slot])];
    const int cell_x = static_cast<int>(std::floor((static_cast<double>(point.x()) - min_x) / cell));
    const int cell_y = static_cast<int>(std::floor((static_cast<double>(point.y()) - min_y) / cell));
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        const int nx = cell_x + dx;
        const int ny = cell_y + dy;
        if (nx < 0 || ny < 0 || nx >= grid_w || ny >= grid_h) continue;
        int32_t& head = cell_head_[static_cast<size_t>(ny) * static_cast<size_t>(grid_w) +
                                    static_cast<size_t>(nx)];
        if (head < 0) {
          if (dx == 0 && dy == 0) head = static_cast<int32_t>(slot);
          continue;
        }
        unite(static_cast<int32_t>(slot), head);
      }
    }
    int32_t& self = cell_head_[static_cast<size_t>(cell_y) * static_cast<size_t>(grid_w) +
                                static_cast<size_t>(cell_x)];
    if (self < 0) self = static_cast<int32_t>(slot);
  }

  clusters_.assign(count, Cluster{});
  std::vector<uint8_t> visited(count, 0);
  std::vector<int32_t> roots;
  for (size_t slot = 0; slot < count; ++slot) {
    const int32_t root = find_root(static_cast<int32_t>(slot));
    Cluster& cluster = clusters_[static_cast<size_t>(root)];
    const Eigen::Vector3f& point = points[static_cast<size_t>(candidates_[slot])];
    if (!visited[static_cast<size_t>(root)]) {
      visited[static_cast<size_t>(root)] = 1;
      roots.push_back(root);
      cluster.min_x = cluster.max_x = point.x();
      cluster.min_y = cluster.max_y = point.y();
      cluster.min_z = cluster.max_z = point.z();
      cluster.points = 0;
    }
    cluster.min_x = std::min(cluster.min_x, static_cast<double>(point.x()));
    cluster.max_x = std::max(cluster.max_x, static_cast<double>(point.x()));
    cluster.min_y = std::min(cluster.min_y, static_cast<double>(point.y()));
    cluster.max_y = std::max(cluster.max_y, static_cast<double>(point.y()));
    cluster.min_z = std::min(cluster.min_z, static_cast<double>(point.z()));
    cluster.max_z = std::max(cluster.max_z, static_cast<double>(point.z()));
    ++cluster.points;
  }
  stats.clusters = roots.size();

  std::vector<uint8_t> accepted(roots.size(), 0);
  for (size_t slot = 0; slot < roots.size(); ++slot) {
    accepted[slot] = accepts(clusters_[static_cast<size_t>(roots[slot])]) ? 1 : 0;
    if (accepted[slot] != 0) {
      ++stats.pedestrian_clusters;
    } else {
      ++stats.rejected_clusters;
    }
  }

  std::unordered_map<int32_t, uint8_t> verdict;
  verdict.reserve(roots.size() * 2 + 1);
  for (size_t slot = 0; slot < roots.size(); ++slot) {
    verdict.emplace(roots[slot], accepted[slot]);
  }

  for (size_t slot = 0; slot < count; ++slot) {
    const int32_t root = find_root(static_cast<int32_t>(slot));
    const auto it = verdict.find(root);
    const bool hit = it != verdict.end() && it->second != 0;
    if (hit) {
      (*pedestrian_mask)[static_cast<size_t>(candidates_[slot])] = 1;
      ++stats.pedestrian_points;
    } else if (!params_.keep_unmatched) {
      (*pedestrian_mask)[static_cast<size_t>(candidates_[slot])] = 1;
      ++stats.pedestrian_points;
    }
  }
  stats.voxel.dynamic_points = stats.pedestrian_points;
  return stats;
}

}
