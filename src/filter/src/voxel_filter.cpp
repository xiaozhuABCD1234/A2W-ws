#include "filter/voxel_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace filter {
namespace {

constexpr int64_t kAxisBits = 21;
constexpr int64_t kAxisOffset = int64_t{1} << 20;
constexpr int64_t kAxisMask = (int64_t{1} << kAxisBits) - 1;
constexpr double kPi = 3.14159265358979323846;
constexpr int kMaxRaySteps = 512;

}

VoxelFilter::VoxelFilter(const FilterParams& params) : params_(params) {
  params_.voxel_size = std::max(0.05, params_.voxel_size);
  params_.min_range = std::max(0.0, params_.min_range);
  params_.max_range = std::max(params_.min_range + params_.voxel_size, params_.max_range);
  params_.azimuth_step_deg = std::clamp(params_.azimuth_step_deg, 0.5, 30.0);
  params_.elevation_step_deg = std::clamp(params_.elevation_step_deg, 0.5, 30.0);
  params_.free_frames_threshold = std::max(1, params_.free_frames_threshold);
  params_.occupied_score = std::max(1, params_.occupied_score);
  params_.free_score = std::max(1, params_.free_score);
  params_.min_score = std::min(-1, params_.min_score);
  params_.max_score = std::max(1, params_.max_score);
  params_.free_label_score = std::clamp(params_.free_label_score, params_.min_score, -1);
  params_.occupied_recent_gain = std::clamp(params_.occupied_recent_gain, 1, 16);
  params_.relearn_frames = std::max(0, params_.relearn_frames);
  params_.decay_period_frames = std::max(1, params_.decay_period_frames);
  voxel_size_ = static_cast<float>(params_.voxel_size);
  min_range2_ = static_cast<float>(params_.min_range * params_.min_range);
  max_range2_ = static_cast<float>(params_.max_range * params_.max_range);
  azimuth_bins_ = std::max(1, static_cast<int>(std::ceil(360.0 / params_.azimuth_step_deg)));
  elevation_bins_ = std::max(1, static_cast<int>(std::ceil(180.0 / params_.elevation_step_deg)));
  bins_.assign(static_cast<size_t>(azimuth_bins_) * static_cast<size_t>(elevation_bins_), Bin{});
}

void VoxelFilter::reset() {
  voxels_.clear();
  dynamic_cells_.clear();
  blocked_cells_.clear();
  considered_.clear();
  cells_.clear();
  touched_bins_.clear();
  frame_id_ = 0;
}

int64_t VoxelFilter::pack(int32_t x, int32_t y, int32_t z) {
  const int64_t px = (static_cast<int64_t>(x) + kAxisOffset) & kAxisMask;
  const int64_t py = (static_cast<int64_t>(y) + kAxisOffset) & kAxisMask;
  const int64_t pz = (static_cast<int64_t>(z) + kAxisOffset) & kAxisMask;
  return (px << (2 * kAxisBits)) | (py << kAxisBits) | pz;
}

VoxelFilter::Cell VoxelFilter::unpack(int64_t key) {
  Cell cell;
  cell.x = static_cast<int32_t>(((key >> (2 * kAxisBits)) & kAxisMask) - kAxisOffset);
  cell.y = static_cast<int32_t>(((key >> kAxisBits) & kAxisMask) - kAxisOffset);
  cell.z = static_cast<int32_t>((key & kAxisMask) - kAxisOffset);
  return cell;
}

VoxelFilter::Voxel& VoxelFilter::touch(int32_t x, int32_t y, int32_t z) {
  return voxels_[pack(x, y, z)];
}

VoxelFilter::Voxel* VoxelFilter::find(int32_t x, int32_t y, int32_t z) {
  const auto it = voxels_.find(pack(x, y, z));
  return it == voxels_.end() ? nullptr : &it->second;
}

bool VoxelFilter::knownFree(const Voxel& voxel) const {
  return voxel.score <= static_cast<int16_t>(params_.free_label_score) &&
         voxel.free_frames >= static_cast<uint16_t>(params_.free_frames_threshold) &&
         voxel.occupied_recent == 0;
}

void VoxelFilter::applyFree(int32_t x, int32_t y, int32_t z) {
  Voxel& voxel = touch(x, y, z);
  if (voxel.free_frame == frame_id_) return;
  voxel.free_frame = frame_id_;
  if (voxel.free_frames < std::numeric_limits<uint16_t>::max()) ++voxel.free_frames;
  if (voxel.occupied_frames > 0) --voxel.occupied_frames;
  if (voxel.occupied_recent > 0) --voxel.occupied_recent;
  voxel.dynamic_streak = 0;
  const int next = static_cast<int>(voxel.score) - params_.free_score;
  voxel.score = static_cast<int16_t>(std::max(next, params_.min_score));
}

void VoxelFilter::applyOccupied(Voxel& voxel) {
  if (voxel.occupied_frame == frame_id_) return;
  voxel.occupied_frame = frame_id_;
  if (voxel.occupied_frames < std::numeric_limits<uint16_t>::max()) ++voxel.occupied_frames;
  voxel.occupied_recent = static_cast<uint8_t>(std::min(
      255, static_cast<int>(voxel.occupied_recent) + params_.occupied_recent_gain));
  const int next = static_cast<int>(voxel.score) + params_.occupied_score;
  voxel.score = static_cast<int16_t>(std::min(next, params_.max_score));
}

void VoxelFilter::castFreeRay(const Eigen::Vector3f& origin, const Eigen::Vector3f& target) {
  const Eigen::Vector3f origin_cell = origin / voxel_size_;
  const Eigen::Vector3f target_cell = target / voxel_size_;
  int32_t cell[3];
  int32_t end[3];
  int32_t step[3];
  float t_max[3];
  float t_delta[3];
  for (int axis = 0; axis < 3; ++axis) {
    cell[axis] = static_cast<int32_t>(std::floor(origin_cell[axis]));
    end[axis] = static_cast<int32_t>(std::floor(target_cell[axis]));
    const float delta = target_cell[axis] - origin_cell[axis];
    if (std::fabs(delta) < 1e-6f) {
      step[axis] = 0;
      t_max[axis] = std::numeric_limits<float>::infinity();
      t_delta[axis] = std::numeric_limits<float>::infinity();
      continue;
    }
    step[axis] = delta > 0.0f ? 1 : -1;
    const float boundary = static_cast<float>(cell[axis]) + (delta > 0.0f ? 1.0f : 0.0f);
    t_max[axis] = (boundary - origin_cell[axis]) / delta;
    t_delta[axis] = 1.0f / std::fabs(delta);
  }
  for (int guard = 0; guard < kMaxRaySteps; ++guard) {
    int axis = 0;
    if (t_max[1] < t_max[0]) axis = 1;
    if (t_max[2] < t_max[axis]) axis = 2;
    if (!(t_max[axis] <= 1.0f)) break;
    cell[axis] += step[axis];
    t_max[axis] += t_delta[axis];
    if (cell[0] == end[0] && cell[1] == end[1] && cell[2] == end[2]) break;
    applyFree(cell[0], cell[1], cell[2]);
  }
}

void VoxelFilter::decay() {
  for (auto it = voxels_.begin(); it != voxels_.end();) {
    Voxel& voxel = it->second;
    voxel.score = static_cast<int16_t>(voxel.score / 2);
    voxel.free_frames = static_cast<uint16_t>(voxel.free_frames >> 1);
    voxel.occupied_frames = static_cast<uint16_t>(voxel.occupied_frames >> 1);
    voxel.occupied_recent = static_cast<uint8_t>(voxel.occupied_recent >> 1);
    if (voxel.dynamic_streak > 0) --voxel.dynamic_streak;
    const bool empty = voxel.score == 0 && voxel.free_frames == 0 &&
                       voxel.occupied_frames == 0 && voxel.occupied_recent == 0 &&
                       voxel.dynamic_streak == 0;
    if (empty) {
      it = voxels_.erase(it);
    } else {
      ++it;
    }
  }
}

void VoxelFilter::forget() {
  if (static_cast<double>(voxels_.size()) <= params_.max_voxels) return;
  decay();
  for (auto it = voxels_.begin(); it != voxels_.end() && voxels_.size() > params_.max_voxels;) {
    const Voxel& voxel = it->second;
    if (voxel.score >= 0 && voxel.occupied_frames == 0) {
      it = voxels_.erase(it);
    } else {
      ++it;
    }
  }
}

FrameStats VoxelFilter::filter(const std::vector<Eigen::Vector3f>& points,
                               const Eigen::Vector3f& origin,
                               std::vector<uint8_t>* dynamic_mask) {
  FrameStats stats;
  stats.input_points = points.size();
  dynamic_mask->assign(points.size(), 0);
  ++frame_id_;
  if (frame_id_ % static_cast<uint32_t>(params_.decay_period_frames) == 0) decay();
  if (static_cast<double>(voxels_.size()) > params_.max_voxels) forget();

  for (size_t index = 0; index < touched_bins_.size(); ++index) {
    bins_[static_cast<size_t>(touched_bins_[index])] = Bin{};
  }
  touched_bins_.clear();
  considered_.clear();
  cells_.clear();
  dynamic_cells_.clear();
  blocked_cells_.clear();

  const double elevation_span = 180.0 / static_cast<double>(elevation_bins_);
  const double azimuth_span = 360.0 / static_cast<double>(azimuth_bins_);
  const float min_height = static_cast<float>(params_.min_height);
  const float max_height = static_cast<float>(params_.max_height);

  for (size_t index = 0; index < points.size(); ++index) {
    const Eigen::Vector3f& point = points[index];
    const Eigen::Vector3f rel = point - origin;
    const float range2 = rel.squaredNorm();
    if (!(range2 >= min_range2_ && range2 <= max_range2_)) continue;
    if (rel.z() < min_height || rel.z() > max_height) continue;

    const int32_t cx = static_cast<int32_t>(std::floor(point.x() / voxel_size_));
    const int32_t cy = static_cast<int32_t>(std::floor(point.y() / voxel_size_));
    const int32_t cz = static_cast<int32_t>(std::floor(point.z() / voxel_size_));
    considered_.push_back(static_cast<int32_t>(index));
    cells_.push_back(Cell{cx, cy, cz});

    double azimuth = std::atan2(static_cast<double>(rel.y()), static_cast<double>(rel.x()));
    azimuth = std::atan2(std::sin(azimuth), std::cos(azimuth));
    const double elevation = std::atan2(static_cast<double>(rel.z()),
                                        std::sqrt(static_cast<double>(rel.x() * rel.x() +
                                                                      rel.y() * rel.y())));
    int azimuth_bin = static_cast<int>(std::floor((azimuth * 180.0 / kPi + 180.0) / azimuth_span));
    int elevation_bin = static_cast<int>(std::floor((elevation * 180.0 / kPi + 90.0) / elevation_span));
    azimuth_bin = std::clamp(azimuth_bin, 0, azimuth_bins_ - 1);
    elevation_bin = std::clamp(elevation_bin, 0, elevation_bins_ - 1);
    const int32_t bin_index = static_cast<int32_t>(
        static_cast<size_t>(elevation_bin) * static_cast<size_t>(azimuth_bins_) +
        static_cast<size_t>(azimuth_bin));
    Bin& bin = bins_[static_cast<size_t>(bin_index)];
    if (bin.index < 0) {
      bin.index = static_cast<int32_t>(index);
      bin.range2 = range2;
      touched_bins_.push_back(bin_index);
    } else if (range2 < bin.range2) {
      bin.index = static_cast<int32_t>(index);
      bin.range2 = range2;
    }
  }
  stats.considered_points = considered_.size();

  for (size_t slot = 0; slot < considered_.size(); ++slot) {
    const Cell& cell = cells_[slot];
    const Voxel* voxel = find(cell.x, cell.y, cell.z);
    if (voxel == nullptr || !knownFree(*voxel)) continue;
    if (params_.relearn_frames > 0 && voxel->dynamic_streak >= static_cast<uint8_t>(
                                          std::min(255, params_.relearn_frames))) {
      continue;
    }
    (*dynamic_mask)[static_cast<size_t>(considered_[slot])] = 1;
    dynamic_cells_.insert(pack(cell.x, cell.y, cell.z));
  }

  if (!touched_bins_.empty()) {
    const size_t limit = static_cast<size_t>(std::max(1, params_.max_rays_per_frame));
    const size_t stride = touched_bins_.size() > limit
                              ? (touched_bins_.size() + limit - 1) / limit
                              : 1;
    for (size_t slot = 0; slot < touched_bins_.size(); slot += stride) {
      const Bin& bin = bins_[static_cast<size_t>(touched_bins_[slot])];
      if (bin.index < 0) continue;
      castFreeRay(origin, points[static_cast<size_t>(bin.index)]);
      ++stats.rays;
    }
  }

  for (size_t slot = 0; slot < considered_.size(); ++slot) {
    const size_t index = static_cast<size_t>(considered_[slot]);
    const Cell& cell = cells_[slot];
    if ((*dynamic_mask)[index] != 0) {
      Voxel& voxel = touch(cell.x, cell.y, cell.z);
      if (voxel.dynamic_streak < 255) ++voxel.dynamic_streak;
      continue;
    }
    Voxel& voxel = touch(cell.x, cell.y, cell.z);
    applyOccupied(voxel);
  }
  stats.dynamic_voxels = dynamic_cells_.size();

  if (!dynamic_cells_.empty()) {
    const int radius = std::max(1, static_cast<int>(std::lround(
                                        params_.cluster_radius / params_.voxel_size)));
    const int radius2 = radius * radius;
    for (const int64_t key : dynamic_cells_) {
      const Cell cell = unpack(key);
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          if (dx * dx + dy * dy > radius2) continue;
          blocked_cells_.insert(pack(cell.x + dx, cell.y + dy, 0));
        }
      }
    }
    for (size_t slot = 0; slot < considered_.size(); ++slot) {
      const size_t index = static_cast<size_t>(considered_[slot]);
      if ((*dynamic_mask)[index] != 0) continue;
      const Cell& cell = cells_[slot];
      if (blocked_cells_.find(pack(cell.x, cell.y, 0)) == blocked_cells_.end()) continue;
      const Voxel* voxel = find(cell.x, cell.y, cell.z);
      if (voxel != nullptr && voxel->score >= static_cast<int16_t>(params_.occupied_score)) {
        continue;
      }
      (*dynamic_mask)[index] = 1;
    }
  }

  for (size_t index = 0; index < dynamic_mask->size(); ++index) {
    if ((*dynamic_mask)[index] != 0) ++stats.dynamic_points;
  }
  stats.map_voxels = voxels_.size();
  return stats;
}

}
