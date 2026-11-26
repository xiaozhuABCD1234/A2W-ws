#include "relocalization/relocator.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <utility>

namespace relocalization {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double yawOf(const Eigen::Matrix3d& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

Eigen::Vector3d estimateUp(const PointCloudT& cloud) {
  const Eigen::Vector3d fallback(0.0, 0.0, 1.0);
  if (cloud.size() < 200) return fallback;
  std::mt19937 rng(0x9e3779b9u);
  std::uniform_int_distribution<size_t> pick(0, cloud.size() - 1);
  std::vector<Eigen::Vector3d> points;
  points.reserve(cloud.size());
  for (const auto& point : cloud.points) {
    points.emplace_back(point.x, point.y, point.z);
  }
  const double threshold = 0.2;
  size_t best_inliers = 0;
  Eigen::Vector3d best_normal = fallback;
  for (int iteration = 0; iteration < 300; ++iteration) {
    const Eigen::Vector3d& a = points[pick(rng)];
    const Eigen::Vector3d& b = points[pick(rng)];
    const Eigen::Vector3d& c = points[pick(rng)];
    Eigen::Vector3d normal = (b - a).cross(c - a);
    const double norm = normal.norm();
    if (norm < 1.0e-6) continue;
    normal /= norm;
    if (std::abs(normal.z()) < 0.5) continue;
    if (normal.z() < 0.0) normal = -normal;
    size_t inliers = 0;
    for (const auto& point : points) {
      if (std::abs(normal.dot(point - a)) < threshold) ++inliers;
    }
    if (inliers > best_inliers) {
      best_inliers = inliers;
      best_normal = normal;
    }
  }
  if (best_inliers < points.size() / 40) return fallback;
  return best_normal;
}

double rotationDistanceDeg(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  return Eigen::AngleAxisd(a.transpose() * b).angle() * 180.0 / kPi;
}

Eigen::Matrix3d yawRotation(double yaw) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

template <typename Pose>
struct TopK {
  explicit TopK(size_t capacity) : capacity_(std::max<size_t>(1, capacity)) {
    heap_.reserve(capacity_);
  }

  void push(const Pose& pose) {
    if (heap_.size() < capacity_) {
      heap_.push_back(pose);
      std::push_heap(heap_.begin(), heap_.end(), Worse());
      return;
    }
    if (pose.score > heap_.front().score) {
      std::pop_heap(heap_.begin(), heap_.end(), Worse());
      heap_.back() = pose;
      std::push_heap(heap_.begin(), heap_.end(), Worse());
    }
  }

  std::vector<Pose> take() { return std::move(heap_); }

  struct Worse {
    bool operator()(const Pose& lhs, const Pose& rhs) const { return lhs.score > rhs.score; }
  };

  size_t capacity_;
  std::vector<Pose> heap_;
};

}

PriorMapRelocator::PriorMapRelocator(const RelocalizationParams& params) : params_(params) {
  if (params_.pyramid_voxels.empty()) params_.pyramid_voxels = {2.0, 1.0, 0.5, 0.25};
  std::vector<double> voxels;
  for (double voxel : params_.pyramid_voxels) {
    if (voxel > 0.0) voxels.push_back(voxel);
  }
  std::sort(voxels.begin(), voxels.end(), std::greater<double>());
  voxels.erase(std::unique(voxels.begin(), voxels.end()), voxels.end());
  params_.pyramid_voxels = voxels;

  std::vector<int> keep;
  for (size_t i = 0; i < params_.pyramid_voxels.size(); ++i) {
    const int value = i < params_.level_keep.size() ? params_.level_keep[i]
                                                    : std::max(4, 240 >> static_cast<int>(i));
    keep.push_back(std::max(1, value));
  }
  params_.level_keep = keep;
}

size_t PriorMapRelocator::keepFor(size_t level) const {
  if (params_.level_keep.empty()) return 32;
  return static_cast<size_t>(params_.level_keep[std::min(level, params_.level_keep.size() - 1)]);
}

double PriorMapRelocator::levelResolution(size_t level) const {
  if (level >= levels_.size()) return 0.0;
  return levels_[level].index.resolution();
}

bool PriorMapRelocator::loadMap(const std::string& path, std::string* error) {
  PointCloudPtr cloud(new PointCloudT());
  if (pcl::io::loadPCDFile<PointT>(path, *cloud) != 0) {
    if (error) *error = "failed to load " + path;
    return false;
  }
  return loadCloud(*cloud, error);
}

bool PriorMapRelocator::loadCloud(const PointCloudT& cloud, std::string* error) {
  levels_.clear();
  verify_target_.reset();
  verify_index_.clear();
  map_size_ = 0;
  z_min_ = 0.0;
  z_max_ = 0.0;

  PointCloudPtr finite(new PointCloudT());
  finite->reserve(cloud.size());
  for (const auto& point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    finite->push_back(point);
  }
  if (finite->size() < 50) {
    if (error) *error = "prior map has fewer than 50 finite points";
    return false;
  }
  finite->width = static_cast<uint32_t>(finite->size());
  finite->height = 1;

  min_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  max_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
  std::vector<float> zs;
  zs.reserve(finite->size());
  for (const auto& point : finite->points) {
    min_ = min_.cwiseMin(Eigen::Vector3d(point.x, point.y, point.z));
    max_ = max_.cwiseMax(Eigen::Vector3d(point.x, point.y, point.z));
    zs.push_back(point.z);
  }
  const size_t tail = static_cast<size_t>(0.002 * static_cast<double>(zs.size() - 1));
  const size_t low_index = tail;
  const size_t high_index = zs.size() - 1 - tail;
  std::nth_element(zs.begin(), zs.begin() + low_index, zs.end());
  const double z_low = zs[low_index];
  std::nth_element(zs.begin() + low_index, zs.begin() + high_index, zs.end());
  const double z_high = zs[high_index];

  double z_min = z_low;
  double z_max = z_high;
  if (!params_.auto_z_range) {
    z_min = std::min(params_.z_min, params_.z_max);
    z_max = std::max(params_.z_min, params_.z_max);
  }
  z_min += params_.z_margin;
  z_max -= params_.z_margin;
  if (!(z_max > z_min)) {
    z_min = z_low;
    z_max = z_high;
  }
  if (!(z_max > z_min)) {
    z_min = min_.z();
    z_max = max_.z();
  }
  z_min_ = z_min;
  z_max_ = z_max;

  levels_.reserve(params_.pyramid_voxels.size());
  for (double voxel : params_.pyramid_voxels) {
    Level level;
    level.index.build(*finite, voxel);
    if (level.index.empty()) continue;
    levels_.push_back(std::move(level));
  }
  if (levels_.size() < 2) {
    levels_.clear();
    if (error) *error = "voxel pyramid needs at least two non-empty levels";
    return false;
  }

  const double verify_voxel = std::max(params_.validation_correspondence, params_.gicp_voxel);
  up_ = estimateUp(*downsample(*finite, levels_.front().index.resolution(), 40000));
  verify_index_.build(*finite, verify_voxel);
  verify_target_ = downsample(*finite, params_.gicp_voxel, params_.gicp_max_points);
  if (!verify_target_ || verify_target_->size() < 20) {
    levels_.clear();
    if (error) *error = "gICP target cloud has fewer than 20 points";
    return false;
  }
  map_size_ = finite->size();
  return true;
}

PointCloudPtr PriorMapRelocator::downsample(const PointCloudT& cloud, double voxel,
                                            int max_points) const {
  PointCloudPtr filtered(new PointCloudT());
  if (voxel > 0.0 && !cloud.empty()) {
    PointCloudConstPtr input(&cloud, [](const PointCloudT*) {});
    pcl::VoxelGrid<PointT> grid;
    const float leaf = static_cast<float>(voxel);
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(input);
    grid.filter(*filtered);
  } else {
    *filtered = cloud;
  }
  if (max_points > 0 && static_cast<int>(filtered->size()) > max_points) {
    std::vector<int> indices(filtered->size());
    std::iota(indices.begin(), indices.end(), 0);
    std::mt19937 rng(0x5eed1234u);
    for (int i = 0; i < max_points; ++i) {
      std::uniform_int_distribution<int> dist(i, static_cast<int>(indices.size()) - 1);
      std::swap(indices[static_cast<size_t>(i)], indices[static_cast<size_t>(dist(rng))]);
    }
    PointCloudPtr thinned(new PointCloudT());
    thinned->reserve(static_cast<size_t>(max_points));
    for (int i = 0; i < max_points; ++i) {
      thinned->push_back(filtered->points[static_cast<size_t>(indices[static_cast<size_t>(i)])]);
    }
    filtered.swap(thinned);
  }
  filtered->width = static_cast<uint32_t>(filtered->size());
  filtered->height = 1;
  return filtered;
}

double PriorMapRelocator::tiltFromUp(const Eigen::Matrix3d& rotation) const {
  const Eigen::Vector3d axis = rotation * Eigen::Vector3d::UnitZ();
  return std::acos(std::clamp(axis.dot(up_), -1.0, 1.0)) * 180.0 / kPi;
}

Eigen::Matrix3d PriorMapRelocator::upAlignment() const {
  const Eigen::Vector3d unit_z(0.0, 0.0, 1.0);
  return Eigen::Quaterniond::FromTwoVectors(unit_z, up_).toRotationMatrix();
}

std::vector<Eigen::Matrix3d> PriorMapRelocator::buildTilts(
    const Eigen::Isometry3d& odom_pose) const {
  std::vector<Eigen::Matrix3d> seeds;
  auto add = [&seeds](const Eigen::Matrix3d& rotation) {
    for (const auto& seed : seeds) {
      if (Eigen::AngleAxisd(seed.transpose() * rotation).angle() * 180.0 / kPi < 1.0) return;
    }
    seeds.push_back(rotation);
  };

  const Eigen::Matrix3d rotation = odom_pose.linear();
  add(yawRotation(-yawOf(rotation)) * rotation);
  add(Eigen::Matrix3d::Identity());
  add(upAlignment());
  if (params_.tilt_mode == TiltMode::Odom) return seeds;

  const double step_deg = std::max(1.0, params_.tilt_step_deg);
  const double step = step_deg * kPi / 180.0;
  if (params_.tilt_mode == TiltMode::Free) {
    const int roll_steps = static_cast<int>(std::round(kPi / step));
    const int pitch_steps = static_cast<int>(std::round(0.5 * kPi / step));
    if (roll_steps * (2 * pitch_steps + 1) > 2048) return seeds;
    std::vector<Eigen::Matrix3d> global;
    for (int roll = -roll_steps; roll < roll_steps; ++roll) {
      for (int pitch = -pitch_steps; pitch <= pitch_steps; ++pitch) {
        global.push_back(
            Eigen::AngleAxisd(roll * step, Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(pitch * step, Eigen::Vector3d::UnitY()).toRotationMatrix());
      }
    }
    return global;
  }

  const int steps = std::max(1, static_cast<int>(std::round(params_.tilt_deg / step_deg)));
  const Eigen::Matrix3d base = upAlignment();
  for (int roll = -steps; roll <= steps; ++roll) {
    for (int pitch = -steps; pitch <= steps; ++pitch) {
      add(base * Eigen::AngleAxisd(roll * step, Eigen::Vector3d::UnitX()).toRotationMatrix() *
          Eigen::AngleAxisd(pitch * step, Eigen::Vector3d::UnitY()).toRotationMatrix());
    }
  }
  return seeds;
}

double PriorMapRelocator::scorePose(const Level& level, const PointCloudT& query,
                                    const Eigen::Matrix3d& rotation,
                                    const Eigen::Vector3d& translation) const {
  if (query.empty()) return 0.0;
  const VoxelIndex& index = level.index;
  const Eigen::Matrix3f rotation_f = rotation.cast<float>();
  const Eigen::Vector3f translation_f = translation.cast<float>();
  size_t hits = 0;
  for (const auto& point : query.points) {
    const Eigen::Vector3f transformed = rotation_f * point.getVector3fMap() + translation_f;
    if (index.occupied(transformed)) ++hits;
  }
  return static_cast<double>(hits) / static_cast<double>(query.size());
}

std::vector<PriorMapRelocator::ScoredPose> PriorMapRelocator::searchGlobal(
    const PointCloudT& query, const std::vector<Eigen::Matrix3d>& tilts) const {
  const Level& level = levels_.front();
  const double resolution = level.index.resolution();
  const double yaw_step = std::max(1.0, params_.coarse_yaw_step_deg) * kPi / 180.0;
  const int yaw_count = std::max(1, static_cast<int>(std::lround(kTwoPi / yaw_step)));

  std::vector<Eigen::Matrix3d> rotations;
  rotations.reserve(static_cast<size_t>(yaw_count) * tilts.size());
  for (const auto& tilt : tilts) {
    for (int i = 0; i < yaw_count; ++i) {
      rotations.push_back(yawRotation(i * yaw_step) * tilt);
    }
  }

  const long long nx = static_cast<long long>(std::floor((max_.x() - min_.x()) / resolution)) + 1;
  const long long ny = static_cast<long long>(std::floor((max_.y() - min_.y()) / resolution)) + 1;
  const long long nz = static_cast<long long>(std::floor((z_max_ - z_min_) / resolution)) + 1;
  const long long nrot = static_cast<long long>(rotations.size());
  const size_t keep = keepFor(0);

  std::vector<ScoredPose> merged;
  merged.reserve(keep);

#pragma omp parallel
  {
    TopK<ScoredPose> local(keep);
#pragma omp for schedule(dynamic, 1)
    for (long long ix = 0; ix < nx; ++ix) {
      const double x = min_.x() + static_cast<double>(ix) * resolution;
      for (long long iy = 0; iy < ny; ++iy) {
        const double y = min_.y() + static_cast<double>(iy) * resolution;
        for (long long iz = 0; iz < nz; ++iz) {
          const double z = z_min_ + static_cast<double>(iz) * resolution;
          const Eigen::Vector3d translation(x, y, z);
          for (long long ir = 0; ir < nrot; ++ir) {
            ScoredPose pose;
            pose.pose.linear() = rotations[static_cast<size_t>(ir)];
            pose.pose.translation() = translation;
            pose.score = scorePose(level, query, pose.pose.linear(), pose.pose.translation());
            local.push(pose);
          }
        }
      }
    }
#pragma omp critical
    {
      std::vector<ScoredPose> items = local.take();
      merged.insert(merged.end(), items.begin(), items.end());
    }
  }
  return prune(std::move(merged), keep);
}

std::vector<PriorMapRelocator::ScoredPose> PriorMapRelocator::expandLevel(
    size_t level_index, const PointCloudT& query, const std::vector<ScoredPose>& parents) const {
  const Level& level = levels_[level_index];
  const double resolution = level.index.resolution();
  const double base_yaw_step = std::max(1.0, params_.coarse_yaw_step_deg) * kPi / 180.0;
  const double yaw_step = base_yaw_step * std::pow(0.5, static_cast<double>(level_index));

  std::vector<ScoredPose> children;
  children.reserve(parents.size() * 81);
  for (const auto& parent : parents) {
    const double parent_yaw = yawOf(parent.pose.linear());
    const Eigen::Matrix3d tilt = yawRotation(-parent_yaw) * parent.pose.linear();
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const Eigen::Vector3d translation =
              parent.pose.translation() +
              Eigen::Vector3d(dx, dy, dz) * static_cast<double>(resolution);
          if (!insideBounds(translation, 2.0 * resolution)) continue;
          for (int dyaw = -1; dyaw <= 1; ++dyaw) {
            ScoredPose child;
            child.pose.linear() = yawRotation(parent_yaw + dyaw * yaw_step) * tilt;
            child.pose.translation() = translation;
            children.push_back(child);
          }
        }
      }
    }
  }
  if (children.empty()) return children;

  std::vector<double> scores(children.size(), 0.0);
#pragma omp parallel for schedule(dynamic, 16)
  for (long long i = 0; i < static_cast<long long>(children.size()); ++i) {
    const ScoredPose& child = children[static_cast<size_t>(i)];
    scores[static_cast<size_t>(i)] =
        scorePose(level, query, child.pose.linear(), child.pose.translation());
  }
  for (size_t i = 0; i < children.size(); ++i) children[i].score = scores[i];
  return children;
}

std::vector<PriorMapRelocator::ScoredPose> PriorMapRelocator::prune(
    std::vector<ScoredPose> poses, size_t keep) const {
  if (poses.empty() || keep == 0) return std::vector<ScoredPose>();
  const size_t limited = std::min(keep, poses.size());
  std::partial_sort(poses.begin(), poses.begin() + static_cast<long>(limited), poses.end(),
                    [](const ScoredPose& lhs, const ScoredPose& rhs) {
                      return lhs.score > rhs.score;
                    });
  poses.resize(limited);
  const double threshold = std::max(params_.level_min_score,
                                    params_.level_keep_ratio * poses.front().score);
  std::vector<ScoredPose> kept;
  kept.reserve(limited);
  for (auto& pose : poses) {
    if (pose.score < threshold) break;
    kept.push_back(std::move(pose));
  }
  return kept;
}

std::vector<PriorMapRelocator::ScoredPose> PriorMapRelocator::selectDistinct(
    const std::vector<ScoredPose>& poses, int limit) const {
  std::vector<ScoredPose> selected;
  const int max_count = std::max(1, limit);
  selected.reserve(static_cast<size_t>(max_count));
  for (const auto& pose : poses) {
    bool separated = true;
    for (const auto& other : selected) {
      const double translation = (pose.pose.translation() - other.pose.translation()).norm();
      const double rotation = rotationDistanceDeg(pose.pose.linear(), other.pose.linear());
      if (translation < std::max(0.1, params_.min_candidate_separation) &&
          rotation < std::max(1.0, params_.min_candidate_rotation_deg)) {
        separated = false;
        break;
      }
    }
    if (!separated) continue;
    selected.push_back(pose);
    if (static_cast<int>(selected.size()) >= max_count) break;
  }
  return selected;
}

bool PriorMapRelocator::insideBounds(const Eigen::Vector3d& translation, double margin) const {
  return translation.x() >= min_.x() - margin && translation.x() <= max_.x() + margin &&
         translation.y() >= min_.y() - margin && translation.y() <= max_.y() + margin &&
         translation.z() >= z_min_ - margin && translation.z() <= z_max_ + margin;
}

double PriorMapRelocator::inlierRatio(const PointCloudT& cloud,
                                      const Eigen::Isometry3d& pose) const {
  if (cloud.empty() || verify_index_.empty()) return 0.0;
  const Eigen::Matrix3f rotation = pose.linear().cast<float>();
  const Eigen::Vector3f translation = pose.translation().cast<float>();
  size_t hits = 0;
  for (const auto& point : cloud.points) {
    const Eigen::Vector3f transformed = rotation * point.getVector3fMap() + translation;
    if (verify_index_.occupied(transformed)) ++hits;
  }
  return static_cast<double>(hits) / static_cast<double>(cloud.size());
}

std::optional<PriorMapRelocator::Verified> PriorMapRelocator::verify(
    const PointCloudT& source, const ScoredPose& candidate, std::string* rejection) const {
  auto fail = [rejection](const std::string& reason) -> std::optional<Verified> {
    if (rejection) *rejection = reason;
    return std::optional<Verified>();
  };
  if (source.size() < 20 || !verify_target_ || verify_target_->size() < 20) {
    return fail("too few gicp points");
  }

  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
  gicp.setInputSource(PointCloudConstPtr(&source, [](const PointCloudT*) {}));
  gicp.setInputTarget(verify_target_);
  gicp.setMaximumIterations(std::max(1, params_.gicp_iterations));
  gicp.setMaxCorrespondenceDistance(std::max(0.1, params_.gicp_max_correspondence));
  PointCloudT aligned;
  gicp.align(aligned, candidate.pose.matrix().cast<float>());
  if (!gicp.hasConverged() || aligned.empty()) return fail("gicp did not converge");

  Verified verified;
  verified.pose.matrix() = gicp.getFinalTransformation().cast<double>();
  if (!verified.pose.matrix().allFinite()) return fail("non finite transform");

  verified.fitness = gicp.getFitnessScore(std::max(0.05, params_.validation_correspondence));
  if (!std::isfinite(verified.fitness) || verified.fitness > params_.max_fitness) {
    std::ostringstream reason;
    reason << "fitness " << verified.fitness;
    return fail(reason.str());
  }

  const Eigen::Isometry3d delta = verified.pose * candidate.pose.inverse();
  if (delta.translation().norm() > params_.max_refinement_trans) {
    return fail("refinement translation " + std::to_string(delta.translation().norm()));
  }
  const double delta_rot = Eigen::AngleAxisd(delta.linear()).angle() * 180.0 / kPi;
  if (delta_rot > params_.max_refinement_rot_deg) {
    return fail("refinement rotation " + std::to_string(delta_rot));
  }
  if (params_.tilt_mode != TiltMode::Free) {
    const double tilt_change =
        std::abs(tiltFromUp(verified.pose.linear()) - tiltFromUp(candidate.pose.linear()));
    if (tilt_change > params_.max_tilt_change_deg) {
      std::ostringstream reason;
      reason << "tilt change " << tilt_change;
      return fail(reason.str());
    }
  }

  verified.inlier_ratio = inlierRatio(source, verified.pose);
  if (verified.inlier_ratio < params_.min_inlier_ratio) {
    return fail("inlier ratio " + std::to_string(verified.inlier_ratio));
  }
  return verified;
}

std::optional<RelocalizationResult> PriorMapRelocator::localize(
    const PointCloudT& cloud_body, const Eigen::Isometry3d& odom_pose,
    std::string* rejection) const {
  auto fail = [rejection](const std::string& reason) -> std::optional<RelocalizationResult> {
    if (rejection) *rejection = reason;
    return std::optional<RelocalizationResult>();
  };
  if (!ready() || cloud_body.size() < 20) return fail("invalid map or query");

  const auto search_start = std::chrono::steady_clock::now();
  const std::vector<Eigen::Matrix3d> tilts = buildTilts(odom_pose);

  const double finest = levels_.back().index.resolution();
  PointCloudPtr fine = downsample(cloud_body, finest, 0);
  if (fine->size() < 20) return fail("too few query points");

  PointCloudPtr query = downsample(*fine, levels_.front().index.resolution(),
                                   std::max(20, params_.coarse_query_points));
  if (query->size() < 20) return fail("too few coarse query points");

  PointCloudPtr source = downsample(*fine, params_.gicp_voxel, params_.gicp_max_points);
  if (source->size() < 20) return fail("too few match points");

  std::vector<ScoredPose> poses = searchGlobal(*query, tilts);
  if (poses.empty()) return fail("no candidate above score threshold");
  for (size_t level = 1; level < levels_.size(); ++level) {
    PointCloudPtr level_query = downsample(*fine, levels_[level].index.resolution(),
                                           std::max(20, params_.max_query_points));
    if (level_query->size() < 20) level_query = query;
    poses = expandLevel(level, *level_query, poses);
    poses = prune(std::move(poses), keepFor(level));
    if (poses.empty()) return fail("no refined candidate at level " + std::to_string(level));
  }
  const double search_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - search_start)
                               .count();

  const std::vector<ScoredPose> candidates = selectDistinct(poses, params_.gicp_candidates);
  if (candidates.empty()) return fail("no distinct candidate");
  if (params_.min_score_margin > 0.0 && candidates.size() > 1) {
    const double best = candidates.front().score;
    const double margin =
        best > 1.0e-6 ? (best - candidates[1].score) / best : 0.0;
    if (margin < params_.min_score_margin) {
      return fail("ambiguous search margin " + std::to_string(margin));
    }
  }

  const auto verify_start = std::chrono::steady_clock::now();
  std::vector<std::optional<Verified>> verified(candidates.size());
  std::vector<std::string> reasons(candidates.size());
#pragma omp parallel for schedule(dynamic, 1)
  for (long long i = 0; i < static_cast<long long>(candidates.size()); ++i) {
    verified[static_cast<size_t>(i)] =
        verify(*source, candidates[static_cast<size_t>(i)], &reasons[static_cast<size_t>(i)]);
  }
  const double verify_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - verify_start)
                               .count();

  std::vector<Verified> accepted;
  std::ostringstream failures;
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (verified[i]) {
      accepted.push_back(*verified[i]);
    } else {
      const auto& t = candidates[i].pose.translation();
      failures << "[" << t.x() << "," << t.y() << "," << t.z() << ";" << candidates[i].score
               << ";" << reasons[i] << "]";
    }
  }
  if (accepted.empty()) return fail(failures.str());

  std::sort(accepted.begin(), accepted.end(), [](const Verified& lhs, const Verified& rhs) {
    return lhs.fitness < rhs.fitness;
  });

  double ambiguity_margin = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < accepted.size(); ++i) {
    const double translation = (accepted[i].pose.translation() -
                                accepted[0].pose.translation()).norm();
    const double rotation = rotationDistanceDeg(accepted[i].pose.linear(),
                                                accepted[0].pose.linear());
    if (translation >= params_.ambiguity_translation ||
        rotation >= params_.ambiguity_rotation_deg) {
      ambiguity_margin = accepted[i].fitness - accepted[0].fitness;
      break;
    }
  }
  if (ambiguity_margin < params_.min_ambiguity_margin) {
    return fail("ambiguous candidates: " + std::to_string(ambiguity_margin));
  }

  RelocalizationResult result;
  result.T_map_body = accepted.front().pose;
  result.fitness = accepted.front().fitness;
  result.inlier_ratio = accepted.front().inlier_ratio;
  result.ambiguity_margin = ambiguity_margin;
  result.score = candidates.front().score;
  result.search_ms = search_ms;
  result.verify_ms = verify_ms;
  result.candidate_count = static_cast<int>(candidates.size());
  result.verified_count = static_cast<int>(accepted.size());
  result.tilt_count = static_cast<int>(tilts.size());
  return result;
}

}
