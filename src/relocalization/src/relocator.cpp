#include "relocalization/relocator.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace relocalization {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double rotationDistance(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  return Eigen::AngleAxisd(a.transpose() * b).angle();
}

std::vector<double> axisValues(double minimum, double maximum, double step) {
  std::vector<double> values;
  if (!(maximum >= minimum) || !(step > 0.0)) return values;
  for (double value = minimum; value < maximum; value += step) {
    values.push_back(value);
    if (values.size() > 200000) break;
  }
  if (values.empty() || maximum - values.back() > 1.0e-6) values.push_back(maximum);
  return values;
}

}

PriorMapRelocator::PriorMapRelocator(const RelocalizationParams& params) : params_(params) {}

bool PriorMapRelocator::loadMap(const std::string& path, std::string* error) {
  levels_.clear();
  map_size_ = 0;
  PointCloudPtr map(new PointCloudT());
  if (pcl::io::loadPCDFile<PointT>(path, *map) != 0) {
    if (error) *error = "failed to load " + path;
    return false;
  }
  PointCloudPtr finite_map(new PointCloudT());
  finite_map->reserve(map->size());
  for (const auto& point : map->points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      finite_map->push_back(point);
    }
  }
  if (finite_map->size() < 20) {
    if (error) *error = "prior map has fewer than 20 finite points";
    return false;
  }
  map_min_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  map_max_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
  for (const auto& point : finite_map->points) {
    const Eigen::Vector3d p(point.x, point.y, point.z);
    map_min_ = map_min_.cwiseMin(p);
    map_max_ = map_max_.cwiseMax(p);
  }
  map_size_ = finite_map->size();

  std::vector<double> voxels = params_.pyramid_voxels;
  voxels.erase(std::remove_if(voxels.begin(), voxels.end(),
                              [](double value) { return !(value > 0.0); }),
               voxels.end());
  if (voxels.empty()) voxels.push_back(0.5);
  std::sort(voxels.begin(), voxels.end(), std::greater<double>());
  voxels.erase(std::unique(voxels.begin(), voxels.end()), voxels.end());
  for (double voxel_size : voxels) {
    Level level;
    level.voxel = voxel_size;
    level.cloud = downsample(*finite_map, voxel_size, 0);
    if (level.cloud->size() < 20) {
      levels_.clear();
      map_size_ = 0;
      if (error) *error = "prior map pyramid level has fewer than 20 points";
      return false;
    }
    level.tree.setInputCloud(level.cloud);
    levels_.push_back(std::move(level));
  }
  return true;
}

PointCloudPtr PriorMapRelocator::downsample(const PointCloudT& cloud, double voxel,
                                             int max_points) const {
  PointCloudPtr filtered(new PointCloudT());
  if (voxel > 0.0) {
    pcl::VoxelGrid<PointT> filter;
    const float leaf = static_cast<float>(voxel);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.setInputCloud(PointCloudConstPtr(new PointCloudT(cloud)));
    filter.filter(*filtered);
  } else {
    *filtered = cloud;
  }
  if (max_points > 0 && filtered->size() > static_cast<size_t>(max_points)) {
    PointCloudPtr thinned(new PointCloudT());
    thinned->reserve(static_cast<size_t>(max_points));
    const double stride = static_cast<double>(filtered->size()) / max_points;
    for (double index = 0.0; index < static_cast<double>(filtered->size()); index += stride) {
      thinned->push_back(filtered->points[static_cast<size_t>(index)]);
    }
    filtered.swap(thinned);
  }
  filtered->width = static_cast<uint32_t>(filtered->size());
  filtered->height = 1;
  return filtered;
}

std::vector<PriorMapRelocator::Candidate> PriorMapRelocator::globalSearch(const Level& level,
                                                        const PointCloudT& query) const {
  const auto xs = axisValues(map_min_.x(), map_max_.x(), params_.coarse_xy_step);
  const auto ys = axisValues(map_min_.y(), map_max_.y(), params_.coarse_xy_step);
  const double z_min = params_.search_z_min;
  const double z_max = params_.search_z_max;
  const auto zs = axisValues(z_min, z_max, params_.coarse_z_step);
  const double yaw_step = params_.coarse_yaw_step_deg * kPi / 180.0;
  const auto yaws = axisValues(-kPi, kPi - yaw_step, yaw_step);
  std::vector<Eigen::Isometry3d> poses;
  poses.reserve(xs.size() * ys.size() * zs.size() * yaws.size());
  for (double x : xs) {
    for (double y : ys) {
      for (double z : zs) {
        for (double yaw : yaws) {
          Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
          pose.translation() = Eigen::Vector3d(x, y, z);
          pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
          poses.push_back(pose);
        }
      }
    }
  }
  return selectCandidates(scorePoses(level, query, poses), params_.coarse_xy_step,
                          yaw_step);
}

std::vector<PriorMapRelocator::Candidate> PriorMapRelocator::refine(
    const Level& level, const PointCloudT& query, const std::vector<Candidate>& parents,
    double xy_step, double z_step, double yaw_step) const {
  std::vector<Eigen::Isometry3d> poses;
  poses.reserve(parents.size() * 81);
  for (const auto& parent : parents) {
    const double parent_yaw =
        std::atan2(parent.pose.linear()(1, 0), parent.pose.linear()(0, 0));
    for (int ix = -1; ix <= 1; ++ix) {
      for (int iy = -1; iy <= 1; ++iy) {
        for (int iz = -1; iz <= 1; ++iz) {
          for (int iyaw = -1; iyaw <= 1; ++iyaw) {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = parent.pose.translation() +
                                 Eigen::Vector3d(ix * xy_step, iy * xy_step, iz * z_step);
            pose.linear() = Eigen::AngleAxisd(wrapAngle(parent_yaw + iyaw * yaw_step),
                                              Eigen::Vector3d::UnitZ())
                                .toRotationMatrix();
            if (insideSearchBounds(pose)) poses.push_back(pose);
          }
        }
      }
    }
  }
  return selectCandidates(scorePoses(level, query, poses), xy_step, yaw_step);
}

std::vector<PriorMapRelocator::Candidate> PriorMapRelocator::scorePoses(
    const Level& level, const PointCloudT& query,
    const std::vector<Eigen::Isometry3d>& poses) const {
  std::vector<Candidate> scored(poses.size());
#pragma omp parallel for schedule(dynamic, 1) if(poses.size() > 16)
  for (long long index = 0; index < static_cast<long long>(poses.size()); ++index) {
    Candidate candidate;
    candidate.pose = poses[static_cast<size_t>(index)];
    candidate.score = scorePose(level, query, candidate.pose);
    scored[static_cast<size_t>(index)] = candidate;
  }
  return scored;
}

std::vector<PriorMapRelocator::Candidate> PriorMapRelocator::selectCandidates(
    std::vector<Candidate> candidates, double translation_separation,
    double yaw_separation) const {
  candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                  [](const Candidate& candidate) {
                                    return !std::isfinite(candidate.score) ||
                                           candidate.score <= 0.0;
                                  }),
                   candidates.end());
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& lhs, const Candidate& rhs) { return lhs.score > rhs.score; });
  std::vector<Candidate> selected;
  const int limit = std::max(1, params_.max_candidates);
  for (const auto& candidate : candidates) {
    bool separated = true;
    for (const auto& other : selected) {
      const double translation =
          (candidate.pose.translation() - other.pose.translation()).norm();
      const double rotation = rotationDistance(candidate.pose.linear(), other.pose.linear());
      if (translation < translation_separation * 0.75 && rotation < yaw_separation * 0.75) {
        separated = false;
        break;
      }
    }
    if (separated) selected.push_back(candidate);
    if (static_cast<int>(selected.size()) >= limit) break;
  }
  return selected;
}

double PriorMapRelocator::scorePose(const Level& level, const PointCloudT& query,
                                    const Eigen::Isometry3d& pose) const {
  if (query.empty()) return 0.0;
  const double max_distance = std::max(0.5, 1.5 * level.voxel);
  const double max_squared = max_distance * max_distance;
  size_t inliers = 0;
  double proximity = 0.0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (const auto& point : query.points) {
    const Eigen::Vector3d transformed = pose * Eigen::Vector3d(point.x, point.y, point.z);
    PointT query_point;
    query_point.x = static_cast<float>(transformed.x());
    query_point.y = static_cast<float>(transformed.y());
    query_point.z = static_cast<float>(transformed.z());
    if (level.tree.nearestKSearch(query_point, 1, indices, distances) != 1) continue;
    if (static_cast<double>(distances[0]) > max_squared) continue;
    ++inliers;
    proximity += 1.0 - std::sqrt(static_cast<double>(distances[0])) / max_distance;
  }
  const double count = static_cast<double>(query.size());
  return 0.65 * static_cast<double>(inliers) / count + 0.35 * proximity / count;
}

bool PriorMapRelocator::insideSearchBounds(const Eigen::Isometry3d& pose) const {
  const auto& t = pose.translation();
  return t.x() >= map_min_.x() && t.x() <= map_max_.x() && t.y() >= map_min_.y() &&
         t.y() <= map_max_.y() && t.z() >= params_.search_z_min &&
         t.z() <= params_.search_z_max;
}

std::optional<PriorMapRelocator::VerifiedCandidate> PriorMapRelocator::verify(
    const PointCloudT& cloud_body, const Candidate& candidate, std::string* rejection) const {
  auto fail = [rejection](const std::string& reason) {
    if (rejection) *rejection = reason;
    return std::nullopt;
  };
  PointCloudPtr source = downsample(cloud_body, params_.gicp_voxel, params_.gicp_max_points);
  if (params_.filter_ground && source->size() >= 50) {
    pcl::SACSegmentation<PointT> segmentation;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(10.0 * kPi / 180.0);
    segmentation.setMaxIterations(100);
    segmentation.setDistanceThreshold(params_.ground_distance_threshold);
    segmentation.setInputCloud(source);
    segmentation.segment(*inliers, *coefficients);
    if (inliers->indices.size() >= 20) {
      pcl::ExtractIndices<PointT> extract;
      PointCloudPtr without_ground(new PointCloudT());
      extract.setInputCloud(source);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*without_ground);
      if (without_ground->size() >= 20) source = without_ground;
    }
  }
  const PointCloudPtr& target = levels_.back().cloud;
  if (source->size() < 20 || target->size() < 20) return fail("too few GICP points");

  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
  gicp.setInputSource(source);
  gicp.setInputTarget(target);
  gicp.setMaximumIterations(std::max(1, params_.gicp_iterations));
  gicp.setMaxCorrespondenceDistance(params_.gicp_max_correspondence);
  PointCloudT aligned;
  gicp.align(aligned, candidate.pose.matrix().cast<float>());
  if (!gicp.hasConverged() || aligned.empty()) return fail("GICP did not converge");

  VerifiedCandidate verified;
  verified.pose.matrix() = gicp.getFinalTransformation().cast<double>();
  verified.fitness = gicp.getFitnessScore(params_.validation_correspondence);
  if (!verified.pose.matrix().allFinite() || !std::isfinite(verified.fitness) ||
      verified.fitness > params_.max_fitness) {
    return fail("fitness threshold: " + std::to_string(verified.fitness));
  }
  const Eigen::Isometry3d correction = verified.pose * candidate.pose.inverse();
  if (correction.translation().norm() > params_.max_refinement_trans ||
      Eigen::AngleAxisd(correction.linear()).angle() * 180.0 / kPi >
          params_.max_refinement_rot_deg) {
    return fail("refinement threshold");
  }
  const auto& rotation = verified.pose.linear();
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::atan2(-rotation(2, 0),
                                  std::hypot(rotation(2, 1), rotation(2, 2)));
  if (std::max(std::abs(roll), std::abs(pitch)) * 180.0 / kPi >
      params_.max_roll_pitch_deg) {
    std::ostringstream reason;
    reason << "gravity threshold roll=" << roll * 180.0 / kPi
           << " pitch=" << pitch * 180.0 / kPi << " fitness=" << verified.fitness;
    return fail(reason.str());
  }
  verified.inlier_ratio = inlierRatio(*source, *target, verified.pose,
                                      params_.validation_correspondence);
  if (verified.inlier_ratio < params_.min_inlier_ratio) {
    return fail("inlier threshold: " + std::to_string(verified.inlier_ratio));
  }
  return verified;
}

double PriorMapRelocator::inlierRatio(const PointCloudT& source, const PointCloudT& target,
                                      const Eigen::Isometry3d& pose,
                                      double max_distance) const {
  if (source.empty()) return 0.0;
  PointCloudT transformed;
  pcl::transformPointCloud(source, transformed, pose.matrix().cast<float>());
  pcl::KdTreeFLANN<PointT> tree;
  PointCloudConstPtr target_ptr(new PointCloudT(target));
  tree.setInputCloud(target_ptr);
  const double max_squared = max_distance * max_distance;
  size_t inliers = 0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (const auto& point : transformed.points) {
    if (tree.nearestKSearch(point, 1, indices, distances) == 1 &&
        static_cast<double>(distances[0]) <= max_squared) {
      ++inliers;
    }
  }
  return static_cast<double>(inliers) / static_cast<double>(source.size());
}

std::optional<RelocalizationResult> PriorMapRelocator::localize(
    const PointCloudT& cloud_body, std::string* rejection) const {
  auto fail = [rejection](const std::string& reason) {
    if (rejection) *rejection = reason;
    return std::nullopt;
  };
  if (!ready() || cloud_body.size() < 20) return fail("invalid map or query");
  PointCloudPtr coarse_query =
      downsample(cloud_body, levels_.front().voxel, params_.max_query_points);
  if (coarse_query->size() < 20) return fail("too few coarse query points");
  std::vector<Candidate> candidates = globalSearch(levels_.front(), *coarse_query);
  if (candidates.empty()) return fail("no global candidates");

  double parent_xy_step = params_.coarse_xy_step;
  double parent_z_step = params_.coarse_z_step;
  double parent_yaw_step = params_.coarse_yaw_step_deg * kPi / 180.0;
  for (size_t index = 1; index < levels_.size(); ++index) {
    const double xy_step = parent_xy_step * 0.5;
    const double z_step = parent_z_step * 0.5;
    const double yaw_step = parent_yaw_step * 0.5;
    PointCloudPtr query = downsample(cloud_body, levels_[index].voxel,
                                     params_.max_query_points * 2);
    candidates = refine(levels_[index], *query, candidates, xy_step, z_step, yaw_step);
    if (candidates.empty()) return fail("no refined candidates");
    parent_xy_step = xy_step;
    parent_z_step = z_step;
    parent_yaw_step = yaw_step;
  }

  std::vector<VerifiedCandidate> verified;
  std::ostringstream failures;
  for (const auto& candidate : candidates) {
    std::string candidate_rejection;
    auto result = verify(cloud_body, candidate, &candidate_rejection);
    if (result) {
      verified.push_back(std::move(*result));
    } else {
      const auto& t = candidate.pose.translation();
      const double yaw = std::atan2(candidate.pose.linear()(1, 0), candidate.pose.linear()(0, 0));
      failures << "[" << t.x() << "," << t.y() << "," << t.z() << "," << yaw
               << ";" << candidate.score << ";" << candidate_rejection << "]";
    }
  }
  if (verified.empty()) return fail(failures.str());
  std::sort(verified.begin(), verified.end(),
            [](const VerifiedCandidate& lhs, const VerifiedCandidate& rhs) {
              return lhs.fitness < rhs.fitness;
            });

  double ambiguity_margin = std::numeric_limits<double>::infinity();
  for (size_t index = 1; index < verified.size(); ++index) {
    const double translation =
        (verified[index].pose.translation() - verified[0].pose.translation()).norm();
    const double rotation = rotationDistance(verified[index].pose.linear(),
                                             verified[0].pose.linear()) *
                            180.0 / kPi;
    if (translation >= params_.ambiguity_translation ||
        rotation >= params_.ambiguity_rotation_deg) {
      ambiguity_margin = verified[index].fitness - verified[0].fitness;
      break;
    }
  }
  if (ambiguity_margin < params_.min_ambiguity_margin) {
    return fail("ambiguous candidates: " + std::to_string(ambiguity_margin));
  }

  RelocalizationResult result;
  result.T_map_body = verified[0].pose;
  result.fitness = verified[0].fitness;
  result.inlier_ratio = verified[0].inlier_ratio;
  result.ambiguity_margin = ambiguity_margin;
  result.candidate_count = static_cast<int>(verified.size());
  return result;
}

}
