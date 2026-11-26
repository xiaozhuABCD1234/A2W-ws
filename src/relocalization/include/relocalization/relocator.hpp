#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <string>
#include <vector>

#include "relocalization/types.hpp"
#include "relocalization/voxel_index.hpp"

namespace relocalization {

struct RelocalizationResult {
  Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
  double fitness = 0.0;
  double inlier_ratio = 0.0;
  double ambiguity_margin = 0.0;
  double score = 0.0;
  double search_ms = 0.0;
  double verify_ms = 0.0;
  int candidate_count = 0;
  int verified_count = 0;
  int tilt_count = 0;
};

class PriorMapRelocator {
 public:
  explicit PriorMapRelocator(const RelocalizationParams& params);

  bool loadMap(const std::string& path, std::string* error = nullptr);
  bool loadCloud(const PointCloudT& cloud, std::string* error = nullptr);
  bool ready() const { return !levels_.empty(); }
  size_t mapSize() const { return map_size_; }
  size_t levelCount() const { return levels_.size(); }
  double zMin() const { return z_min_; }
  double zMax() const { return z_max_; }
  double levelResolution(size_t level) const;

  std::optional<RelocalizationResult> localize(const PointCloudT& cloud_body,
                                               const Eigen::Isometry3d& odom_pose,
                                               std::string* rejection = nullptr) const;

 private:
  struct Level {
    VoxelIndex index;
  };

  struct ScoredPose {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    double score = 0.0;
  };

  struct Verified {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    double fitness = 0.0;
    double inlier_ratio = 0.0;
  };

  std::vector<Eigen::Matrix3d> buildTilts(const Eigen::Isometry3d& odom_pose) const;
  PointCloudPtr downsample(const PointCloudT& cloud, double voxel, int max_points) const;
  double scorePose(const Level& level, const PointCloudT& query, const Eigen::Matrix3d& rotation,
                   const Eigen::Vector3d& translation) const;
  std::vector<ScoredPose> searchGlobal(const PointCloudT& query,
                                       const std::vector<Eigen::Matrix3d>& tilts) const;
  std::vector<ScoredPose> expandLevel(size_t level_index, const PointCloudT& query,
                                      const std::vector<ScoredPose>& parents) const;
  std::vector<ScoredPose> prune(std::vector<ScoredPose> poses, size_t keep) const;
  std::vector<ScoredPose> selectDistinct(const std::vector<ScoredPose>& poses, int limit) const;
  std::optional<Verified> verify(const PointCloudT& source, const ScoredPose& candidate,
                                 std::string* rejection) const;
  double inlierRatio(const PointCloudT& cloud, const Eigen::Isometry3d& pose) const;
  bool insideBounds(const Eigen::Vector3d& translation, double margin) const;
  double tiltFromUp(const Eigen::Matrix3d& rotation) const;
  Eigen::Matrix3d upAlignment() const;
  size_t keepFor(size_t level) const;

  RelocalizationParams params_;
  std::vector<Level> levels_;
  VoxelIndex verify_index_;
  PointCloudPtr verify_target_;
  Eigen::Vector3d up_ = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d min_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d max_ = Eigen::Vector3d::Zero();
  double z_min_ = 0.0;
  double z_max_ = 0.0;
  size_t map_size_ = 0;
};

}
