#pragma once

#include <pcl/kdtree/kdtree_flann.h>

#include <optional>
#include <string>
#include <vector>

#include "relocalization/types.hpp"

namespace relocalization {

struct RelocalizationResult {
  Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
  double fitness = 0.0;
  double inlier_ratio = 0.0;
  double ambiguity_margin = 0.0;
  int candidate_count = 0;
};

class PriorMapRelocator {
 public:
  explicit PriorMapRelocator(const RelocalizationParams& params);

  bool loadMap(const std::string& path, std::string* error = nullptr);
  bool ready() const { return !levels_.empty(); }
  size_t mapSize() const { return map_size_; }

  std::optional<RelocalizationResult> localize(const PointCloudT& cloud_body,
                                                std::string* rejection = nullptr) const;

 private:
  struct Level {
    PointCloudPtr cloud;
    pcl::KdTreeFLANN<PointT> tree;
    double voxel = 0.0;
  };

  struct Candidate {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    double score = 0.0;
  };

  struct VerifiedCandidate {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    double fitness = 0.0;
    double inlier_ratio = 0.0;
  };

  PointCloudPtr downsample(const PointCloudT& cloud, double voxel, int max_points) const;
  std::vector<Candidate> globalSearch(const Level& level, const PointCloudT& query) const;
  std::vector<Candidate> refine(const Level& level, const PointCloudT& query,
                                const std::vector<Candidate>& parents, double xy_step,
                                double z_step, double yaw_step) const;
  std::vector<Candidate> scorePoses(const Level& level, const PointCloudT& query,
                                    const std::vector<Eigen::Isometry3d>& poses) const;
  std::vector<Candidate> selectCandidates(std::vector<Candidate> candidates,
                                          double translation_separation,
                                          double yaw_separation) const;
  double scorePose(const Level& level, const PointCloudT& query,
                   const Eigen::Isometry3d& pose) const;
  std::optional<VerifiedCandidate> verify(const PointCloudT& cloud_body,
                                          const Candidate& candidate,
                                          std::string* rejection) const;
  double inlierRatio(const PointCloudT& source, const PointCloudT& target,
                     const Eigen::Isometry3d& pose, double max_distance) const;
  bool insideSearchBounds(const Eigen::Isometry3d& pose) const;

  RelocalizationParams params_;
  std::vector<Level> levels_;
  Eigen::Vector3d map_min_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d map_max_ = Eigen::Vector3d::Zero();
  size_t map_size_ = 0;
};

}
