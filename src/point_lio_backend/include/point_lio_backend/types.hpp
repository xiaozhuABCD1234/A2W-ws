#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace point_lio_backend {

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloudT::Ptr;
using PointCloudConstPtr = PointCloudT::ConstPtr;

// ── 参数（唯一真相源：config/backend_mid360.yaml）─────────────────────────
struct ScanContextParams {
  int rings = 20;
  int sectors = 60;
  double max_radius = 40.0;          // m
  double ring_key_threshold = 0.15;  // 粗筛阈值
  double sc_distance_threshold = 0.35;  // 精筛阈值
  int top_candidates = 5;
  int min_index_gap = 20;            // 至少隔 20 个关键帧
  double min_time_gap_sec = 10.0;
};

struct KeyframeParams {
  double min_distance = 1.0;         // m
  double min_rotation_deg = 10.0;
  double max_interval_sec = 2.0;
  double min_interval_sec = 0.3;
  int submap_frames = 4;
  double submap_voxel = 0.2;         // m
  int max_keyframes = 2000;
};

struct IcpParams {
  double max_correspondence = 1.0;   // m
  int max_iterations = 40;
  double max_fitness = 0.10;         // GICP 均方误差
  double min_inlier_ratio = 0.6;
  double max_correction_trans = 5.0; // m
  double max_correction_rot_deg = 30.0;
};

struct GraphParams {
  double odom_trans_sigma = 0.02;    // m
  double odom_rot_sigma_deg = 0.5;
  double loop_trans_sigma = 0.10;    // m
  double loop_rot_sigma_deg = 1.5;
  double huber_delta = 1.0;
  int max_iterations = 50;
  int num_threads = 2;
  double max_loop_residual_trans = 0.5;  // m
  double max_loop_residual_rot_deg = 5.0;
  bool remove_outlier_loops = true;
};

struct OutputParams {
  double map_publish_period = 2.0;   // s
  double map_voxel = 0.2;            // m
  double correction_tau = 1.5;       // s，0 = 直接跳变
  bool publish_odom = true;
  bool publish_raw_path = false;
};

struct RuntimeParams {
  int pending_clouds = 2;
  double reset_on_jump_m = 2.0;
  double stamp_tolerance_sec = 0.05;
  double max_loop_solve_sec = 0.5;
};

// ── 数据结构 ───────────────────────────────────────────────────────────────
// pose_raw = T_odom_body（前端 camera_init 系），pose_opt = T_map_body（后端 map 系）。
struct Keyframe {
  int id = -1;
  double stamp = 0.0;
  Eigen::Isometry3d pose_raw = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose_opt = Eigen::Isometry3d::Identity();
  PointCloudPtr cloud;              // body 系子图，已降采样
  Eigen::MatrixXd descriptor;       // Scan Context：rings x sectors
  Eigen::VectorXd ring_key;         // 每环均值
};

// 回环约束：T_from_to 把 to 帧的点变到 from 帧（p_from = T_from_to * p_to），
// 与 Ceres 里 Z_ij = T_i^{-1} T_j（i = from, j = to）一致。
struct LoopConstraint {
  int from = -1;
  int to = -1;
  Eigen::Isometry3d T_from_to = Eigen::Isometry3d::Identity();
  double fitness = 0.0;
  double inlier_ratio = 0.0;
  bool active = true;
};

struct BackendParams {
  std::string world_frame = "map";
  std::string odom_frame = "camera_init";
  std::string body_frame = "body";
  std::string odom_topic = "/aft_mapped_to_init";
  std::string cloud_topic = "/cloud_registered_body";
  std::string path_topic = "/backend/path";
  std::string odom_out_topic = "/backend/odom";
  std::string map_topic = "/backend/map_points";
  std::string loop_marker_topic = "/backend/loop_markers";
  std::string status_topic = "/backend/status";

  ScanContextParams scan_context;
  KeyframeParams keyframe;
  IcpParams icp;
  GraphParams graph;
  OutputParams output;
  RuntimeParams runtime;
};

// ── 小工具 ─────────────────────────────────────────────────────────────────
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

inline Eigen::Matrix<double, 6, 6> makeInformation(double trans_sigma, double rot_sigma_rad) {
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
  const double wt = 1.0 / (trans_sigma * trans_sigma);
  const double wr = 1.0 / (rot_sigma_rad * rot_sigma_rad);
  info.diagonal() << wt, wt, wt, wr, wr, wr;
  return info;
}

}  // namespace point_lio_backend
