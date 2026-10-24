# point_lio_backend 实施规格（团队契约）

本文件是 `feat/backend-posegraph` 分支的实施契约：接口、文件所有权、验收标准。
计划全文见同目录 `BACKEND_PLAN.md`。**未经 Lead 同意，不要改本文件或跨所有权文件。**

## 0. 背景

point_lio 是纯 LIO，没有回环；长距离绕回后轨迹漂移、octomap 把同一面墙叠成两层。
本分支新增独立后端包 `point_lio_backend`：不修改 vendored `point_lio_ros2` 与
`mid360_driver` 的任何源码，只订阅前端话题，做 Scan Context 回环检测 + GICP 校验 +
Ceres 位姿图优化，输出 `map` 系纠偏 TF、优化轨迹与优化点云。

## 1. 坐标系与话题（冻结）

    map --(backend, 20 Hz)--> camera_init --(point_lio)--> body

| 方向 | 话题 / TF | 类型 | 备注 |
|---|---|---|---|
| 入 | /aft_mapped_to_init | nav_msgs/Odometry | camera_init -> body，10 Hz |
| 入 | /cloud_registered_body | sensor_msgs/PointCloud2 | body 系，已 deskew（P4），10 Hz |
| 出 | /backend/path | nav_msgs/Path | map 系，优化后关键帧轨迹 |
| 出 | /backend/odom | nav_msgs/Odometry | map -> body，平滑后实时位姿 |
| 出 | /backend/map_points | sensor_msgs/PointCloud2 | map 系，0.2 m 体素 |
| 出 | /backend/loop_markers | visualization_msgs/MarkerArray | 回环边 + 候选 |
| 出 | /backend/status | diagnostic_msgs/DiagnosticArray | 关键帧/回环/耗时 |
| 出 | TF map -> camera_init | tf2 | 与 odom 同 stamp，20 Hz |

## 2. 文件所有权（硬边界）

| 所有者 | 文件 |
|---|---|
| Lead | CMakeLists.txt、package.xml、include/point_lio_backend/types.hpp、config/backend_mid360.yaml、launch/backend_mid360.launch.py、docs/、mid360_bringup/**、test/integration/** |
| M1 backend-core | src/backend_node.cpp、src/keyframe_manager.cpp、include/point_lio_backend/keyframe_manager.hpp、test/test_keyframe_manager.cpp |
| M2 loop | src/scan_context.cpp、include/point_lio_backend/scan_context.hpp、src/loop_detector.cpp、include/point_lio_backend/loop_detector.hpp、test/test_scan_context.cpp |
| M3 graph | src/pose_graph.cpp、include/point_lio_backend/pose_graph.hpp、test/test_pose_graph.cpp |

- 不允许新增源文件（CMake 已冻结文件列表）；需要新文件先找 Lead。
- 公共类接口见现有头文件；可以加私有成员/私有辅助函数，**不要改公共函数签名**。
- `types.hpp` 的 `Keyframe` / `LoopConstraint` / 参数结构由 Lead 维护；需要字段先找 Lead。
- 三个模块的 cpp 必须能在没有 ROS 运行时的 gtest 里链接（核心库不依赖 rclcpp）。

## 3. 冻结接口

```cpp
// scan_context.hpp
class ScanContext {
 public:
  explicit ScanContext(const ScanContextParams& params);
  Eigen::MatrixXd describe(const PointCloudT& cloud) const;   // rings x sectors，max z，空格 0
  Eigen::VectorXd ringKey(const Eigen::MatrixXd& descriptor) const;
  double ringKeyDistance(const Eigen::VectorXd& a, const Eigen::VectorXd& b) const;
  double distance(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, int* best_shift = nullptr) const;
};

// loop_detector.hpp
class LoopDetector {
 public:
  LoopDetector(const ScanContextParams&, const IcpParams&);
  std::optional<LoopConstraint> detect(const Keyframe& current,
                                       const std::vector<Keyframe>& history) const;
};

// pose_graph.hpp
class PoseGraph {
 public:
  explicit PoseGraph(const GraphParams& params);
  bool optimize(std::vector<Keyframe>& keyframes,
                const std::vector<LoopConstraint>& loops,
                std::vector<bool>* loop_active = nullptr) const;
};

// keyframe_manager.hpp（M1 可改私有部分）
class KeyframeManager {
 public:
  explicit KeyframeManager(const KeyframeParams&);
  void addFrame(double stamp, const Eigen::Isometry3d& pose_raw, const PointCloudPtr& cloud_body);
  bool shouldCreate(double stamp, const Eigen::Isometry3d& pose_raw) const;
  bool buildNextKeyframe(Keyframe* out);
  void reset();
};
```

### 语义约定

- `Keyframe.pose_raw = T_camera_init_body`；`pose_opt = T_map_body`。
- `LoopConstraint.T_from_to`：把 `to` 帧的点变到 `from` 帧，
  即 `p_from = T_from_to * p_to`；等价于 Ceres 的 `Z_ij = T_i^{-1} T_j`（i=from, j=to）。
- GICP 用 PCL `pcl::GeneralizedIterativeClosestPoint`：source=current，target=history，
  `getFinalTransformation()` 即 `T_from_to`。
- `Map` 原点 = 第 0 个关键帧的 `pose_raw`（PoseGraph 固定首节点），因此初始
  `map->camera_init` 为单位阵。

## 4. 各模块做完的定义（DoD）

### M1 backend-core
- 节点名/可执行名：`point_lio_backend` / `point_lio_backend_node`；参数全部从 yaml 声明。
- 订阅、配对（|Δt| < `runtime.stamp_tolerance_sec`）、关键帧、调用 M2/M3、发布全部输出。
- 20 Hz 定时器发布 TF `map->camera_init`；纠偏平滑 `output.correction_tau`。
- 纠偏目标：用最近优化关键帧 + 前端相对运动推出 `T_map_body`，再合成
  `T_map_camera_init = T_map_body * T_camera_init_body^{-1}`。
- 独立 worker 线程；回调不阻塞；队列上限 `runtime.pending_clouds`；跳变 > `reset_on_jump_m` 时清图重置。
- 单测：关键帧三种触发 + 硬下限 + max_keyframes。

### M2 loop
- `describe`：ring/sector 分桶，取桶内 max z；ring_key = 每环均值。
- `distance`：扇区循环移位，逐环余弦距离（跳过全零环），取最小；返回 best_shift。
- `detect`：索引差/时间差过滤 → ring-key 粗筛取 top_candidates → SC 距离阈值 →
  GICP 校验（max_correspondence、max_iterations、fitness、inlier_ratio、修正量门限）→ 返回约束。
- 单测：合成 360° 房间点云绕 z 旋转 37° 后距离 < 0.35、航向误差 < 5°。

### M3 graph
- 节点 7 维（平移 + 四元数，EigenQuaternionManifold），首节点固定。
- 相邻关键帧 odom 边（pose_raw 相对位姿）+ 回环边；Huber；SPARSE_NORMAL_CHOLESKY。
- 优化后残差门控删外点并重优化（最多 2 轮）；失败时保留上一版并返回 false。
- 单测：12 节点、5% 尺度漂移 + 航向偏置 + 1 条真回环，末点误差 < 0.05 m；
  再注入 1 条错误回环，验证被剔除。

## 5. 构建与测试（每个 worktree 内）

```bash
source /opt/ros/jazzy/setup.bash
PATH=/usr/bin:$PATH colcon build --packages-select point_lio_backend --cmake-args -DCMAKE_BUILD_TYPE=Release
PATH=/usr/bin:$PATH colcon test --packages-select point_lio_backend --event-handlers console_direct+
PATH=/usr/bin:$PATH colcon test-result --verbose
```

- 任何运行期测试用独立 DDS 域：`export ROS_DOMAIN_ID=77`。
- 默认域里有一个用户在跑的 `mid360_driver_node`，**不要 kill 它**。
- 不要动 `PCD/scans.pcd`（可能正被用户建图覆盖）；需要时复制到 /tmp。

## 6. 提交与汇报

- 在自己的分支（`feat/backend-core` / `feat/backend-loop` / `feat/backend-graph`）提交。
- 汇报格式：分支名、commit hash、改了哪些文件、build 结果、test 结果（贴关键输出）、
  遗留问题/接口变更请求。
- 不要 merge 到 feat/backend-posegraph；由 Lead 统一合并。
