# Point-LIO 后端优化（回环 + 位姿图）计划

## 目标

给现有 Mid-360 + Point-LIO 建图链路加在线后端：回环检测 + 位姿图优化，消除长距离
漂移和 octomap 重影。**不修改 vendored point_lio / mid360_driver 源码**，通过新包
`point_lio_backend` 订阅前端话题实现；`backend:=false` 可完全退回现状。

## 范围

- 做：关键帧与子图、Scan Context 回环检测、GICP 校验、Ceres 6-DOF 位姿图、
  map->camera_init 纠偏 TF、优化轨迹/点云输出、octomap 接入 map 系、合成回放测试。
- 不做：多会话地图合并、重定位（LOCALIZATION.md 的 P1/AMCL 路线）、GPS/IMU 因子、
  离线重算现有 scans.pcd（没有逐帧位姿，原理上做不到）。

## 架构

    map --(backend 纠偏, 20 Hz)--> camera_init --(point_lio)--> body

后端是独立进程：输入 /aft_mapped_to_init 与 /cloud_registered_body，输出
/backend/path、/backend/odom、/backend/map_points、/backend/loop_markers、
/backend/status 与 TF map->camera_init。

## 算法要点

1. 关键帧：平移 > 1.0 m 或航向 > 10° 或间隔 > 2.0 s；硬下限 0.3 s；
   子图 = 最近 4 帧机体系点云按相对位姿拼接，0.2 m 体素降采样。
2. Scan Context：20 环 × 60 扇区、半径 40 m、每格 max z；ring key 粗筛
   （阈值 0.15，取 5 个候选，索引差 ≥ 20、时间差 ≥ 10 s），扇区移位余弦距离精筛
   （阈值 0.35）。
3. GICP：source=当前子图，target=历史子图；初值取前端相对位姿；接受条件
   fitness < 0.10、inlier_ratio > 0.6、修正量 < 5 m / 30°。
4. 位姿图：7 维节点（平移 + 四元数），首节点固定；odom 边 σt=0.02 m/σr=0.5°，
   回环边 σt=0.10 m/σr=1.5°，Huber(1.0)；优化后残差门控删外点并重优化。
5. 纠偏：

$$
T^{map}_{body}(t)=T^{map}_{body,k}\,T^{camera\_init}_{body,k}{}^{-1}\,T^{camera\_init}_{body}(t)
$$

$$
T^{map}_{camera\_init}(t)=T^{map}_{body}(t)\,\left(T^{camera\_init}_{body}(t)\right)^{-1}
$$

   用一阶平滑（τ=1.5 s）逼近目标，避免回环瞬间跳变把 octomap 撕成两层。

## 验收

- colcon build + 3 组 gtest 全绿。
- 合成回环：PCD 快照生成 8×8 m 矩形 + 5% 尺度/0.5°/m 航向漂移，至少 1 个回环，
  ATE RMSE 下降 ≥ 50%，末点误差 < 0.5 m。
- 无回环场景 0 误检；后端平均 CPU < 0.8 核。
- 真实雷达：绕一圈后 /backend/path 闭合、octomap 无重影（人工验收）。
- backend:=false 行为与改造前一致。

## 集成

- mid360.launch.py 增加 backend 参数（默认 true），include 后端并让 octomap 用 map 系。
- octomap_mid360.launch.py 增加 frame_id 参数（空 = 用 yaml 的 camera_init）。
- 新增 rviz/backend_map.rviz（Fixed Frame: map）。
- preflight 增加后端配置自检；README / PATCHES 同步。
