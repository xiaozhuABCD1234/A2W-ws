# 后端优化验收记录（point_lio_backend）

本文件记录 `feat/backend-posegraph` 的实测证据。计划与契约见 `BACKEND_PLAN.md` /
`BACKEND_SPEC.md`；用户文档见 `README.md` 的「后端优化」一节。

## 1. 范围

给 Mid-360 + Point-LIO 加在线后端：Scan Context 回环检测 + 两级 GICP 校验 +
Ceres 6-DOF 位姿图，输出 `map` 系纠偏 TF / 轨迹 / 点云。**不修改任何 vendored 源码**
（`point_lio_ros2` / `mid360_driver` 的 diff 仍只有 P1~P11 / M1~M2）。

## 2. 单元测试

```
PATH=/usr/bin:$PATH colcon test --packages-select point_lio_backend
```

| 测试 | 用例数 | 内容 |
|---|---|---|
| `test_scan_context` | 10 | SC 旋转不变性（37° → 距离 0.0073、航向误差 1.0°）、不同房间不匹配（1.147）、LoopDetector 合成回环与 1.5 m 漂移初值 |
| `test_pose_graph` | 5 | 12 节点 5% 尺度 + 0.5°/m 航向漂移闭合、3 m 错误回环剔除、id≠index、50 节点、500 节点耗时 |
| `test_keyframe_manager` | 8 | 距离/角度/时间触发、0.3 s 硬下限、max_keyframes、子图拼接、reset |

## 3. 合成回放（无雷达，用真实场景 PCD 当世界）

```
export ROS_DOMAIN_ID=8x
python3 src/point_lio_backend/test/integration/synthetic_loop_replay.py --mode loop --laps 2
python3 src/point_lio_backend/test/integration/synthetic_loop_replay.py --mode straight
```

见 §4 的表格（每次运行的实际数值）。

## 4. 实测数据（2026-10-24，本机 32 核）

### 4.1 单元测试

```
Summary: 25 tests, 0 errors, 0 failures, 0 skipped
```

### 4.2 合成回环（2 圈矩形 7×7 m，5% 尺度 + 0.5°/m 航向漂移）

| 指标 | 原始（纯 LIO） | 后端优化后 | 结论 |
|---|---|---|---|
| 回环数 | 0 | **91** | 检出 |
| ATE RMSE | 1.245 m | **0.317 m** | ↓ **74.5%**（验收线 ≥50%） |
| 末点误差 | 2.441 m | **0.060 m** | < 0.5 m（验收线） |

### 4.3 无回环场景（直线 8 m，同样漂移注入）

| 指标 | 数值 | 结论 |
|---|---|---|
| 回环数 | **0** | 无漏报之外的误检 |
| ATE | 0.254 m（优化前后相同） | 只在有回环时才动轨迹 |

### 4.4 性能（同一回放，152 个关键帧）

| 阶段 | 均值 | p50 | p95 | 上限预算 |
|---|---|---|---|---|
| 回环检测（SC + GICP） | 124 ms | 28 ms | 478 ms | `icp.time_budget_ms` = 400（生产）／2000（测试） |
| 话题发布（path/markers/map） | 13 ms | — | — | `output.map_publish_period` |
| 位姿图优化 | 2.2 ms | — | — | 91 次全部成功 |
| 进程 CPU | 17 % 单核 | 2 % | 48 % | 验收线 < 80 % |

关键:第 1 版两级 GICP 对 8 个候选 × 全分辨率子图（~5000 点）跑，单帧 0.7~3.3 s，
队列持续丢帧；改为「0.35 m 体素降到 ~1200 点 + 候选按 SC 距离升序 + 命中即返回 +
400 ms 预算」后降到 p50 28 ms，整段回放只丢 3 帧（旧版持续丢帧）。


## 5. 已知限制与后续

- 真实雷达验收需要人工绕场（见 README「验收」），本机 CI 只覆盖合成回放。
- 重复走廊/对称场景仍可能误检：SC 阈值 + GICP + 修正量门限 + Huber + 残差门控是
  四道闸，仍建议现场看 `/backend/loop_markers`。
- 漂移 > 5 m 时宁可不闭环（安全失败），不会把地图拉歪。
- 现有 `PCD/scans.pcd` 无法事后加回环（没有逐帧位姿），只能建图时在线跑。
