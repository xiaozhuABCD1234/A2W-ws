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

### 4.2 合成回环 —— 用**生产默认配置**跑（最重要的一组）

2 圈 7×7 m 矩形，注入 5% 尺度 + 0.5°/m 航向漂移，帧密度按真实雷达（每帧约 1.9 万点、
20 m 视距）：

```bash
python3 src/point_lio_backend/test/integration/synthetic_loop_replay.py \
  --mode loop --laps 2 --max-points 19000 --range-m 20 \
  --backend-params src/point_lio_backend/config/backend_mid360.yaml
```

| 指标 | 原始（纯 LIO） | 后端优化后 | 验收线 | 结论 |
|---|---|---|---|---|
| 关键帧 | — | 81 | — | — |
| 回环数 | 0 | **40** | ≥1 | 检出 |
| ATE RMSE | 1.263 m | **0.358 m** | ↓ ≥50% | ↓ **71.6%** |
| 末点误差 | 2.441 m | **0.065 m** | < 0.5 m | 通过 |
| 进程 CPU | — | 均值 2.7%、峰值 21.8% 单核 | < 80% | 通过 |
| 队列丢帧 | — | **0** | 0 | 通过 |

### 4.3 合成回环 —— 更严苛的测试配置（关键帧 0.5 m、map 0.5 s 刷新）

| 指标 | 原始 | 优化后 | 结论 |
|---|---|---|---|
| 关键帧 / 回环 | — / 0 | 148 / **89** | 检出 |
| ATE RMSE | 1.214 m | **0.299 m** | ↓ 75.4% |
| 末点误差 | 2.441 m | **0.104 m** | 通过 |
| CPU | — | 均值 15.2%、峰值 44.7% | 通过 |

### 4.4 无回环场景（直线 8 m，同样漂移注入）

| 指标 | 数值 | 结论 |
|---|---|---|
| 回环数 | **0** | 无误检 |
| ATE | 0.254 m（优化前后完全相同） | 只在真有回环时才动轨迹 |

### 4.5 性能演进（每一条都是实测后改的）

| 版本 | detect 均值 / p50 / p95 | 发布耗时 | 队列丢帧 | CPU 峰值 |
|---|---|---|---|---|
| V1：8 候选 × 全分辨率子图（~5000 点）两级 GICP | 888 / 747 / 2570 ms | 16 ms | 持续丢帧 | — |
| V2：0.35 m 体素降到 ~1200 点 + 候选按 SC 距离升序 + 命中即返回 + 预算 | 124 / 28 / 478 ms | 13 ms | 3 | 52% |
| V3：地图改增量累积（每次只并入新关键帧）+ 纠偏量门限才整图重建 | 116 / 30 / 495 ms | 4（p50），全量重建 0.6~1.1 s 偶尔一次 | 5 | 45% |
| **生产默认配置**（关键帧 1 m、map 2 s 刷新） | — | — | **0** | **22%** |

位姿图优化本身：81~148 次调用，均值 2.2 ms、最大 3.7 ms（Ceres 不是瓶颈）。


## 5. 已知限制与后续

- 真实雷达验收需要人工绕场（见 README「验收」），本机 CI 只覆盖合成回放。
- 重复走廊/对称场景仍可能误检：SC 阈值 + GICP + 修正量门限 + Huber + 残差门控是
  四道闸，仍建议现场看 `/backend/loop_markers`。
- 漂移 > 5 m 时宁可不闭环（安全失败），不会把地图拉歪。
- 现有 `PCD/scans.pcd` 无法事后加回环（没有逐帧位姿），只能建图时在线跑。
