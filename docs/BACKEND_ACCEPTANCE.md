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

PLACEHOLDER_TABLE

## 5. 已知限制与后续

- 真实雷达验收需要人工绕场（见 README「验收」），本机 CI 只覆盖合成回放。
- 重复走廊/对称场景仍可能误检：SC 阈值 + GICP + 修正量门限 + Huber + 残差门控是
  四道闸，仍建议现场看 `/backend/loop_markers`。
- 漂移 > 5 m 时宁可不闭环（安全失败），不会把地图拉歪。
- 现有 `PCD/scans.pcd` 无法事后加回环（没有逐帧位姿），只能建图时在线跑。
