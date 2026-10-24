#!/usr/bin/env python3
"""point_lio_backend 合成回放测试（不依赖真实雷达）。

做法：
  1) 从 PCD/scans.pcd 快照读一张真实场景地图当"世界"；
  2) 在稠密区里生成矩形回环（或直线）真值轨迹；
  3) 每个位姿从地图裁 15 m 内的点、转到机体系，当成 /cloud_registered_body 发布；
  4) 对真值增量注入尺度/航向漂移，当成 /aft_mapped_to_init 发布；
  5) 起后端节点，收 /backend/path 与 /backend/status，算优化前后 ATE，退出码即结论。

用法（先 source /opt/ros/jazzy/setup.bash 与 install/setup.bash）：
  export ROS_DOMAIN_ID=77
  python3 src/point_lio_backend/test/integration/synthetic_loop_replay.py --mode loop
  python3 ... --mode straight
"""

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry, Path
from diagnostic_msgs.msg import DiagnosticArray

# ── PCD 读取 ───────────────────────────────────────────────────────────────

def load_pcd_xyz(path):
    header = {}
    with open(path, 'rb') as f:
        while True:
            raw = f.readline()
            if not raw:
                raise RuntimeError('PCD 头不完整: ' + path)
            line = raw.decode('ascii', 'ignore').strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            header[parts[0]] = parts[1:]
            if parts[0] == 'DATA':
                data_kind = parts[1]
                break
        payload = f.read()

    if data_kind != 'binary':
        raise RuntimeError('只支持 binary PCD，实际是 ' + data_kind)
    fields = header['FIELDS']
    sizes = [int(x) for x in header['SIZE']]
    types = header['TYPE']
    counts = [int(x) for x in header.get('COUNT', ['1'] * len(fields))]
    if not all(s == 4 and c == 1 for s, c in zip(sizes, counts)):
        raise RuntimeError('只支持 4 字节单值字段的 PCD')
    type_map = {'F': np.float32, 'I': np.int32, 'U': np.uint32}
    dtype = np.dtype([(f, type_map[t]) for f, t in zip(fields, types)])
    data = np.frombuffer(payload, dtype=dtype)
    n = int(header['POINTS'][0])
    if data.size < n:
        raise RuntimeError(f'PCD 点数不足: {data.size} < {n}')
    xyz = np.stack([data['x'], data['y'], data['z']], axis=1).astype(np.float64)
    return xyz[:n]


# ── 轨迹生成 ───────────────────────────────────────────────────────────────

def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def make_pose(x, y, yaw, z=0.0):
    T = np.eye(4)
    T[:3, :3] = rot_z(yaw)
    T[:3, 3] = [x, y, z]
    return T


def generate_loop_path(cx, cy, size, speed, dt, turn_time=1.0):
    """矩形回环：四边直行 + 每个角原地转 90°。返回 [(stamp_offset, T_world_body)]。"""
    half = size * 0.5
    corners = [(cx - half, cy - half), (cx + half, cy - half),
               (cx + half, cy + half), (cx - half, cy + half)]
    poses = []
    t = 0.0
    for i in range(4):
        x0, y0 = corners[i]
        x1, y1 = corners[(i + 1) % 4]
        yaw = math.atan2(y1 - y0, x1 - x0)
        length = math.hypot(x1 - x0, y1 - y0)
        n = max(2, int(round(length / (speed * dt))))
        for k in range(n):
            s = k / n
            poses.append((t, make_pose(x0 + (x1 - x0) * s, y0 + (y1 - y0) * s, yaw)))
            t += dt
        # 原地转 90°（最后一圈仍转，保持角点处的观测）
        n_turn = max(1, int(round(turn_time / dt)))
        yaw1 = yaw + math.pi / 2
        for k in range(1, n_turn + 1):
            a = k / n_turn
            poses.append((t, make_pose(x1, y1, yaw + (yaw1 - yaw) * a)))
            t += dt
    return poses


def generate_straight_path(cx, cy, length, speed, dt):
    poses = []
    t = 0.0
    x0, x1 = cx - length * 0.5, cx + length * 0.5
    n = max(2, int(round(length / (speed * dt))))
    for k in range(n):
        s = k / n
        poses.append((t, make_pose(x0 + (x1 - x0) * s, cy, 0.0)))
        t += dt
    return poses


def distort_odometry(gt_poses, scale=1.05, yaw_bias_deg_per_m=0.5):
    """把真值增量加上尺度误差和航向偏置，模拟纯 LIO 的累积漂移。"""
    raw = [gt_poses[0].copy()]
    bias = math.radians(yaw_bias_deg_per_m)
    for prev, cur in zip(gt_poses[:-1], gt_poses[1:]):
        rel = np.linalg.inv(prev) @ cur
        t = rel[:3, 3] * scale
        yaw = math.atan2(rel[1, 0], rel[0, 0])
        dist = float(np.linalg.norm(rel[:3, 3]))
        rel2 = np.eye(4)
        rel2[:3, :3] = rot_z(yaw + bias * dist)
        rel2[:3, 3] = t
        raw.append(raw[-1] @ rel2)
    return raw


# ── ROS 发布/订阅 ──────────────────────────────────────────────────────────

def xyz_to_pointcloud2(points, frame_id, stamp):
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    arr = np.zeros((points.shape[0], 4), dtype=np.float32)
    arr[:, :3] = points.astype(np.float32)
    arr[:, 3] = 100.0
    msg.data = arr.tobytes()
    return msg


class Harness(Node):
    def __init__(self):
        super().__init__('synthetic_loop_replay')
        qos = QoSProfile(depth=10)
        self.pub_odom = self.create_publisher(Odometry, '/aft_mapped_to_init', qos)
        self.pub_cloud = self.create_publisher(PointCloud2, '/cloud_registered_body', qos)
        self.path = None
        self.status = {}
        self.loop_count = 0
        self.create_subscription(Path, '/backend/path', self._on_path, qos)
        self.create_subscription(DiagnosticArray, '/backend/status', self._on_status, qos)
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DiagnosticArray, '/backend/status', self._on_status, latched)

    def _on_path(self, msg):
        self.path = msg

    def _on_status(self, msg):
        for st in msg.status:
            for kv in st.values:
                self.status[kv.key] = kv.value
        try:
            self.loop_count = max(self.loop_count, int(float(self.status.get('loops', 0))))
        except ValueError:
            pass

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def publish_frame(node, stamp, raw_pose, gt_pose, world_pts, tree, args):
    # 用真值位置取可见世界点（漂移只体现在 odom 里，扫描本身是真实观测）
    idx = tree.query_ball_point(gt_pose[:3, 3], args.range_m, workers=-1)
    if len(idx) < 100:
        return
    pts = world_pts[np.asarray(idx)]
    # 真值位姿下转到机体系（传感器观测与真值一致；漂移只体现在 odom）
    rel = pts - gt_pose[:3, 3]
    body = rel @ gt_pose[:3, :3]  # R^T * (p - t)
    if body.shape[0] > args.max_points:
        pick = np.random.default_rng(1234 + int(stamp.sec * 10 + stamp.nanosec // 100000000)).choice(
            body.shape[0], args.max_points, replace=False)
        body = body[pick]
    body = body + np.random.default_rng(int(stamp.nanosec)).normal(0.0, 0.01, body.shape)

    cloud = xyz_to_pointcloud2(body, 'body', stamp)
    node.pub_cloud.publish(cloud)

    odom = Odometry()
    odom.header.frame_id = 'camera_init'
    odom.child_frame_id = 'body'
    odom.header.stamp = stamp
    odom.pose.pose.position.x = float(raw_pose[0, 3])
    odom.pose.pose.position.y = float(raw_pose[1, 3])
    odom.pose.pose.position.z = float(raw_pose[2, 3])
    q = quat_from_rot(raw_pose[:3, :3])
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]
    node.pub_odom.publish(odom)


def quat_from_rot(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def pose_from_msg(p):
    q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    q /= np.linalg.norm(q)
    x, y, z, w = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


# ── 主流程 ─────────────────────────────────────────────────────────────────

def find_ws(start):
    d = os.path.abspath(start)
    while d != '/':
        if os.path.isfile(os.path.join(d, 'src', 'point_lio_backend', 'package.xml')):
            return d
        d = os.path.dirname(d)
    return None


def start_backend(ws, params, log_path):
    env = os.environ.copy()
    env.setdefault('ROS_DOMAIN_ID', '77')
    cmd = ['ros2', 'run', 'point_lio_backend', 'point_lio_backend_node',
           '--ros-args', '--params-file', params]
    log = open(log_path, 'wb')
    proc = subprocess.Popen(cmd, cwd=ws, env=env, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    return proc, log


def stop_backend(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['loop', 'straight'], default='loop')
    parser.add_argument('--pcd', default=None)
    parser.add_argument('--backend-params', default=None)
    parser.add_argument('--speed', type=float, default=1.2)
    parser.add_argument('--loop-size', type=float, default=7.0)
    parser.add_argument('--center-x', type=float, default=-6.5)
    parser.add_argument('--center-y', type=float, default=-4.0)
    parser.add_argument('--straight-length', type=float, default=8.0)
    parser.add_argument('--range-m', type=float, default=15.0)
    parser.add_argument('--max-points', type=int, default=4000)
    parser.add_argument('--keep-backend', action='store_true')
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()

    ws = find_ws(os.path.dirname(os.path.abspath(__file__)))
    if not ws:
        raise SystemExit('找不到工作空间（src/point_lio_backend/package.xml）')
    pcd = args.pcd or os.path.join(ws, 'PCD', 'scans.pcd')
    params = args.backend_params or os.path.join(
        ws, 'src', 'point_lio_backend', 'test', 'integration', 'backend_test.yaml')
    if not os.path.isfile(pcd):
        raise SystemExit('找不到 PCD: ' + pcd)
    if not os.path.isfile(params):
        raise SystemExit('找不到测试参数: ' + params)

    os.environ.setdefault('ROS_DOMAIN_ID', '77')
    print(f'[replay] ws={ws} pcd={pcd} mode={args.mode} domain={os.environ["ROS_DOMAIN_ID"]}')

    # 快照，避免用户在建图时覆盖
    snap = os.path.join(tempfile.gettempdir(), 'backend_replay_map.pcd')
    if not args.pcd:
        print('[replay] 快照 PCD ->', snap)
        shutil.copyfile(pcd, snap)
        pcd = snap
    xyz = load_pcd_xyz(pcd)
    mask = np.abs(xyz[:, 2]) < 3.0
    xyz = xyz[mask]
    print(f'[replay] PCD points(|z|<3)={len(xyz)}')

    gt = (generate_loop_path(args.center_x, args.center_y, args.loop_size,
                             args.speed, 0.1)
          if args.mode == 'loop'
          else generate_straight_path(args.center_x, args.center_y, args.straight_length,
                                      args.speed, 0.1))
    gt_poses = [p for _, p in gt]
    raw_poses = distort_odometry(gt_poses)
    print(f'[replay] frames={len(gt_poses)} duration={len(gt_poses)*0.1:.1f}s')

    # 只需要轨迹附近（range + 2 m）的点。
    cx = np.mean([p[0, 3] for p in gt_poses])
    cy = np.mean([p[1, 3] for p in gt_poses])
    ext = max(np.max(np.abs([p[0, 3] - cx for p in gt_poses])),
              np.max(np.abs([p[1, 3] - cy for p in gt_poses]))) + args.range_m + 2.0
    crop = xyz[(np.abs(xyz[:, 0] - cx) < ext) & (np.abs(xyz[:, 1] - cy) < ext)]
    print(f'[replay] cropped world points={len(crop)}')

    from scipy.spatial import cKDTree
    print('[replay] building KD-tree ...')
    tree = cKDTree(crop)

    # 起后端
    log_path = os.path.join(tempfile.gettempdir(), 'backend_replay_node.log')
    proc, log = start_backend(ws, params, log_path)

    rclpy.init()
    node = Harness()
    fail_reason = None
    metrics = {}
    try:
        # 等节点起来（收到 status 或超时）
        t0 = time.time()
        while time.time() - t0 < 10 and not node.status:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.status:
            fail_reason = '后端 10 s 内没有发布 /backend/status'
        else:
            print('[replay] backend ready:', node.status)

        if fail_reason is None:
            start_wall = time.time()
            stamp_map = {}
            raw_map = {}
            gt_map = {}
            for i, (offset, gt_pose) in enumerate(gt):
                stamp = node.get_clock().now().to_msg()
                # 用稳定递增的时间戳（避免 0.1 s 抖动影响配对）
                stamp.sec = int(start_wall) + (i // 10)
                stamp.nanosec = (i % 10) * 100_000_000
                publish_frame(node, stamp, raw_poses[i], gt_pose, crop, tree, args)
                stamp_map[(stamp.sec, stamp.nanosec)] = stamp
                raw_map[(stamp.sec, stamp.nanosec)] = raw_poses[i]
                gt_map[(stamp.sec, stamp.nanosec)] = gt_pose
                rclpy.spin_once(node, timeout_sec=0)
                # 按 10 Hz 节奏
                target = start_wall + (i + 1) * 0.1
                sleep = target - time.time()
                if sleep > 0:
                    time.sleep(sleep)
            print('[replay] replay done, waiting for backend ...')

            # 等回环出现（loop 模式）或确认没有回环（straight 模式）
            deadline = time.time() + 20.0
            while time.time() < deadline:
                node.spin(0.2)
                if args.mode == 'loop' and node.loop_count > 0 and node.path is not None:
                    break
            node.spin(3.0)  # 让优化后的 path 发出来

        if fail_reason is None:
            if node.path is None:
                fail_reason = '没有收到 /backend/path'
            else:
                opt, raw, ref = [], [], []
                for ps in node.path.poses:
                    key = (ps.header.stamp.sec, ps.header.stamp.nanosec)
                    if key not in gt_map:
                        continue
                    opt.append(pose_from_msg(ps.pose)[:3, 3])
                    raw.append(raw_map[key][:3, 3])
                    ref.append(gt_map[key][:3, 3])
                if len(opt) < 5:
                    fail_reason = f'/backend/path 可用位姿太少({len(opt)})'
                else:
                    opt = np.asarray(opt); raw = np.asarray(raw); ref = np.asarray(ref)
                    ate_raw = float(np.sqrt(np.mean(np.sum((raw - ref) ** 2, axis=1))))
                    ate_opt = float(np.sqrt(np.mean(np.sum((opt - ref) ** 2, axis=1))))
                    final_raw = float(np.linalg.norm(raw[-1] - ref[-1]))
                    final_opt = float(np.linalg.norm(opt[-1] - ref[-1]))
                    metrics = dict(frames=len(opt), loops=node.loop_count,
                                   ate_raw=round(ate_raw, 4), ate_opt=round(ate_opt, 4),
                                   final_raw=round(final_raw, 4), final_opt=round(final_opt, 4))
                    print('[replay] metrics:', json.dumps(metrics))
                    if args.mode == 'loop':
                        if node.loop_count < 1:
                            fail_reason = '没有检出回环'
                        elif ate_opt > 0.5 * ate_raw:
                            fail_reason = f'ATE 改善不足: raw={ate_raw:.3f} opt={ate_opt:.3f}'
                        elif final_opt >= 0.5:
                            fail_reason = f'末点误差过大: {final_opt:.3f} m'
                    else:
                        if node.loop_count != 0:
                            fail_reason = f'直线场景误检回环 {node.loop_count} 次'
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
        if not args.keep_backend:
            stop_backend(proc)
            log.close()

    if fail_reason:
        print('\n[replay] FAIL:', fail_reason)
        try:
            with open(log_path, 'rb') as f:
                tail = f.read()[-4000:].decode('utf-8', 'replace')
            print('--- backend log tail ---\n' + tail)
        except OSError:
            pass
        return 1
    print('[replay] PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
