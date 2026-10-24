#!/usr/bin/env python3
"""TF 时戳回归测试：模拟 point_lio 的发布顺序，验证 rviz 式的定点查询不会失败。

背景（真机实测）：
  point_lio **先发** /cloud_registered(frame=camera_init, stamp=T)，**后发**
  /aft_mapped_to_init(stamp=T)。后端的 map->camera_init 只由 odom 推出。
  如果 TF 也用 T 打戳，那么点云到达的瞬间 TF 缓冲最新只到 T-100ms，rviz 的
  message filter 会报
    "Lookup would require extrapolation into the future"
  并把状态栏刷成 Could not transform from [camera_init] to [map]。
  把 TF 改成墙钟打戳后，T 落在插值窗口内，一次命中。

本脚本：起一个假前端（cloud 先、odom 后，10 Hz）+ 后端 + 一个 rviz 式探针，
逐帧对 map <- camera_init 做**定点**查询，统计失败数。

用法：
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  export ROS_DOMAIN_ID=89
  python3 src/point_lio_backend/test/integration/tf_stamp_check.py
退出码 0 = 没有 future-extrapolation 失败。
"""

import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener
except ModuleNotFoundError:  # uv/venv 的 python 没有 rclpy → 换系统 python 重跑
    if os.path.realpath(sys.executable) != '/usr/bin/python3':
        os.execv('/usr/bin/python3', ['/usr/bin/python3'] + sys.argv)
    raise


def make_cloud(n, stamp):
    msg = PointCloud2()
    msg.header.frame_id = 'camera_init'
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    rng = np.random.default_rng(0)
    pts = rng.normal(0.0, 5.0, (n, 3))
    arr = np.zeros((n, 4), dtype=np.float32)
    arr[:, :3] = pts
    arr[:, 3] = 100.0
    msg.data = arr.tobytes()
    return msg


def main():
    domain = os.environ.setdefault('ROS_DOMAIN_ID', '89')
    ros_home = os.path.join(tempfile.gettempdir(), 'plio_tf_ros_home')
    os.makedirs(os.path.join(ros_home, 'log'), exist_ok=True)
    os.environ.setdefault('ROS_HOME', ros_home)
    os.environ.setdefault('ROS_LOG_DIR', os.path.join(ros_home, 'log'))
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')

    ws = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
    params = os.path.join(ws, 'src', 'point_lio_backend', 'config', 'backend_mid360.yaml')
    node_proc = subprocess.Popen(
        ['ros2', 'run', 'point_lio_backend', 'point_lio_backend_node',
         '--ros-args', '--params-file', params],
        stdout=open('/tmp/tf_stamp_check_backend.log', 'wb'),
        stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(3.0)

    rclpy.init()
    node = Node('tf_stamp_check')
    qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.VOLATILE)
    pub_cloud = node.create_publisher(PointCloud2, '/cloud_registered', qos)
    pub_odom = node.create_publisher(Odometry, '/aft_mapped_to_init', qos)

    buffer = Buffer()
    TransformListener(buffer, node)

    failures = []
    retried = []
    checked = 0

    pending = []  # [(到达时间, 首错, frame, stamp)]

    def on_cloud(msg):
        nonlocal checked
        checked += 1
        # 不能在这里阻塞等 TF:回调跑在单线程 executor 里,sleep 会让 TF 消息
        # 也收不进来,重试永远失败。这里只入队,由主循环边 spin 边重试
        # —— 这正是 rviz 的 message filter 的做法。
        pending.append([time.time(), None, msg.header.frame_id, msg.header.stamp])

    def drain_pending(timeout_sec=0.2):
        """对队列里每条消息做定点查询;超时的记失败。"""
        still = []
        now = time.time()
        for item in pending:
            arrived, first_error, frame, stamp = item
            try:
                buffer.lookup_transform('map', frame, stamp)
                if first_error is not None:
                    retried.append(first_error)
            except Exception as exc:  # noqa: BLE001
                if first_error is None:
                    first_error = str(exc)
                if now - arrived < timeout_sec:
                    still.append([arrived, first_error, frame, stamp])
                elif now - test_t0 < 1.5:
                    pass  # 预热期（后端还没发出第一帧 TF），不计失败
                else:
                    failures.append(first_error)
        pending[:] = still

    node.create_subscription(PointCloud2, '/cloud_registered', on_cloud, qos)

    clock = node.get_clock()
    frames = 60
    start = time.time()
    test_t0 = start
    try:
        for i in range(frames):
            stamp = clock.now().to_msg()
            # 模拟 point_lio：先点云、后 odom（同一 stamp）
            pub_cloud.publish(make_cloud(200, stamp))
            rclpy.spin_once(node, timeout_sec=0.0)
            odom = Odometry()
            odom.header.frame_id = 'camera_init'
            odom.child_frame_id = 'body'
            odom.header.stamp = stamp
            odom.pose.pose.orientation.w = 1.0
            odom.pose.pose.position.x = 0.01 * math.sin(i * 0.1)
            pub_odom.publish(odom)

            end = start + (i + 1) * 0.1
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
                drain_pending()
        # 收尾：多转一会儿把队列里的消息处理完
        tail = time.time() + 3.0
        while time.time() < tail:
            rclpy.spin_once(node, timeout_sec=0.02)
            drain_pending()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(os.getpgid(node_proc.pid), 15)
            node_proc.wait(timeout=8)
        except Exception:  # noqa: BLE001
            try:
                os.killpg(os.getpgid(node_proc.pid), 9)
            except Exception:  # noqa: BLE001
                pass

    print(f'domain={domain} frames={frames} cloud_msgs={checked} '
          f'lookup_failures={len(failures)} retried_ok={len(retried)}')
    for f in failures[:5]:
        print('  FAIL:', f)
    if checked < frames // 2:
        print('FAIL: 收到的点云太少,测试没跑起来')
        return 1
    if failures:
        print('FAIL: 存在定点查询失败(见上);TF 时戳应改成墙钟')
        return 1
    print('PASS: 每帧 map <- camera_init 定点查询都成功')
    return 0


if __name__ == '__main__':
    sys.exit(main())
