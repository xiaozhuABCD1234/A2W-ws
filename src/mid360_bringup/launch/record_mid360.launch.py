"""Mid-360 (+D435i) 数据录制。

    ros2 launch mid360_bringup record_mid360.launch.py
    ros2 launch mid360_bringup record_mid360.launch.py camera:=false          # 只录雷达
    ros2 launch mid360_bringup record_mid360.launch.py out:=bags/my_bag       # 自定义输出名
    ros2 launch mid360_bringup record_mid360.launch.py camera_wait:=20.0      # 相机初始化等待

行为:
- lsusb 检测到 D435i(或显式 camera:=true)时,先起相机节点
  (enable_gyro/accel + unite_imu_method:=2,与开发机用法一致),
  等 camera_wait 秒再启动录制器 —— 给相机初始化留时间,保证 bag 开头就有 RGB。
- 没检测到硬件(或 camera:=false)时只录 /livox/lidar /livox/imu,立即开录。
- 录制格式 mcap + zstd(与 justfile record 一致);Ctrl+C 由 launch 统一收尾:
  rosbag2 优雅写盘,相机节点随 launch 退出。
"""

import subprocess
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

TOPIC_LIDAR = '/livox/lidar'
TOPIC_IMU = '/livox/imu'
TOPIC_COLOR = '/camera/camera/color/image_raw'
TOPIC_CAMERA_INFO = '/camera/camera/color/camera_info'


def _camera_connected() -> bool:
    try:
        out = subprocess.run(['lsusb'], capture_output=True, text=True,
                             timeout=5).stdout.lower()
        return '8086:0b3a' in out or 'realsense' in out
    except Exception:
        return False


def _setup(context, *args, **kwargs):
    camera = LaunchConfiguration('camera').perform(context)
    camera_wait = float(LaunchConfiguration('camera_wait').perform(context))
    out = LaunchConfiguration('out').perform(context) or f'bag_{datetime.now():%Y%m%d_%H%M%S}'

    use_camera = camera == 'true' or (camera == 'auto' and _camera_connected())
    topics = [TOPIC_LIDAR, TOPIC_IMU]
    actions = []

    if use_camera:
        topics += [TOPIC_COLOR, TOPIC_CAMERA_INFO]
        delay = camera_wait
        actions.append(LogInfo(msg=f'D435i 已连接: 起相机节点, {delay:.0f}s 后开录'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
            launch_arguments={
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',
            }.items(),
        ))
    else:
        delay = 0.5
        actions.append(LogInfo(msg='未检测到 D435i(camera=auto),仅录制雷达'))

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '--topics', *topics,
             '-s', 'mcap',
             # zstd_fast = mcap 分块压缩:写入时就地压,关闭零成本。
             # 不能用 --compression-mode file:那是对整个文件关时压缩,
             # RGB bag 上 GB,NX 要压几分钟,Ctrl+C 后长时间静默还可能被 launch 强杀坏 bag。
             '--storage-preset-profile', 'zstd_fast',
             '--max-cache-size', '0',
             '-o', out],
        output='screen',
        # 分块压缩下收尾应秒级完成;宽限放宽只为兜底
        # (值须为字符串 —— launch 的 kwarg 走 substitution,传 float 会 'not iterable')
        sigterm_timeout='15.0',
        sigkill_timeout='15.0',
    )
    actions.append(TimerAction(period=delay, actions=[recorder]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='auto',
                              description='true/false/auto:是否带 D435i 录制'),
        DeclareLaunchArgument('camera_wait', default_value='12.0',
                              description='相机初始化等待秒数(到点即开录,RGB 落后则截掉开头)'),
        DeclareLaunchArgument('out', default_value='',
                              description='bag 输出名,默认 bag_<时间戳>(相对当前目录)'),
        OpaqueFunction(function=_setup),
    ])
