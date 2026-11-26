"""Point-LIO 后端（回环 + 位姿图）单独启动。

一般不用直接跑：slam.launch.py 里 backend:=true 会自动 include 本文件。
单独调试：

    ros2 launch point_lio_backend backend_mid360.launch.py
    ros2 launch point_lio_backend backend_mid360.launch.py params_file:=/path/to/x.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _clean_ld_library_path():
    """剔除 /opt/MVS：其旧 libusb 会导致后端加载 PCL 失败。"""
    raw = os.environ.get('LD_LIBRARY_PATH', '')
    paths = [p for p in raw.split(':') if p]
    kept = [p for p in paths if '/opt/MVS' not in p]
    if len(kept) == len(paths):
        return {}
    print('[backend_mid360] /opt/MVS in LD_LIBRARY_PATH (old libusb breaks PCL), '
          'stripped for the backend node')
    return {'LD_LIBRARY_PATH': ':'.join(kept) if kept
            else '/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu'}


def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('point_lio_backend'), 'config', 'backend_mid360.yaml'
        ]),
        description='后端参数 yaml（唯一真相源）')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    save_pcd_arg = DeclareLaunchArgument('save_pcd', default_value='')

    def _backend_node(context):
        params = [
            LaunchConfiguration('params_file').perform(context),
            {'use_sim_time': context.perform_substitution(
                LaunchConfiguration('use_sim_time')).strip().lower() in ('1', 'true', 'yes', 'on')}
        ]
        save_pcd = context.perform_substitution(LaunchConfiguration('save_pcd')).strip().lower()
        if save_pcd:
            params.append({'output.save_pcd': save_pcd in ('1', 'true', 'yes', 'on')})
        return [Node(
            package='point_lio_backend',
            executable='point_lio_backend_node',
            name='point_lio_backend',
            output='screen',
            parameters=params,
            additional_env=_clean_ld_library_path(),
        )]

    return LaunchDescription([
        params_arg, use_sim_time_arg, save_pcd_arg, OpaqueFunction(function=_backend_node)
    ])
