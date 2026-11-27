import os
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MOUNT_KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def _urdf(mount):
    share = get_package_share_directory('robot_model')
    source = os.path.join(share, 'urdf', 'warehouse_agv', 'warehouse_agv.urdf')
    if not os.path.exists(source):
        return ''
    if shutil.which('xacro') is None:
        print('[robot_model] xacro 未安装,跳过 robot_state_publisher 的 URDF', flush=True)
        return ''
    with tempfile.TemporaryDirectory(prefix='robot_model_urdf_') as directory:
        target = os.path.join(directory, 'robot.urdf')
        args = ['xacro', source, '-o', target]
        args += ['lidar_{}:={}'.format(key, mount[key]) for key in MOUNT_KEYS]
        try:
            subprocess.run(args, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as error:
            print('[robot_model] xacro 失败,回退到 robot_model 生成的描述: {}'.format(
                error.stderr.strip()), flush=True)
            return ''
        with open(target, 'r', encoding='utf-8') as stream:
            return stream.read()


def _nodes(context):
    config = os.path.join(get_package_share_directory('robot_model'), 'config', 'robot.yaml')
    sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() in (
        'true', '1', 'yes', 'on')

    mount = {}
    for key in MOUNT_KEYS:
        value = LaunchConfiguration('mount_' + key).perform(context).strip()
        try:
            mount[key] = float(value) if value else None
        except ValueError as error:
            raise RuntimeError(
                'mount_{} 必须是数字,收到 {!r}'.format(key, value)) from error
    defaults = {'x': 0.10, 'y': 0.0, 'z': 0.24, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    for key in MOUNT_KEYS:
        if mount[key] is None:
            mount[key] = defaults[key]

    description = _urdf(mount)
    parameters = [config, {'use_sim_time': sim_time}]
    parameters.append({'frames.lidar_' + key: mount[key] for key in MOUNT_KEYS})

    actions = []
    if description:
        parameters.append({'description.publish': False})
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': description, 'use_sim_time': sim_time}],
        ))
    actions.append(Node(
        package='robot_model',
        executable='robot_model_node',
        name='robot_model',
        output='screen',
        parameters=parameters,
    ))
    return actions


def generate_launch_description():
    args = [DeclareLaunchArgument('use_sim_time', default_value='false')]
    args += [DeclareLaunchArgument('mount_' + key, default_value='') for key in MOUNT_KEYS]
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
