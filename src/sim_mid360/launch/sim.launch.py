import os
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _float(context, name):
    return float(LaunchConfiguration(name).perform(context))


def _pattern_plugin(csv, samples_per_second, rate):
    return ('  <plugin filename="gz-sim-mid360-pattern-system"'
            ' name="sim_mid360::Mid360Pattern">\n'
            '    <parent_link>lidar_link</parent_link>\n'
            '    <csv_file>{csv}</csv_file>\n'
            '    <samples_per_second>{samples}</samples_per_second>\n'
            '    <update_rate>{rate}</update_rate>\n'
            '    <min_range>0.10</min_range>\n'
            '    <max_range>40.0</max_range>\n'
            '    <topic>/lidar</topic>\n'
            '    <frame_id>lidar_link</frame_id>\n'
            '  </plugin>\n').format(csv=csv, samples=int(samples_per_second), rate=rate)


def _generate_model(directory, mapping, csv, samples_per_second, rate):
    source = os.path.join(get_package_share_directory('robot_model'), 'urdf',
                          'warehouse_agv', 'warehouse_agv.urdf')
    if not os.path.exists(source):
        raise RuntimeError('找不到 URDF:{}'.format(source))
    for tool in ('xacro', 'gz'):
        if shutil.which(tool) is None:
            raise RuntimeError(
                '缺少 {} 命令,无法生成仿真模型(需要 ROS 2 + Gazebo Harmonic)'.format(tool))
    urdf_path = os.path.join(directory, 'warehouse_agv.urdf')
    args = ['xacro', source, '-o', urdf_path]
    args += ['{}:={}'.format(k, v) for k, v in mapping.items()]
    subprocess.run(args, check=True, capture_output=True, text=True)
    sdf_text = subprocess.run(['gz', 'sdf', '-p', urdf_path], check=True,
                              capture_output=True, text=True).stdout
    sdf_text = sdf_text.replace('<sdf version="1.12"', '<sdf version="1.11"')
    sdf_text = sdf_text.replace("<sdf version='1.12'", "<sdf version='1.11'")
    if csv and os.path.exists(csv):
        marker = "<model name='warehouse_agv'>"
        if marker in sdf_text:
            sdf_text = sdf_text.replace(
                marker, marker + '\n' + _pattern_plugin(csv, samples_per_second, rate), 1)
    target = os.path.join(directory, 'warehouse_agv')
    os.makedirs(target, exist_ok=True)
    with open(os.path.join(target, 'model.sdf'), 'w', encoding='utf-8') as stream:
        stream.write(sdf_text)
    with open(os.path.join(target, 'model.config'), 'w', encoding='utf-8') as stream:
        stream.write('<?xml version="1.0"?>\n<model>\n  <name>warehouse_agv</name>\n'
                     '  <version>1.0</version>\n  <sdf version="1.9">model.sdf</sdf>\n'
                     '</model>\n')
    return urdf_path


def _launch(context):
    share = get_package_share_directory('sim_mid360')
    mount = {
        'x': _float(context, 'mount_x'),
        'y': _float(context, 'mount_y'),
        'z': _float(context, 'mount_z'),
        'roll': _float(context, 'mount_roll'),
        'pitch': _float(context, 'mount_pitch'),
        'yaw': _float(context, 'mount_yaw'),
    }
    resource_dir = tempfile.mkdtemp(prefix='sim_mid360_models_')
    mapping = {
        'lidar_x': mount['x'], 'lidar_y': mount['y'], 'lidar_z': mount['z'],
        'lidar_roll': mount['roll'], 'lidar_pitch': mount['pitch'],
        'lidar_yaw': mount['yaw'],
    }
    csv = os.path.expanduser(LaunchConfiguration('pattern_csv').perform(context))
    try:
        samples = max(1, int(LaunchConfiguration('pattern_samples').perform(context)))
    except ValueError as error:
        raise RuntimeError('pattern_samples 必须是正整数') from error
    try:
        rate = float(LaunchConfiguration('update_rate').perform(context))
    except ValueError as error:
        raise RuntimeError('update_rate 必须是数字') from error
    if not (rate > 0.0):
        raise RuntimeError('update_rate 必须大于 0')
    if not os.path.isfile(csv):
        print('[sim_mid360] ⚠ pattern csv 不存在:{}'.format(csv), flush=True)
        print('[sim_mid360]   没有扫描模式文件时不会发布 /lidar 点云,仿真里将没有雷达数据。',
              flush=True)
    _generate_model(resource_dir, mapping, csv, samples, rate)

    headless = LaunchConfiguration('headless').perform(context)
    gz_share = '/usr/share/gz'
    env = dict(os.environ)
    robot_model_share = get_package_share_directory('robot_model')
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = '{}:{}'.format(
        os.path.join(share, '..', '..', 'lib'),
        env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''))
    env['GZ_SIM_RESOURCE_PATH'] = ':'.join([
        resource_dir,
        os.path.join(share, 'models'),
        os.path.dirname(robot_model_share),
        os.path.join(gz_share, 'gz-sim8'),
        os.path.join(gz_share, 'gz-sim'),
    ]) + ':' + env.get('GZ_SIM_RESOURCE_PATH', '')

    args = ['gz', 'sim', '-r']
    if headless in ('true', '1'):
        args.append('-s')
    args.append(os.path.join(share, 'world', 'room.sdf'))

    return [
        ExecuteProcess(cmd=args, output='screen', additional_env=env),
        Node(
            package='sim_mid360',
            executable='gz_bridge_node',
            name='sim_mid360_bridge',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'bridge.yaml'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'mount.roll': mount['roll'],
                    'mount.pitch': mount['pitch'],
                    'mount.yaw': mount['yaw'],
                    'mount.x': mount['x'],
                    'mount.y': mount['y'],
                    'mount.z': mount['z'],
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mount_x', default_value='0.10'),
        DeclareLaunchArgument('mount_y', default_value='0.0'),
        DeclareLaunchArgument('mount_z', default_value='0.24'),
        DeclareLaunchArgument('mount_roll', default_value='0.0'),
        DeclareLaunchArgument('mount_pitch', default_value='0.0'),
        DeclareLaunchArgument('mount_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'pattern_csv',
            default_value=os.path.join(
                os.path.expanduser('~'), 'Downloads', 'Mid360_simulation_plugin',
                'livox_laser_simulation', 'scan_mode', 'mid360-real-centr.csv')),
        DeclareLaunchArgument('pattern_samples', default_value='100000'),
        DeclareLaunchArgument('update_rate', default_value='10.0'),
        OpaqueFunction(function=_launch),
    ])
