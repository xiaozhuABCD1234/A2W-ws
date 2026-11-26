import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _clean_ld_library_path():
    """LOCAL PATCH P11:剔除 /opt/MVS 的旧 libusb(缺 libusb_set_option,会让 PCL 启动即 symbol lookup error)。
    只作用于 laserMapping 进程。
    """
    raw = os.environ.get('LD_LIBRARY_PATH', '')
    paths = [p for p in raw.split(':') if p]
    kept = [p for p in paths if '/opt/MVS' not in p]
    if len(kept) == len(paths):
        return {}
    print('[mapping_mid360] LOCAL PATCH P11: LD_LIBRARY_PATH 含 /opt/MVS'
          '(旧 libusb 会破坏 PCL),已为 laserMapping 剔除该路径')
    return {'LD_LIBRARY_PATH': ':'.join(kept) if kept
            else '/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu'}


def _laser_mapping_node(context):
    """组装 laserMapping 节点(参数约定 A:yaml 优先,只有显式传参才覆盖)。

    LOCAL PATCH P7:新增 launch 参数 `pcd_save`,
    用来在**不改 yaml、不重新 colcon build** 的前提下开关 PCD 落盘。

    为什么非得用 OpaqueFunction:节点 parameters 列表里排在后面的项覆盖
    前面的,所以「无条件把 pcd_save.pcd_save_en 塞进字典」= 把 yaml 顶掉。
    default_value='' 表示「没传」,求值为空串 → 不追加覆盖项。
    """
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config', 'mid360.yaml'
        ]).perform(context),
        # LOCAL PATCH P5:此字典排在 params_file 后,会覆盖 mid360.yaml 同名参数(改这里才生效)。
        # filter_size_* 不在此处,yaml 是唯一真相源。
        {
            'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 3,  # Options: 1, 3
            'space_down_sample': True,
            # filter_size_surf / filter_size_map:见 config/mid360.yaml(唯一真相源)
            'cube_side_length': 1000.0,  # Option: 1000
            'runtime_pos_log_enable': False,  # Option: True
        }
    ]

    # LOCAL PATCH P7:只有显式传了 pcd_save 才覆盖 yaml。
    raw = context.perform_substitution(LaunchConfiguration('pcd_save')).strip().lower()
    if raw:
        if raw not in ('true', 'false', '1', '0', 'yes', 'no', 'on', 'off'):
            # 提前报错:塞进 bool 参数的 InvalidParameterTypeException 看不出来源
            raise RuntimeError(f"pcd_save 只接受 true/false,收到: {raw!r}")
        laser_mapping_params.append(
            {'pcd_save': {'pcd_save_en': raw in ('true', '1', 'yes', 'on')}})

    sim = context.perform_substitution(
        LaunchConfiguration('use_sim_time')).strip().lower()
    laser_mapping_params.append({'use_sim_time': sim in ('true', '1', 'yes', 'on')})

    # LOCAL PATCH P11:清理 LD_LIBRARY_PATH(见 _clean_ld_library_path)
    extra_env = _clean_ld_library_path()

    return [Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        additional_env=extra_env,
        # prefix='gdb -ex run --args'
    )]


def generate_launch_description():
    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.')

    # LOCAL PATCH P7:建图点云落盘开关(先验地图用)
    pcd_save_arg = DeclareLaunchArgument(
        'pcd_save', default_value='',
        description='是否把建图点云落盘。留空 = 用 config/mid360.yaml 的 pcd_save.pcd_save_en;'
                    '显式传 true/false 才覆盖。注意:只在节点【正常退出】(Ctrl+C)时写一次,'
                    '写到 ./PCD/scans.pcd(相对运行本 launch 的目录,P8)')

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        parameters=[{'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool)}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    # Assemble the launch description
    ld = LaunchDescription([
        rviz_arg,
        pcd_save_arg,
        use_sim_time_arg,
        OpaqueFunction(function=_laser_mapping_node),
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

    return ld
