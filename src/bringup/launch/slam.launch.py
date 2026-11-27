"""Mid-360 全栈一键启动:驱动 + point_lio + octomap + rviz(单窗口)。

    ros2 launch bringup slam.launch.py
    ros2 launch bringup slam.launch.py rviz:=false
    ros2 launch bringup slam.launch.py pcd_save:=true      # 临时开 PCD 落盘(先验地图)
    ros2 launch bringup slam.launch.py backend:=false      # 关后端,octomap 退回 camera_init 系
    ros2 launch bringup slam.launch.py use_driver:=false use_sim_time:=true

point_lio 的 rviz 固定关掉,rviz 由本 launch 直接起。驱动为本地包
mid360_driver:UDP 收流直接发 /livox/lidar(PointCloud2)与 /livox/imu,
没有 livox_ros_driver2 的 xfer_format 选项。

"""

import glob
import os
import tempfile

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

OCTOMAP_OVERRIDABLE = {
    'resolution': ('resolution', float),
    'max_range': ('sensor_model.max_range', float),
}


def _has_package(name):
    try:
        get_package_prefix(name)
        return True
    except Exception:
        return False


def _octomap_preload():
    """返回可用的 liboctomap 路径;不同发行版/架构版本号不同,不能写死。"""
    for pattern in ('/usr/lib/*-linux-gnu/liboctomap.so.*',
                    '/usr/lib/liboctomap.so.*',
                    '/usr/local/lib/liboctomap.so.*'):
        for path in sorted(glob.glob(pattern)):
            if os.path.isfile(path):
                return path
    return ''


def _prepare_rviz_config(template_path, fixed_frame, drop_octomap_plugin):
    """按当前环境生成 rviz 配置,返回临时文件路径。

    1) Fixed Frame 在 backend 开/关时分别是 map/camera_init,复制一份替换,
       不改模板(避免污染仓库);
    2) 没装 octomap_rviz_plugins 时删掉对应 display,其余显示照常。
    """
    with open(template_path, encoding='utf-8') as f:
        lines = f.readlines()

    out = []
    skip_display = False
    for line in lines:
        stripped = line.strip()
        if skip_display:
            # display 块以 "- Class:" 开头,遇到下一个 "- Class:" 或顶层键结束
            if stripped.startswith('- Class:') or (
                    stripped and not line.startswith(' ' * 6)):
                skip_display = False
            else:
                continue
        if drop_octomap_plugin and stripped == '- Class: octomap_rviz_plugins/OccupancyGrid':
            skip_display = True
            continue
        if stripped.startswith('Fixed Frame:'):
            out.append(f'    Fixed Frame: {fixed_frame}\n')
            continue
        out.append(line)

    # 用确定性文件名,避免每次启动都在 /tmp 里堆一份(同一模板+世界系只保留一份)。
    import hashlib
    digest = hashlib.md5((''.join(out)).encode('utf-8')).hexdigest()[:10]
    path = os.path.join(tempfile.gettempdir(), f'mid360_rviz_{digest}.rviz')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.writelines(out)
    return path


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='启动 rviz2(单窗口:octomap 体素 + LIO 点云 + 2D 占据图 + path)')

    # P7:透传「是否落盘 PCD」给 point_lio。留空 = 用 yaml
    # (约定 A);显式传 true/false 才覆盖。launch 参数是唯一免 build 的开关。
    pcd_save_arg = DeclareLaunchArgument(
        'pcd_save', default_value='',
        description='是否把建图点云落盘(先验地图用)。留空 = 用 point_lio 的 yaml '
                    '(现为 false);显式 true/false 才覆盖。注意:只在节点【正常退出】'
                    '(Ctrl+C)时写一次,写到 ./PCD/scans.pcd(相对运行本 launch 的目录)')

    use_octomap_arg = DeclareLaunchArgument(
        'use_octomap', default_value='true',
        description='是否启动 octomap_server(3D 占据栅格)。需要系统包 ros-jazzy-octomap-server;'
                    '没装时本 launch 会自动跳过并提示装法,不影响驱动/point_lio/后端')

    resolution_arg = DeclareLaunchArgument('resolution', default_value='')
    max_range_arg = DeclareLaunchArgument('max_range', default_value='')

    backend_arg = DeclareLaunchArgument(
        'backend', default_value='true',
        description='是否启动 point_lio_backend(回环检测 + 位姿图)。true 时 octomap 与 rviz '
                    '切到 map 系;false 时退回改造前行为(camera_init 系)')

    relocalization_arg = DeclareLaunchArgument(
        'relocalization', default_value='true')

    backend_save_arg = DeclareLaunchArgument('backend_save', default_value='')

    ground_segmentation_arg = DeclareLaunchArgument(
        'ground_segmentation', default_value='true')

    filter_arg = DeclareLaunchArgument(
        'filter', default_value='true',
        description='是否启动 filter(在线剔除行人点云)。开启时 ground_segmentation '
                    '与 octomap 改吃 /filter/cloud_static')

    filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('filter'),
            'launch', 'filter.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('filter')),
    )

    occupancy_map_2d_arg = DeclareLaunchArgument(
        'occupancy_map_2d', default_value='true')

    costmap_arg = DeclareLaunchArgument(
        'costmap', default_value='true',
        description='是否启动 costmap_2d(滚动窗口局部代价地图,0.05 m,Z-aware DDA)')

    esdf_arg = DeclareLaunchArgument(
        'esdf', default_value='true',
        description='是否启动 esdf(Felzenszwalb 距离场,供规划启发式与碰撞半径查询)')

    robot_model_arg = DeclareLaunchArgument(
        'robot_model', default_value='true',
        description='是否启动 robot_model(足迹多边形/内切外接半径/速度加速度限幅/robot_description)')

    planner_arg = DeclareLaunchArgument(
        'planner', default_value='false',
        description='是否启动 planner(Octile A* / Theta* + 足迹碰撞的全局规划)')
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='false',
        description='是否启动 controller(梯度平滑 + MPPI + 纯跟踪兜底,输出 /cmd_vel)')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    use_driver_arg = DeclareLaunchArgument(
        'use_driver', default_value='true')

    sim_arg = DeclareLaunchArgument(
        'sim', default_value='false',
        description='是否用 Gazebo Harmonic 仿真替代真实驱动(sim_mid360:dart+bullet 雷达 + IMU)')

    sim_headless_arg = DeclareLaunchArgument(
        'sim_headless', default_value='false',
        description='sim:=true 时是否关闭 Gazebo GUI')

    mount_pitch_arg = DeclareLaunchArgument(
        'mount_pitch', default_value='0.0',
        description='雷达相对底盘的俯仰安装角(rad),0 = 与底盘平行,0.5236 = 前倾 30 度')

    mount_roll_arg = DeclareLaunchArgument(
        'mount_roll', default_value='0.0', description='雷达相对底盘的横滚安装角(rad)')

    mount_yaw_arg = DeclareLaunchArgument(
        'mount_yaw', default_value='0.0', description='雷达相对底盘的偏航安装角(rad)')

    mount_x_arg = DeclareLaunchArgument('mount_x', default_value='0.10')
    mount_y_arg = DeclareLaunchArgument('mount_y', default_value='0.0')
    mount_z_arg = DeclareLaunchArgument('mount_z', default_value='0.24')

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mid360_driver'),
            'launch', 'mid360_driver.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_driver')),
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('sim_mid360'),
            'launch', 'sim.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('sim_headless'),
            'mount_x': LaunchConfiguration('mount_x'),
            'mount_y': LaunchConfiguration('mount_y'),
            'mount_z': LaunchConfiguration('mount_z'),
            'mount_roll': LaunchConfiguration('mount_roll'),
            'mount_pitch': LaunchConfiguration('mount_pitch'),
            'mount_yaw': LaunchConfiguration('mount_yaw'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    point_lio = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'launch', 'mapping_mid360.launch.py',
        ])),
        launch_arguments={
            'rviz': 'false',
            'pcd_save': LaunchConfiguration('pcd_save'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )])

    relocalization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('relocalization'),
            'launch', 'relocalization.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('relocalization')),
    )

    def _ground_segmentation(context):
        use_filter = context.perform_substitution(
            LaunchConfiguration('filter')).strip().lower() in ('1', 'true', 'yes', 'on')
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ground_segmentation'),
                'launch', 'ground_segmentation.launch.py',
            ])),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cloud_topic': '/filter/cloud_static' if use_filter else '',
            }.items(),
            condition=IfCondition(LaunchConfiguration('ground_segmentation')),
        )]

    occupancy_map_2d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('occupancy_map_2d'),
            'launch', 'occupancy_map_2d.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('occupancy_map_2d')),
    )

    costmap_2d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('costmap_2d'),
            'launch', 'costmap_2d.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('costmap')),
    )

    esdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('esdf'),
            'launch', 'esdf.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('esdf')),
    )

    robot_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robot_model'),
            'launch', 'robot_model.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'mount_x': LaunchConfiguration('mount_x'),
            'mount_y': LaunchConfiguration('mount_y'),
            'mount_z': LaunchConfiguration('mount_z'),
            'mount_roll': LaunchConfiguration('mount_roll'),
            'mount_pitch': LaunchConfiguration('mount_pitch'),
            'mount_yaw': LaunchConfiguration('mount_yaw'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('robot_model')),
    )

    planner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('planner'),
            'launch', 'planner.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('planner')),
    )

    controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('controller'),
            'launch', 'controller.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('controller')),
    )

    def _backend_include(context):
        """backend:=true 时 include 后端,包找不到就给出可操作报错。

        不用 IfCondition:环境旧(在新增该包前 source)时 launch 只会报
        "package not found",看不出是环境旧了,这里前置判断并提示重新 source。
        """
        raw = context.perform_substitution(LaunchConfiguration('backend')).strip().lower()
        if raw in ('0', 'false', 'no', 'off', ''):
            return []
        try:
            get_package_prefix('point_lio_backend')
        except Exception:
            raise RuntimeError(
                'backend:=true 需要 point_lio_backend,但当前终端环境里找不到它。\n'
                '  几乎都是因为:这个终端是在新增该包之前 source 的(AMENT_PREFIX_PATH 是旧的)。\n'
                '  解决:在**同一个终端**重新 source,或新开终端:\n'
                '      source ~/Projects/A2W-ws/install/setup.zsh\n'
                '  若 install/point_lio_backend 不存在,先:\n'
                '      colcon build --packages-select point_lio_backend\n'
                '  只用前端(point_lio + octomap,不要回环):加 backend:=false')
        backend_save = context.perform_substitution(
            LaunchConfiguration('backend_save')).strip()
        use_relocalization = context.perform_substitution(
            LaunchConfiguration('relocalization')).strip().lower() in ('1', 'true', 'yes', 'on')
        if not backend_save and use_relocalization:
            backend_save = 'false'
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('point_lio_backend'),
                'launch', 'backend_mid360.launch.py',
            ])),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'save_pcd': backend_save,
            }.items(),
        )]

    def _octomap_and_rviz(context):
        """按 backend / octomap 可用性组装 octomap + rviz。

        backend=true 时 octomap 用 map 系(后端发布 map->camera_init),false 时退回
        camera_init 系。octomap 系统包没装时只跳过对应功能并提示,不拖垮核心链路。
        """
        def _is_on(name):
            return context.perform_substitution(
                LaunchConfiguration(name)).strip().lower() in ('1', 'true', 'yes', 'on')

        use_backend = _is_on('backend')
        has_rviz = _has_package('rviz2')
        want_rviz = _is_on('rviz') and has_rviz
        if _is_on('rviz') and not has_rviz:
            print('[bringup] 本机没有 rviz2,已跳过可视化(其余链路照常)', flush=True)
        want_octomap = _is_on('use_octomap')
        has_octomap_server = _has_package('octomap_server')
        has_octomap_rviz = _has_package('octomap_rviz_plugins')

        actions = []
        if want_octomap and has_octomap_server:
            params = [PathJoinSubstitution([
                FindPackageShare('bringup'), 'config', 'octomap.yaml',
            ]).perform(context)]
            override = {'use_sim_time': _is_on('use_sim_time')}
            if use_backend:
                override['frame_id'] = 'map'
            for arg_name, (param_name, cast) in OCTOMAP_OVERRIDABLE.items():
                raw = context.perform_substitution(LaunchConfiguration(arg_name)).strip()
                if not raw:
                    continue
                value = cast(raw)
                top, _, leaf = param_name.partition('.')
                if leaf:
                    override.setdefault(top, {})[leaf] = value
                else:
                    override[top] = value
            params.append(override)
            actions.append(Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server',
                output='screen',
                parameters=params,
                remappings=[('cloud_in', '/filter/cloud_static' if _is_on(
                    'filter') else '/cloud_registered_body')],
            ))
            actions.append(Node(
                package='bringup',
                executable='map_watchdog',
                name='map_watchdog',
                output='screen',
                parameters=[{
                    'out_dir': 'PCD',
                    'map2d_topic': '/occupancy_map_2d/map',
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                }],
            ))
        elif want_octomap:
            print('[mid360] ⚠ 本机没有 octomap_server,已跳过 3D 占据栅格(其余链路照常)。'
                  '\n[mid360]   需要的话: sudo apt install ros-jazzy-octomap-server '
                  'ros-jazzy-octomap-rviz-plugins', flush=True)
        if not has_octomap_rviz and want_rviz:
            print('[mid360] 提示: 没装 octomap_rviz_plugins,rviz 会用去掉 octomap 显示的配置',
                  flush=True)

        if want_rviz:
            from launch_ros.substitutions import FindPackageShare as _FPS
            template = PathJoinSubstitution([
                _FPS('bringup'), 'rviz',
                'backend_map.rviz' if use_backend else 'octomap.rviz',
            ]).perform(context)
            rviz_cfg = _prepare_rviz_config(
                template,
                fixed_frame='map' if use_backend else 'camera_init',
                drop_octomap_plugin=not has_octomap_rviz)
            # octomap_rviz_plugins 2.1.0 的 .so 缺 liboctomap 的 DT_NEEDED,
            # 不预加载会 dlopen 失败(undefined symbol OcTreeStamped)。
            # 必须用 additional_env:env= 会替换整个环境,冲掉 LD_LIBRARY_PATH。
            preload = _octomap_preload()
            if not preload:
                print('[mid360] 提示: 没找到 liboctomap.so,rviz 的 octomap 插件可能加载失败',
                      flush=True)
            rviz_env = {'LD_PRELOAD': preload} if preload else {}
            actions.append(Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)}],
                additional_env=rviz_env,
            ))
        return actions

    return LaunchDescription([
        rviz_arg,
        pcd_save_arg,
        backend_arg,
        use_octomap_arg,
        resolution_arg,
        max_range_arg,
        relocalization_arg,
        backend_save_arg,
        ground_segmentation_arg,
        occupancy_map_2d_arg,
        costmap_arg,
        esdf_arg,
        robot_model_arg,
        planner_arg,
        controller_arg,
        use_sim_time_arg,
        use_driver_arg,
        sim_arg,
        sim_headless_arg,
        mount_pitch_arg,
        mount_roll_arg,
        mount_yaw_arg,
        mount_x_arg,
        mount_y_arg,
        mount_z_arg,
        driver,
        simulator,
        point_lio,
        relocalization,
        filter_arg,
        filter_node,
        OpaqueFunction(function=_ground_segmentation),
        occupancy_map_2d,
        costmap_2d,
        esdf,
        robot_model,
        planner_node,
        controller_node,
        OpaqueFunction(function=_backend_include),
        OpaqueFunction(function=_octomap_and_rviz),
    ])
