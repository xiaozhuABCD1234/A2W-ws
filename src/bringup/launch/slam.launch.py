"""Mid-360 全栈一键启动:驱动 + point_lio + octomap + rviz(单窗口)。

    ros2 launch bringup slam.launch.py
    ros2 launch bringup slam.launch.py show_rviz:=false
    ros2 launch bringup slam.launch.py pcd_save:=true      # 临时开 PCD 落盘(先验地图)
    ros2 launch bringup slam.launch.py backend:=false      # 关后端,octomap 退回 camera_init 系
    ros2 launch bringup slam.launch.py use_driver:=false use_sim_time:=true

point_lio 的 rviz 固定关掉,rviz 由本 launch 直接起。驱动为本地包
mid360_driver:UDP 收流直接发 /livox/lidar(PointCloud2)与 /livox/imu,
没有 livox_ros_driver2 的 xfer_format 选项。

⚠️ 参数名用 show_rviz 而非 rviz:include 会把参数写进共享的
launch_configurations,传 'rviz' 会把父级自己的 'rviz' 也改成 false,
导致引用它的节点条件全变假。改名后互不干扰。调参(resolution 等)
在 octomap.launch.py 层做,本 launch 不转发。

启动前先跑 preflight.sh 静态自检(驱动与 point_lio 的
lidar_type/timestamp_unit/话题名/host_ip/octomap 输入/src 是否 build)。
不通过直接拒绝启动 —— 这类错配的现场表现是“节点都在,rviz 里永远没点”。
临时跳过:MID360_SKIP_PREFLIGHT=1 ros2 launch ...
"""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _has_package(name):
    try:
        get_package_prefix(name)
        return True
    except Exception:
        return False


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


def _run_preflight():
    """启动前跑静态自检,不通过就抛异常(launch 不启动任何节点)。

    同步执行,只读配置与源码。退出码 2 = 环境不完整,只提醒不拦;1 = 不自洽,拦截。
    """
    if os.environ.get('MID360_SKIP_PREFLIGHT') == '1':
        print('[preflight] 已按 MID360_SKIP_PREFLIGHT=1 跳过')
        return
    try:
        script = os.path.join(get_package_prefix('bringup'), 'lib',
                              'bringup', 'preflight.sh')
    except Exception as exc:  # ament index 里查不到包
        print(f'[preflight] 跳过:定位不到 bringup({exc})')
        return
    if not os.path.isfile(script):
        print(f'[preflight] 跳过:{script} 不存在'
              '(先 colcon build --packages-select bringup)')
        return

    proc = subprocess.run([script, '--quiet'], capture_output=True, text=True)
    print(proc.stdout, end='')
    if proc.returncode == 1:
        # 先打到 stdout 再抛异常:直接抛会被包成 InvalidLaunchFileError("可能是语法错"),带偏方向。
        print('[preflight] ❌ 配置不自洽,拒绝启动:见上方 FAIL 项。', flush=True)
        print('[preflight]   修好后重试;确需强行启动:MID360_SKIP_PREFLIGHT=1',
              flush=True)
        raise RuntimeError('preflight 未通过,拒绝启动')
    if proc.returncode == 2:
        print('[preflight] 环境不完整,跳过拦截(仅提醒)', flush=True)


def generate_launch_description():
    _run_preflight()

    show_rviz_arg = DeclareLaunchArgument(
        'show_rviz', default_value='true',
        description='启动 rviz2(单窗口:octomap 体素 + LIO 点云 + path)。'
                    '不用 rviz:=true —— 见文件头注释,会被点_lio 的 include 污染')

    # P7:透传「是否落盘 PCD」给 point_lio。留空 = 用 yaml
    # (约定 A);显式传 true/false 才覆盖。launch 参数是唯一免 build 的开关。
    pcd_save_arg = DeclareLaunchArgument(
        'pcd_save', default_value='',
        description='是否把建图点云落盘(先验地图用)。留空 = 用 point_lio 的 yaml '
                    '(现为 true);显式 true/false 才覆盖。注意:只在节点【正常退出】'
                    '(Ctrl+C)时写一次,写到 ./PCD/scans.pcd(相对运行本 launch 的目录)')

    use_octomap_arg = DeclareLaunchArgument(
        'use_octomap', default_value='true',
        description='是否启动 octomap_server(3D 占据栅格)。需要系统包 ros-jazzy-octomap-server;'
                    '没装时本 launch 会自动跳过并提示装法,不影响驱动/point_lio/后端')

    backend_arg = DeclareLaunchArgument(
        'backend', default_value='true',
        description='是否启动 point_lio_backend(回环检测 + 位姿图)。true 时 octomap 与 rviz '
                    '切到 map 系;false 时退回改造前行为(camera_init 系)')

    relocalization_arg = DeclareLaunchArgument(
        'relocalization', default_value='true')

    backend_save_arg = DeclareLaunchArgument('backend_save', default_value='')

    ground_segmentation_arg = DeclareLaunchArgument(
        'ground_segmentation', default_value='true')

    occupancy_map_2d_arg = DeclareLaunchArgument(
        'occupancy_map_2d', default_value='true')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    use_driver_arg = DeclareLaunchArgument(
        'use_driver', default_value='true')

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

    point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'launch', 'mapping_mid360.launch.py',
        ])),
        launch_arguments={
            'rviz': 'false',
            # P7:透传给 point_lio 的 mapping launch;空串 = 不覆盖 yaml
            'pcd_save': LaunchConfiguration('pcd_save'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

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

    ground_segmentation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ground_segmentation'),
            'launch', 'ground_segmentation.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('ground_segmentation')),
    )

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
        show_rviz = _is_on('show_rviz')
        want_octomap = _is_on('use_octomap')
        has_octomap_server = _has_package('octomap_server')
        has_octomap_rviz = _has_package('octomap_rviz_plugins')

        actions = []
        if want_octomap and has_octomap_server:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('bringup'),
                    'launch', 'octomap.launch.py',
                ])),
                launch_arguments={
                    'frame_id': 'map' if use_backend else '',
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            ))
            actions.append(Node(
                package='bringup',
                executable='map_watchdog',
                name='map_watchdog',
                output='screen',
                parameters=[{
                    'out_dir': 'PCD',
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                }],
            ))
        elif want_octomap:
            print('[mid360] ⚠ 本机没有 octomap_server,已跳过 3D 占据栅格(其余链路照常)。'
                  '\n[mid360]   需要的话: sudo apt install ros-jazzy-octomap-server '
                  'ros-jazzy-octomap-rviz-plugins', flush=True)
        if not has_octomap_rviz and show_rviz:
            print('[mid360] 提示: 没装 octomap_rviz_plugins,rviz 会用去掉 octomap 显示的配置',
                  flush=True)

        if show_rviz:
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
            actions.append(Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)}],
                additional_env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.7'},
            ))
        return actions

    return LaunchDescription([
        show_rviz_arg,
        pcd_save_arg,
        backend_arg,
        use_octomap_arg,
        relocalization_arg,
        backend_save_arg,
        ground_segmentation_arg,
        occupancy_map_2d_arg,
        use_sim_time_arg,
        use_driver_arg,
        driver,
        point_lio,
        relocalization,
        ground_segmentation,
        occupancy_map_2d,
        OpaqueFunction(function=_backend_include),
        OpaqueFunction(function=_octomap_and_rviz),
    ])