"""Mid-360 全栈一键启动:驱动 + point_lio + octomap + rviz(单窗口)。

    ros2 launch mid360_bringup mid360.launch.py
    ros2 launch mid360_bringup mid360.launch.py show_rviz:=false
    ros2 launch mid360_bringup mid360.launch.py pcd_save:=true      # 临时开 PCD 落盘(先验地图)
    ros2 launch mid360_bringup mid360.launch.py pcd_save:=false     # 建图时不落盘(省内存/CPU)
    ros2 launch mid360_bringup mid360.launch.py backend:=false      # 关后端回环/位姿图
                                                                    # (octomap 退回 camera_init 系)

point_lio 的 rviz 固定关掉;rviz 由本 launch 直接起。

驱动换成本地包 mid360_driver:它直接 UDP 收流并发布 /livox/lidar
(sensor_msgs/PointCloud2)与 /livox/imu(sensor_msgs/Imu),没有
livox_ros_driver2 的 xfer_format 选项(点云恒为 PointCloud2)。

⚠️ 参数名用 `show_rviz` 而不是 `rviz` 的原因:
launch 的 IncludeLaunchDescription 会把 include 参数展开成
SetLaunchConfiguration 写进**共享**的 launch_configurations(见
ros2/launch jazzy include_launch_description.py)。给 point_lio 传
'rviz':'false' 会连带把父级自己的 'rviz' 改成 'false' → 后面引用
LaunchConfiguration('rviz') 的节点条件全变假。改名后互不干扰。

注:一键模式下分辨率/量程等调参在 octomap_mid360.launch.py 那层做,
本 launch 不转发(避免同类问题)。

启动前会先跑 scripts/preflight.sh 做一次静态自检(mid360_driver 的
PointCloud2/Imu 与 point_lio 的 lidar_type/timestamp_unit 是否匹配、
话题名、host_ip、octomap 是否吃机体系点云、src 是否忘了 build)。
不通过则直接拒绝启动 —— 因为这类错配的现场表现是“节点都起来了,
但 rviz 里永远没有点”,靠日志很难定位。
临时跳过:MID360_SKIP_PREFLIGHT=1 ros2 launch ...
"""

import os
import subprocess

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _run_preflight():
    """启动前跑静态自检,不通过就抛异常(launch 不会启动任何节点)。

    在 generate_launch_description 阶段同步执行,只读配置与源码,
    不依赖 ROS 节点,约 0.2 s。退出码 2 = 环境不完整(比如系统安装
    没有 src/),只提醒不拦;退出码 1 = 配置真的不自洽,拦截。
    """
    if os.environ.get('MID360_SKIP_PREFLIGHT') == '1':
        print('[preflight] 已按 MID360_SKIP_PREFLIGHT=1 跳过')
        return
    try:
        script = os.path.join(get_package_prefix('mid360_bringup'), 'lib',
                              'mid360_bringup', 'preflight.sh')
    except Exception as exc:  # ament index 里查不到包
        print(f'[preflight] 跳过:定位不到 mid360_bringup({exc})')
        return
    if not os.path.isfile(script):
        print(f'[preflight] 跳过:{script} 不存在'
              '(先 colcon build --packages-select mid360_bringup)')
        return

    proc = subprocess.run([script, '--quiet'], capture_output=True, text=True)
    print(proc.stdout, end='')
    if proc.returncode == 1:
        # 先打在 stdout,再抛异常 —— 否则用户只能看到 launch 的
        # InvalidLaunchFileError("可能是语法错"),会把方向带偏。
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

    # LOCAL PATCH P7(见 docs/PATCHES.md):把「是否落盘 PCD」透传给 point_lio 的
    # mapping launch。留空 = 用 point_lio 的 config/mid360.yaml(约定 A:yaml 是
    # 唯一真相源);显式传 true/false 才覆盖。
    # 这是唯一不需要 colcon build 的开关方式:yaml 改了要 build 才同步到 install/,
    # 而 launch 参数是启动时生效的。
    pcd_save_arg = DeclareLaunchArgument(
        'pcd_save', default_value='',
        description='是否把建图点云落盘(先验地图用)。留空 = 用 point_lio 的 yaml '
                    '(现为 true);显式 true/false 才覆盖。注意:只在节点【正常退出】'
                    '(Ctrl+C)时写一次,写到 ./PCD/scans.pcd(相对运行本 launch 的目录)')

    backend_arg = DeclareLaunchArgument(
        'backend', default_value='true',
        description='是否启动 point_lio_backend(回环检测 + 位姿图)。true 时 octomap 与 rviz '
                    '切到 map 系;false 时退回改造前行为(camera_init 系)')

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mid360_driver'),
            'launch', 'mid360_driver.launch.py',
        ]))
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
        }.items(),
    )

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('point_lio_backend'),
            'launch', 'backend_mid360.launch.py',
        ])),
        condition=IfCondition(LaunchConfiguration('backend')),
    )

    def _octomap_and_rviz(context):
        """按 backend 开关选择 octomap 世界系与 rviz 配置。

        backend=true: point_lio_backend 发布 map->camera_init 纠偏 TF,octomap 用 map 系;
        backend=false: 完全退回改造前行为(camera_init 系)。
        """
        def _is_on(name):
            return context.perform_substitution(
                LaunchConfiguration(name)).strip().lower() in ('1', 'true', 'yes', 'on')

        use_backend = _is_on('backend')
        show_rviz = _is_on('show_rviz')
        actions = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('mid360_bringup'),
                'launch', 'octomap_mid360.launch.py',
            ])),
            launch_arguments={'frame_id': 'map' if use_backend else ''}.items(),
        )]
        if show_rviz:
            # 关键:octomap_rviz_plugins 2.1.0 的 .so 缺 liboctomap 的 DT_NEEDED,
            # rviz2 又自己不加载 octomap → 不注入的话 OccupancyGrid 显示 dlopen 会报
            # undefined symbol OcTreeStamped。预加载 1.9.7 即解决(实测符号全解析)。
            # 用 additional_env 而非 env:env= 会替换整个环境,把 LD_LIBRARY_PATH 冲掉。
            actions.append(Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('mid360_bringup'), 'rviz',
                    'backend_map.rviz' if use_backend else 'octomap_mid360.rviz',
                ])],
                additional_env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.7'},
            ))
        return actions

    return LaunchDescription([
        show_rviz_arg,
        pcd_save_arg,
        backend_arg,
        driver,
        point_lio,
        backend,
        OpaqueFunction(function=_octomap_and_rviz),
    ])