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
import tempfile

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _has_package(name):
    try:
        get_package_prefix(name)
        return True
    except Exception:
        return False


def _prepare_rviz_config(template_path, fixed_frame, drop_octomap_plugin):
    """按当前环境生成一份 rviz 配置,返回临时文件路径。

    解决两个"环境不全就整个 launch 失败/报一堆插件错"的问题:
      1. World 系在 backend 开启时是 map、关闭时是 camera_init,而 Fixed Frame
         写在 rviz 文件里 —— 直接改模板会污染仓库,所以复制一份再替换。
      2. 本机没装 octomap_rviz_plugins 时,配置里 octomap_rviz_plugins/OccupancyGrid
         这个 display 类加载不了(rviz 面板里一堆红字)。把这一段删掉即可,
         其余显示(点云/path/marker/TF)照常。
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

    use_octomap_arg = DeclareLaunchArgument(
        'use_octomap', default_value='true',
        description='是否启动 octomap_server(3D 占据栅格)。需要系统包 ros-jazzy-octomap-server;'
                    '没装时本 launch 会自动跳过并提示装法,不影响驱动/point_lio/后端')

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

    def _backend_include(context):
        """backend:=true 时 include 后端;包找不到就给出可操作的报错。

        为什么不用 IncludeLaunchDescription + IfCondition:
        当终端环境是在新增 point_lio_backend 之前 source 的,AMENT_PREFIX_PATH 里
        没有它,launch 会抛
          "package 'point_lio_backend' not found, searching: [...]"
        —— 看不出是"环境旧了"。这里前置判断并直接说要重新 source。
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
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('point_lio_backend'),
                'launch', 'backend_mid360.launch.py',
            ])),
        )]

    def _octomap_and_rviz(context):
        """按 backend / octomap 可用性组装 octomap + rviz。

        backend=true: point_lio_backend 发布 map->camera_init 纠偏 TF,octomap 用 map 系;
        backend=false: 完全退回改造前行为(camera_init 系)。

        octomap_server / octomap_rviz_plugins 是**系统包**(ros-jazzy-octomap-server 等),
        没装时不能把整个一键启动拖垮 —— 驱动 + point_lio + 后端才是核心链路。
        所以:没装 octomap_server 就跳过 octomap(大声提示装法),没装 rviz 插件就
        从配置里去掉那个 display。核心链路照常可用。
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
                    FindPackageShare('mid360_bringup'),
                    'launch', 'octomap_mid360.launch.py',
                ])),
                launch_arguments={'frame_id': 'map' if use_backend else ''}.items(),
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
                _FPS('mid360_bringup'), 'rviz',
                'backend_map.rviz' if use_backend else 'octomap_mid360.rviz',
            ]).perform(context)
            rviz_cfg = _prepare_rviz_config(
                template,
                fixed_frame='map' if use_backend else 'camera_init',
                drop_octomap_plugin=not has_octomap_rviz)
            # 关键:octomap_rviz_plugins 2.1.0 的 .so 缺 liboctomap 的 DT_NEEDED,
            # rviz2 又自己不加载 octomap → 不注入的话 OccupancyGrid 显示 dlopen 会报
            # undefined symbol OcTreeStamped。预加载 1.9.7 即解决(实测符号全解析)。
            # 用 additional_env 而非 env:env= 会替换整个环境,把 LD_LIBRARY_PATH 冲掉。
            actions.append(Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_cfg],
                additional_env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.7'},
            ))
        return actions

    return LaunchDescription([
        show_rviz_arg,
        pcd_save_arg,
        backend_arg,
        use_octomap_arg,
        driver,
        point_lio,
        OpaqueFunction(function=_backend_include),
        OpaqueFunction(function=_octomap_and_rviz),
    ])