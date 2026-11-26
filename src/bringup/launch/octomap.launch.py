"""Point-LIO 点云 -> octomap 三维占据栅格。只负责 octomap_server + rviz 层。

前置(另开终端):
    ros2 launch mid360_driver mid360_driver.launch.py
    ros2 launch point_lio mapping_slam.launch.py rviz:=false

    ros2 launch bringup octomap.launch.py [rviz:=true] [resolution:=0.2 max_range:=15.0]

参数来源:config/octomap.yaml 是 resolution/max_range 的唯一真相源,
只有命令行显式传参才覆盖(旧的 default_value='0.1' 写法等于无条件顶掉 yaml,
改 yaml 不生效)。

日常一键启动:ros2 launch bringup slam.launch.py

输出:/octomap_full, /octomap_binary(Octomap)/ /octomap_point_cloud_centers
/ /occupied_cells_vis_array(MarkerArray)/ /projected_map(2D 投影,给 nav2)
服务:/octomap_server/octomap_full, /octomap_binary, /octomap_server/clear_bbox
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# 命令行可临时覆盖的两个参数:arg 名 -> (参数名, 类型)
OVERRIDABLE = {
    'resolution': ('resolution', float),
    'max_range': ('sensor_model.max_range', float),
}


def _is_on(context, name):
    return context.perform_substitution(
        LaunchConfiguration(name)).strip().lower() in ('1', 'true', 'yes', 'on')


def _octomap_node(context):
    """按「yaml 优先,显式传参才覆盖」组装 octomap_server。

    不能在 DeclareLaunchArgument 上给数字 default_value:parameters 列表后面的项
    覆盖前面的,LaunchConfiguration 又总是被求值,等于无条件顶掉 yaml ——
    改 config/*.yaml 不生效。default_value='' 时只有显式传参才追加覆盖项。
    """
    params = [PathJoinSubstitution([
        FindPackageShare('bringup'), 'config', 'octomap.yaml'
    ]).perform(context)]

    override = {'use_sim_time': _is_on(context, 'use_sim_time')}
    # frame_id 默认空 = 用 yaml 的 camera_init；backend 开启时由上层 launch 传 map。
    frame_override = context.perform_substitution(LaunchConfiguration('frame_id')).strip()
    if frame_override:
        override['frame_id'] = frame_override
    for arg_name, (param_name, cast) in OVERRIDABLE.items():
        raw = context.perform_substitution(LaunchConfiguration(arg_name)).strip()
        if not raw:
            continue
        value = cast(raw)          # 自己转换类型,避免 declare_parameter 抛 InvalidParameterTypeException
        top, _, leaf = param_name.partition('.')
        if leaf:
            override.setdefault(top, {})[leaf] = value
        else:
            override[top] = value
    if override:
        params.append(override)

    return [Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=params,
        # 关键:必须喂机体系点云,见 config/octomap.yaml 顶部的说明。
        remappings=[('cloud_in', '/cloud_registered_body')],
    )]


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='启动 rviz2(octomap_rviz_plugins 的 OccupancyGrid 显示, '
                    'subscribes /octomap_binary,按占据概率着色)')

    resolution_arg = DeclareLaunchArgument(
        'resolution', default_value='',
        description='体素边长 [m]。留空 = 用 config/octomap.yaml 里的值;'
                    '显式传参才覆盖。运行时改不了,须重启节点')

    max_range_arg = DeclareLaunchArgument(
        'max_range', default_value='',
        description='射线截断距离 [m]。留空 = 用 config/octomap.yaml 里的值;'
                    '显式传参才覆盖。越大 CPU/内存越高,也越容易在远处出重影')

    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='',
        description='世界系。留空 = 用 config/octomap.yaml 的 camera_init;'
                    '后端启用时上层传 map(map->camera_init 由 point_lio_backend 发布)')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=PathJoinSubstitution([
            FindPackageShare('bringup'), 'rviz', 'octomap.rviz'
        ]),
        description='rviz 配置文件。后端启用时上层传 rviz/backend_map.rviz(Fixed Frame: map)')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool)}],
        # octomap_rviz_plugins 2.1.0 的 .so 没把 liboctomap 写进 DT_NEEDED,
        # rviz2 自己又不加载 octomap → dlopen 报 undefined symbol OcTreeStamped;
        # 预加载 liboctomap.so.1.9.7 解决(只注入 rviz2)。
        # 必须用 additional_env 而非 env:Jazzy 的 env= 是替换整个环境,
        # 会冲掉 LD_LIBRARY_PATH,rviz2 因找不到 libOgreMain 以 127 退出(实测)。
        additional_env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.7'},
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        resolution_arg,
        max_range_arg,
        frame_id_arg,
        rviz_config_arg,
        use_sim_time_arg,
        OpaqueFunction(function=_octomap_node),
        rviz_node,
    ])
