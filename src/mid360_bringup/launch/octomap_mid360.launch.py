"""Point-LIO 点云 -> octomap 三维占据栅格。

前置(另开终端,先跑起来):
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
    ros2 launch point_lio mapping_mid360.launch.py rviz:=false

本 launch 只负责 octomap_server 这一层,方便单独重启/调参。

    ros2 launch mid360_bringup octomap_mid360.launch.py
    ros2 launch mid360_bringup octomap_mid360.launch.py resolution:=0.2 max_range:=30.0
    ros2 launch mid360_bringup octomap_mid360.launch.py rviz:=true

输出:
    /octomap_full, /octomap_binary   octomap_msgs/Octomap(3D 栅格)
    /octomap_point_cloud_centers     占据体素中心点云
    /occupied_cells_vis_array        MarkerArray(粒度随 octree 深度)
    /projected_map                   nav_msgs/OccupancyGrid(2D 投影,给 nav2/costmap)
服务:
    /octomap_server/octomap_full, /octomap_binary  (用 octomap_saver 存盘)
    /octomap_server/clear_bbox
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('mid360_bringup'), 'config', 'octomap_mid360.yaml'
    ])

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='启动 rviz2(需要手动加 Octomap display,subscribes /octomap_full)')

    resolution_arg = DeclareLaunchArgument(
        'resolution', default_value='0.1',
        description='体素边长 [m],同时决定 2D 投影分辨率')

    max_range_arg = DeclareLaunchArgument(
        'max_range', default_value='20.0',
        description='射线截断距离 [m];越大 CPU/内存越高')

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            params_file,
            # 注意:LaunchConfiguration 拿到的是字符串,double 参数必须用
            # ParameterValue(..., value_type=float) 转换,否则 declare_parameter
            # 会因为类型不匹配抛 InvalidParameterTypeException。
            {'resolution': ParameterValue(
                LaunchConfiguration('resolution'), value_type=float)},
            {'sensor_model.max_range': ParameterValue(
                LaunchConfiguration('max_range'), value_type=float)},
        ],
        # 关键:必须喂机体系点云,见 config/octomap_mid360.yaml 顶部的说明。
        remappings=[('cloud_in', '/cloud_registered_body')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        resolution_arg,
        max_range_arg,
        octomap_node,
        rviz_node,
    ])
