"""Mid-360 全栈一键启动:驱动 + point_lio + octomap + rviz(单窗口)。

    ros2 launch mid360_bringup mid360.launch.py
    ros2 launch mid360_bringup mid360.launch.py rviz:=false
    ros2 launch mid360_bringup mid360.launch.py resolution:=0.2 max_range:=30.0

point_lio 的 rviz 固定关掉(由本 launch 的 rviz 单窗口统一显示)。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='启动 rviz2(单窗口:octomap + LIO 点云 + path)')

    resolution_arg = DeclareLaunchArgument(
        'resolution', default_value='0.1',
        description='octomap 体素边长 [m],同时决定 2D 投影分辨率')

    max_range_arg = DeclareLaunchArgument(
        'max_range', default_value='20.0',
        description='octomap 射线截断距离 [m];越大 CPU/内存越高')

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('livox_ros_driver2'),
            'launch_ROS2', 'msg_MID360_launch.py',
        ]))
    )

    point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'launch', 'mapping_mid360.launch.py',
        ])),
        launch_arguments={'rviz': 'false'}.items(),
    )

    octomap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mid360_bringup'),
            'launch', 'octomap_mid360.launch.py',
        ])),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'resolution': LaunchConfiguration('resolution'),
            'max_range': LaunchConfiguration('max_range'),
        }.items(),
    )

    return LaunchDescription([
        rviz_arg,
        resolution_arg,
        max_range_arg,
        driver,
        point_lio,
        octomap,
    ])