"""Mid-360 全栈一键启动:驱动 + point_lio + octomap + rviz(单窗口)。

    ros2 launch mid360_bringup mid360.launch.py
    ros2 launch mid360_bringup mid360.launch.py show_rviz:=false

point_lio 的 rviz 固定关掉;rviz 由本 launch 直接起。

⚠️ 参数名用 `show_rviz` 而不是 `rviz` 的原因:
launch 的 IncludeLaunchDescription 会把 include 参数展开成
SetLaunchConfiguration 写进**共享**的 launch_configurations(见
ros2/launch jazzy include_launch_description.py)。给 point_lio 传
'rviz':'false' 会连带把父级自己的 'rviz' 改成 'false' → 后面引用
LaunchConfiguration('rviz') 的节点条件全变假。改名后互不干扰。

注:一键模式下分辨率/量程等调参在 octomap_mid360.launch.py 那层做,
本 launch 不转发(避免同类问题)。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    show_rviz_arg = DeclareLaunchArgument(
        'show_rviz', default_value='true',
        description='启动 rviz2(单窗口:octomap 体素 + LIO 点云 + path)。'
                    '不用 rviz:=true —— 见文件头注释,会被点_lio 的 include 污染')

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
        ]))
    )

    # rviz 直接在父级定义;只读 show_rviz(不会被任何 include 改动)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('mid360_bringup'), 'rviz', 'octomap_mid360.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('show_rviz')),
    )

    return LaunchDescription([
        show_rviz_arg,
        driver,
        point_lio,
        octomap,
        rviz_node,
    ])