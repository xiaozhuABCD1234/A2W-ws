"""Point-LIO 后端（回环 + 位姿图）单独启动。

一般不用直接跑：mid360.launch.py 里 backend:=true 会自动 include 本文件。
单独调试：

    ros2 launch point_lio_backend backend_mid360.launch.py
    ros2 launch point_lio_backend backend_mid360.launch.py params_file:=/path/to/x.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('point_lio_backend'), 'config', 'backend_mid360.yaml'
        ]),
        description='后端参数 yaml（唯一真相源）')

    backend_node = Node(
        package='point_lio_backend',
        executable='point_lio_backend_node',
        name='point_lio_backend',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_arg, backend_node])
