from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = get_package_share_directory('controller') + '/config/controller.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='controller',
            executable='controller_node',
            name='controller',
            output='screen',
            parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
