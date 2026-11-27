from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    mid360_driver_node = Node(
        package='mid360_driver',
        executable='mid360_driver_node',
        name='mid360_driver',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('mid360_driver'),
                    'config',
                    'param.yaml',
                ]
            ),
            {'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)},
        ],
    )

    return LaunchDescription([use_sim_time_arg, mid360_driver_node])
