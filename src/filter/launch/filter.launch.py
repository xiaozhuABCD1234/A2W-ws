from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = DeclareLaunchArgument(
        'filter_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('filter'), 'config', 'mid360.yaml'
        ]))
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    enable = DeclareLaunchArgument('filter_node', default_value='true')
    node = Node(
        package='filter',
        executable='filter_node',
        name='filter',
        output='screen',
        parameters=[
            LaunchConfiguration('filter_params_file'),
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
        ],
        condition=IfCondition(LaunchConfiguration('filter_node')))
    return LaunchDescription([params_file, use_sim_time, enable, node])
