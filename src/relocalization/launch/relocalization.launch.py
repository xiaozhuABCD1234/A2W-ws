from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    relocalization_params_file = DeclareLaunchArgument(
        'relocalization_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('relocalization'), 'config', 'mid360.yaml'
        ]))
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    node = Node(
        package='relocalization',
        executable='relocalization_node',
        name='relocalization',
        output='screen',
        parameters=[
            LaunchConfiguration('relocalization_params_file'),
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
        ])
    return LaunchDescription([relocalization_params_file, use_sim_time, node])
