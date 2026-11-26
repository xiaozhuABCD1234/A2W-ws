from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = DeclareLaunchArgument(
        'occupancy_map_2d_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('occupancy_map_2d'), 'config', 'mid360.yaml'
        ]))
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    node = Node(
        package='occupancy_map_2d',
        executable='occupancy_map_2d_node',
        name='occupancy_map_2d',
        output='screen',
        parameters=[
            LaunchConfiguration('occupancy_map_2d_params_file'),
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
        ])
    return LaunchDescription([params_file, use_sim_time, node])
