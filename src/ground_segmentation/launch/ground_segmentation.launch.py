from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = DeclareLaunchArgument(
        'ground_segmentation_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ground_segmentation'), 'config', 'mid360.yaml'
        ]))
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    node = Node(
        package='ground_segmentation',
        executable='ground_segmentation_node',
        name='ground_segmentation',
        output='screen',
        parameters=[
            LaunchConfiguration('ground_segmentation_params_file'),
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
        ])
    return LaunchDescription([params_file, use_sim_time, node])
