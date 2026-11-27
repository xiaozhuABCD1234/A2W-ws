from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _node(context):
    overrides = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    cloud_topic = context.perform_substitution(LaunchConfiguration('cloud_topic')).strip()
    if cloud_topic:
        overrides['cloud_topic'] = cloud_topic
    return [Node(
        package='ground_segmentation',
        executable='ground_segmentation_node',
        name='ground_segmentation',
        output='screen',
        parameters=[
            context.perform_substitution(LaunchConfiguration('ground_segmentation_params_file')),
            overrides,
        ])]


def generate_launch_description():
    params_file = DeclareLaunchArgument(
        'ground_segmentation_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ground_segmentation'), 'config', 'mid360.yaml'
        ]))
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    cloud_topic = DeclareLaunchArgument(
        'cloud_topic', default_value='',
        description='覆盖 yaml 里的 cloud_topic;留空 = 用 yaml')
    return LaunchDescription([params_file, use_sim_time, cloud_topic,
                              OpaqueFunction(function=_node)])
