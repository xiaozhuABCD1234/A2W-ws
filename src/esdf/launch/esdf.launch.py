from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _node(name, config):
    return Node(
        package='esdf',
        executable='esdf_node',
        name=name,
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('esdf'), 'config', config]),
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    return LaunchDescription([
        use_sim_time,
        _node('esdf_local', 'local.yaml'),
        _node('esdf_global', 'global.yaml'),
    ])
