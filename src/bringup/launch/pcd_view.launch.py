"""离线看 PCD:把点云文件当话题发出来 + rviz 显示,不需要雷达/point_lio 在跑。

    ros2 launch bringup pcd_view.launch.py [pcd:=...] [rviz:=false] [static_tf:=false]

话题 /cloud_pcd 是 pcd_to_pointcloud 写死的(无 topic 参数),frame_id 由 tf_frame 决定。

单独看 PCD 时 TF 树为空,rviz 会报 "Fixed Frame does not exist" —— 点云照画
(点云 frame 与 Fixed Frame 同名时 tf2 直接返回单位变换),只是报红难看;
默认发一条 map -> <frame> 的单位静态 TF 塞进树里。已在跑 point_lio 时传 static_tf:=false。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# point_lio 相对路径落盘(P8):运行 launch 目录下的 PCD/scans.pcd,同目录启动即可看到。
DEFAULT_PCD = './PCD/scans.pcd'


def generate_launch_description():
    pcd_arg = DeclareLaunchArgument(
        'pcd', default_value=DEFAULT_PCD,
        description='PCD 文件路径。默认是 point_lio 落盘的那份(scans.pcd)')

    frame_arg = DeclareLaunchArgument(
        'frame', default_value='camera_init',
        description='点云 frame_id。point_lio 的世界系就叫 camera_init;'
                    '看别的文件(比如机体系)时改这里')

    period_arg = DeclareLaunchArgument(
        'period_ms', default_value='1000',
        description='重发周期 [ms]。pcd_to_pointcloud 的 QoS 是 RELIABLE/VOLATILE'
                    '(不 latch),靠周期重发让后启动的 rviz 也能收到;'
                    '调大省带宽,调小加载更快')

    static_tf_arg = DeclareLaunchArgument(
        'static_tf', default_value='true',
        description='发一条 map -> <frame> 的单位静态 TF,消掉 rviz 的 '
                    '"Fixed Frame does not exist" 报错。已经在跑 point_lio 时'
                    '(camera_init 本来就在 TF 里)传 false')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='启动 rviz2(用 rviz/pcd_view.rviz)')

    # 注意:节点名 pcd_publisher 和话题 /cloud_pcd 都是上游写死的,改不了
    pcd_publisher = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='pcd_publisher',
        output='screen',
        parameters=[{
            'file_name': LaunchConfiguration('pcd'),
            'tf_frame': LaunchConfiguration('frame'),
            'publishing_period_ms': LaunchConfiguration('period_ms'),
        }],
    )

    # 父帧叫 map:不与 point_lio 抢 camera_init(tf2 一个帧只能有一个父帧,只是挂个上家)。
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='pcd_static_tf',
        output='log',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', LaunchConfiguration('frame'),
        ],
        condition=IfCondition(LaunchConfiguration('static_tf')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('bringup'), 'rviz', 'pcd_view.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        pcd_arg,
        frame_arg,
        period_arg,
        static_tf_arg,
        rviz_arg,
        pcd_publisher,
        static_tf,
        rviz_node,
    ])
