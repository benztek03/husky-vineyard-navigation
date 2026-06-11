from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_file_arg = DeclareLaunchArgument('mission_file', default_value='')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='a200_1071')

    row_follower = Node(
        package='vineyard_nav',
        executable='row_follower',
        name='row_follower',
        output='screen',
        parameters=[{
            'scan_topic': '/a200_1071/sensors/lidar2d_0/scan',
            'cmd_topic': '/a200_1071/cmd_vel',
            'enable_topic': '/a200_1071/vineyard_row_follow/enable',
            'status_topic': '/a200_1071/vineyard_row_follow/status',
            'base_speed': 0.30,
            'max_speed': 0.45,
            'max_angular': 0.80,
            'lookahead_min_x': 0.7,
            'lookahead_max_x': 4.0,
            'side_min_abs_y': 0.35,
            'side_max_abs_y': 2.5,
            'min_side_points': 8,
            'center_gain': 1.4,
            'heading_gain': 1.0,
            'fit_residual_threshold': 0.15,
            'lost_row_stop_scans': 4,
            'x_eval': 2.0,
        }],
    )

    mission_manager = Node(
        package='vineyard_nav',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'namespace': LaunchConfiguration('namespace'),
            'mission_file': LaunchConfiguration('mission_file'),
            'enable_topic': '/a200_1071/vineyard_row_follow/enable',
            'odom_topic': '/a200_1071/platform/odom/filtered',
            'localizer_name': 'amcl',
        }],
    )

    return LaunchDescription([
        mission_file_arg,
        namespace_arg,
        row_follower,
        mission_manager,
    ])
