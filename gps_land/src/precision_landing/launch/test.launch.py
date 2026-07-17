"""
Precision Landing — Launch File

Starts:
    - MAVROS (Pixhawk connection)
    - camera_publisher (auto-detects RealSense/USB → /down/image_raw + rangefinder)
    - precision_landing_node (activation-based: waits for /precision_landing/activate)

Usage:
    ros2 launch precision_landing test.launch.py
    ros2 launch precision_landing test.launch.py alt:=1.0 marker_id:=100
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Arguments
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='/dev/ttyACM0:921600')
    alt_arg = DeclareLaunchArgument(
        'alt', default_value='1.5', description='Takeoff altitude (m)')
    hover_time_arg = DeclareLaunchArgument(
        'hover_time', default_value='3.0', description='Hover before scanning (s)')
    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='-1', description='ArUco ID (-1=any)')
    align_tolerance_arg = DeclareLaunchArgument(
        'align_tolerance', default_value='0.05')
    max_align_speed_arg = DeclareLaunchArgument(
        'max_align_speed', default_value='0.15')

    # MAVROS
    mavros_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mavros', 'mavros_node',
            '--ros-args',
            '-p', ['fcu_url:=', LaunchConfiguration('fcu_url')],
            '--params-file', os.path.join(
                get_package_share_directory('precision_landing'),
                'config', 'mavros.yaml'),
        ],
        output='screen',
    )

    # Camera (auto-detects)
    camera = Node(
        package='precision_landing', executable='camera_publisher',
        name='camera_publisher', output='screen')

    # Precision Landing Test
    landing_node = Node(
        package='precision_landing', executable='precision_landing_node',
        name='precision_landing', output='screen',
        parameters=[{
            'alt': LaunchConfiguration('alt'),
            'hover_time': LaunchConfiguration('hover_time'),
            'marker_id': LaunchConfiguration('marker_id'),
            'align_tolerance': LaunchConfiguration('align_tolerance'),
            'max_align_speed': LaunchConfiguration('max_align_speed'),
            'camera_topic': '/down/image_raw',
            'use_aruco': True,
            'kp_align': 0.15,
        }])

    # Delay nodes to let MAVROS come up
    delayed = TimerAction(
        period=5.0,
        actions=[camera, landing_node])

    return LaunchDescription([
        fcu_url_arg, alt_arg, hover_time_arg, marker_id_arg,
        align_tolerance_arg, max_align_speed_arg,
        mavros_cmd, delayed,
    ])
