"""
Full Mission launch — GPS-denied waypoint navigation.

Starts, in one launch:
    - MAVROS            (Pixhawk 6C link + H-Flow config)
    - state_bridge      (/mavros odom -> /vehicle/state)
    - guidance_controller (waypoint navigation)
    - command_bridge    (guidance -> MAVROS setpoints, gated by nav_enabled)
    - mission_executive (orchestrator; reads waypoints_file)

Using ros2 launch means MAVROS and all nodes are shut down cleanly on
Ctrl+C — no leftover mavros_node holding the serial port.

Waypoints are provided via a JSON file written by gps_to_local
(run interactively BEFORE launching — see run_mission.sh).

Usage:
    ros2 launch mission_executive mission.launch.py \
        waypoints_file:=/tmp/gps_mission.json \
        mission_altitude:=3.0

    Then start the mission:
        ros2 topic pub --once /mission/start std_msgs/String "data: go"

NOTE: set port permissions first (once per plug-in):
    sudo chmod 666 /dev/ttyACM0
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('mission_executive'),
        'config',
        'mavros.yaml'
    )

    # ---------------- Launch arguments ----------------
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='/dev/ttyACM0:921600',
        description='FCU connection URL')
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file', default_value='/tmp/gps_mission.json',
        description='JSON waypoints file written by gps_to_local')
    mission_altitude_arg = DeclareLaunchArgument(
        'mission_altitude', default_value='3.0',
        description='Default mission altitude (m)')
    climb_velocity_arg = DeclareLaunchArgument(
        'climb_velocity', default_value='0.4',
        description='Climb speed (m/s)')
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='false',
        description='Start automatically once ready (else wait for /mission/start)')
    return_home_arg = DeclareLaunchArgument(
        'return_home', default_value='true',
        description='Return to home after last waypoint (else land at last waypoint)')
    detection_mode_arg = DeclareLaunchArgument(
        'detection_mode', default_value='aruco',
        description='Marker detection mode: aruco, color, or model')
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='',
        description='Path to YOLOv8 model file (.pt or .engine) for model detection mode')

    # ---------------- MAVROS ----------------
    mavros_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mavros', 'mavros_node',
            '--ros-args',
            '-p', ['fcu_url:=', LaunchConfiguration('fcu_url')],
            '--params-file', config_file,
        ],
        output='screen',
    )

    # ---------------- Flight nodes ----------------
    state_bridge = Node(
        package='px4_adapter', executable='state_bridge',
        name='state_bridge', output='screen')

    guidance = Node(
        package='guidance_controller', executable='guidance_controller_node',
        name='guidance_controller', output='screen')

    command_bridge = Node(
        package='command_bridge', executable='command_bridge_node',
        name='command_bridge', output='screen')

    camera_publisher = Node(
        package='precision_landing', executable='camera_publisher',
        name='camera_publisher', output='screen')

    precision_landing_node = Node(
        package='precision_landing', executable='precision_landing_node',
        name='precision_landing', output='screen',
        parameters=[{
            'alt': LaunchConfiguration('mission_altitude'),
            'kp_align': 0.15,
            'max_align_speed': 0.15,
            'align_tolerance': 0.05,
            'marker_id': -1,
            'detection_mode': LaunchConfiguration('detection_mode'),
            'model_path': LaunchConfiguration('model_path'),
            'camera_topic': '/down/image_raw',
        }])

    mission_exec = Node(
        package='mission_executive', executable='mission_executive_node',
        name='mission_executive', output='screen',
        parameters=[{
            'mission_altitude': LaunchConfiguration('mission_altitude'),
            'climb_velocity': LaunchConfiguration('climb_velocity'),
            'auto_start': LaunchConfiguration('auto_start'),
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'return_home': LaunchConfiguration('return_home'),
            'detection_mode': LaunchConfiguration('detection_mode'),
            'model_path': LaunchConfiguration('model_path'),
        }])

    # Start flight nodes after MAVROS has had a few seconds to come up.
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[state_bridge, guidance, command_bridge,
                 camera_publisher, precision_landing_node, mission_exec],
    )

    return LaunchDescription([
        fcu_url_arg,
        waypoints_file_arg,
        mission_altitude_arg,
        climb_velocity_arg,
        auto_start_arg,
        return_home_arg,
        detection_mode_arg,
        model_path_arg,
        mavros_cmd,
        delayed_nodes,
    ])
