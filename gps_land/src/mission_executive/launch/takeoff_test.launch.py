"""
Takeoff & Land Test launch — starts MAVROS + the takeoff_land_test node.

Using ros2 launch (not a backgrounded bash 'ros2 run') means the launch
system owns MAVROS's lifecycle and shuts it down cleanly on Ctrl+C — no
leftover mavros_node holding the serial port (the zombie/port-busy bug).

MAVROS is started exactly like the proven drone_takeoff mavros.launch.py:
    ros2 run mavros mavros_node --ros-args -p fcu_url:=... --params-file mavros.yaml

Usage:
    ros2 launch mission_executive takeoff_test.launch.py
    ros2 launch mission_executive takeoff_test.launch.py alt:=0.5 hover:=5.0
    ros2 launch mission_executive takeoff_test.launch.py fcu_url:=/dev/ttyACM1:921600

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

    # mavros.yaml (installed with this package)
    config_file = os.path.join(
        get_package_share_directory('mission_executive'),
        'config',
        'mavros.yaml'
    )

    # ---------------- Launch arguments ----------------
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='/dev/ttyACM0:921600',
        description='FCU connection URL'
    )
    alt_arg = DeclareLaunchArgument(
        'alt', default_value='2.0',
        description='Takeoff altitude (m)'
    )
    hover_arg = DeclareLaunchArgument(
        'hover', default_value='5.0',
        description='Hover time at altitude (s)'
    )
    climb_rate_arg = DeclareLaunchArgument(
        'climb_rate', default_value='0.4',
        description='Climb speed (m/s)'
    )

    # ---------------- MAVROS ----------------
    # Same invocation as the proven drone_takeoff mavros.launch.py.
    mavros_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mavros', 'mavros_node',
            '--ros-args',
            '-p', ['fcu_url:=', LaunchConfiguration('fcu_url')],
            '--params-file', config_file,
        ],
        output='screen',
    )

    # ---------------- Takeoff & Land test node ----------------
    takeoff_node = Node(
        package='mission_executive',
        executable='takeoff_land_test',
        name='takeoff_land_node',
        output='screen',
        parameters=[{
            'alt': LaunchConfiguration('alt'),
            'hover': LaunchConfiguration('hover'),
            'climb_rate': LaunchConfiguration('climb_rate'),
        }],
    )

    # Give MAVROS a few seconds to come up before the node starts.
    # (The node also waits for services, so this is just clean ordering.)
    delayed_node = TimerAction(period=5.0, actions=[takeoff_node])

    return LaunchDescription([
        fcu_url_arg,
        alt_arg,
        hover_arg,
        climb_rate_arg,
        mavros_cmd,
        delayed_node,
    ])
