"""
Launch file: Takeoff and Land Test

Minimal test — just takes off, holds for a few seconds, then lands.
Does NOT require waypoints or gps_to_local.

Uses a standalone test node that:
  1. Streams setpoints
  2. Sets OFFBOARD
  3. Arms
  4. Climbs to altitude
  5. Holds for hold_time seconds
  6. Lands via AUTO.LAND
  7. Waits for disarm
  8. Exits

Usage:
    ros2 launch mission_executive takeoff_land_test_launch.py
    ros2 launch mission_executive takeoff_land_test_launch.py mission_altitude:=2.0 hold_time:=5.0

Parameters:
    mission_altitude : meters (default: 2.0)
    climb_velocity   : m/s (default: 0.4)
    hold_time        : seconds to hold at altitude before landing (default: 5.0)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # =====================================================
    # Launch Arguments
    # =====================================================

    alt_arg = DeclareLaunchArgument(
        "mission_altitude",
        default_value="2.0",
        description="Takeoff altitude in meters"
    )

    vel_arg = DeclareLaunchArgument(
        "climb_velocity",
        default_value="0.4",
        description="Climb speed in m/s"
    )

    hold_arg = DeclareLaunchArgument(
        "hold_time",
        default_value="5.0",
        description="Seconds to hold at altitude before landing"
    )

    # =====================================================
    # Nodes
    # =====================================================

    takeoff_land_test_node = Node(
        package="mission_executive",
        executable="takeoff_land_test",
        name="takeoff_land_test",
        output="screen",
        parameters=[{
            "mission_altitude": LaunchConfiguration("mission_altitude"),
            "climb_velocity": LaunchConfiguration("climb_velocity"),
            "hold_time": LaunchConfiguration("hold_time"),
        }],
    )

    # =====================================================
    # Launch
    # =====================================================

    return LaunchDescription([
        alt_arg,
        vel_arg,
        hold_arg,
        takeoff_land_test_node,
    ])
