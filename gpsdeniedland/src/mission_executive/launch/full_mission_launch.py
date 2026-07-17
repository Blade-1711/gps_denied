"""
Launch file: Full GPS-Denied Navigation Mission

Starts all nodes needed for the complete waypoint mission:
  1. state_bridge      — MAVROS odom → VehicleState
  2. gps_to_local      — Terminal input for waypoints (MUST RUN IN TERMINAL)
  3. guidance_controller — Waypoint navigation
  4. command_bridge    — GuidanceCommand → MAVROS setpoint
  5. mission_executive — Orchestrates everything

Usage:
    This launch file does NOT include gps_to_local because it requires
    terminal input. Run it separately first:

    Terminal 1: ros2 run gps_to_local gps_to_local_node
    Terminal 2: ros2 launch mission_executive full_mission_launch.py
    Terminal 3: ros2 topic pub --once /mission/start std_msgs/String "data: go"

    OR use auto_start:=true and start gps_to_local before launching.

Parameters (pass as launch args):
    mission_altitude : Flight altitude in meters (default: 3.0)
    climb_velocity   : Climb speed in m/s (default: 0.4)
    auto_start       : Start mission automatically when waypoints received (default: false)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # =====================================================
    # Launch Arguments
    # =====================================================

    mission_altitude_arg = DeclareLaunchArgument(
        "mission_altitude",
        default_value="3.0",
        description="Mission flight altitude in meters"
    )

    climb_velocity_arg = DeclareLaunchArgument(
        "climb_velocity",
        default_value="0.4",
        description="Vertical climb speed in m/s"
    )

    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="false",
        description="Start mission automatically when waypoints are received"
    )

    # =====================================================
    # Nodes
    # =====================================================

    state_bridge_node = Node(
        package="px4_adapter",
        executable="state_bridge",
        name="state_bridge",
        output="screen",
    )

    guidance_controller_node = Node(
        package="guidance_controller",
        executable="guidance_controller_node",
        name="guidance_controller",
        output="screen",
    )

    command_bridge_node = Node(
        package="command_bridge",
        executable="command_bridge_node",
        name="command_bridge",
        output="screen",
    )

    mission_executive_node = Node(
        package="mission_executive",
        executable="mission_executive_node",
        name="mission_executive",
        output="screen",
        parameters=[{
            "mission_altitude": LaunchConfiguration("mission_altitude"),
            "climb_velocity": LaunchConfiguration("climb_velocity"),
            "auto_start": LaunchConfiguration("auto_start"),
        }],
    )

    # =====================================================
    # Launch Description
    # =====================================================

    return LaunchDescription([
        mission_altitude_arg,
        climb_velocity_arg,
        auto_start_arg,
        state_bridge_node,
        guidance_controller_node,
        command_bridge_node,
        mission_executive_node,
    ])
