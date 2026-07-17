#!/usr/bin/env python3

"""
Node:
    Command Bridge

Purpose:
    Converts guidance commands (forward velocity + desired yaw) into MAVROS
    PositionTarget setpoints using LOCAL_NED frame with:

    TWO MODES (automatic switching based on forward_speed):

    ALIGN mode (forward_speed ≈ 0):
      - Position XY: holds drone at captured position (no drift during rotation)
      - Position Z: holds the target altitude
      - Yaw Rate: smooth tanh rotation

    MOVE mode (forward_speed > 0):
      - Velocity XY: forward speed decomposed into ENU components
      - Position Z: holds the target altitude
      - Yaw Rate: smooth tanh yaw corrections

    NOTE: Despite the frame being named "LOCAL_NED", MAVROS's setpoint_raw
    plugin internally converts ENU→NED before sending to PX4. So we provide
    all values in ENU convention (ROS standard):
      position.x = East, position.y = North, position.z = Up
      yaw_rate = CCW positive

    The drone is always pre-aligned to face the waypoint before moving
    forward. Because of this alignment, the physical motion is along the
    drone's body X-axis — giving the optical flow sensor a clean single-axis
    measurement for EKF pose estimation.

    GATED: Only forwards setpoints when navigation is enabled by the
    mission_executive (via /mission/nav_enabled).

Input:
    /guidance/command       (GuidanceCommand — forward_speed, desired_yaw ENU)
    /vehicle/state          (VehicleState — x, y, z, yaw in ENU from state_bridge)
    /mavros/state           (State — FCU connection)
    /mission/nav_enabled    (Bool — gate: only publish when True)
    /mission/current_waypoint (Waypoint — target waypoint with altitude)

Output:
    /mavros/setpoint_raw/local  (PositionTarget — ENU values, MAVROS converts to NED)
"""

import math

import rclpy
from rclpy.node import Node

from gps_navigation_interfaces.msg import GuidanceCommand, VehicleState, Waypoint
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import Bool


# ==========================================================
# Tunable Parameters
# ==========================================================

CONTROL_RATE = 50.0             # Hz — setpoint publish rate

# Yaw rate controller (tanh profile)
MAX_YAW_RATE = 0.2618          # rad/s — maximum yaw rotation rate (15°/s)
YAW_TANH_K = 1.5              # steepness of tanh curve (lower = gentler
                               # approach near alignment, less overshoot)

# ALIGN vs MOVE threshold
ALIGN_SPEED_THRESHOLD = 0.01   # m/s — below this, switch to position hold XY


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class CommandBridge(Node):

    def __init__(self):
        super().__init__("command_bridge")

        # ------------------------------------------
        # Subscribers
        # ------------------------------------------
        self.create_subscription(
            GuidanceCommand, "/guidance/command",
            self.guidance_callback, 10)

        self.create_subscription(
            VehicleState, "/vehicle/state",
            self.vehicle_state_callback, 10)

        self.create_subscription(
            State, "/mavros/state",
            self.state_callback, 10)

        self.create_subscription(
            Bool, "/mission/nav_enabled",
            self.nav_enabled_callback, 10)

        self.create_subscription(
            Waypoint, "/mission/current_waypoint",
            self.waypoint_callback, 10)

        # ------------------------------------------
        # Publisher
        # ------------------------------------------
        self.setpoint_pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10)

        # ------------------------------------------
        # Timer
        # ------------------------------------------
        self.timer = self.create_timer(
            1.0 / CONTROL_RATE, self.control_loop)

        # ------------------------------------------
        # Internal Data
        # ------------------------------------------
        self.guidance_cmd = GuidanceCommand()
        self.vehicle_state = VehicleState()
        self.mavros_state = State()
        self.current_waypoint = Waypoint()

        self.have_guidance = False
        self.have_vehicle_state = False
        self.have_mavros_state = False
        self.have_waypoint = False
        self.nav_enabled = False

        # Target altitude (EKF z from mission_executive via waypoint.z)
        self.target_altitude = 3.0

        # Position hold for ALIGN mode
        self.align_hold_x = 0.0     # Captured ENU East when entering ALIGN
        self.align_hold_y = 0.0     # Captured ENU North when entering ALIGN
        self.in_align_mode = False  # True when holding position (speed ≈ 0)

        self.get_logger().info("────────────────────────────────────")
        self.get_logger().info(" Command Bridge Started")
        self.get_logger().info(" Frame    : LOCAL_NED (ENU input)")
        self.get_logger().info(" ALIGN    : Position XY hold + yaw_rate")
        self.get_logger().info(" MOVE     : Velocity XY + yaw_rate")
        self.get_logger().info(" Both     : Position Z (altitude hold)")
        self.get_logger().info(f" Yaw Rate : tanh (max {math.degrees(MAX_YAW_RATE):.0f}°/s)")
        self.get_logger().info(" (waiting for nav_enabled=True)")
        self.get_logger().info("────────────────────────────────────")

    # ------------------------------------------
    # Callbacks
    # ------------------------------------------
    def guidance_callback(self, msg):
        self.guidance_cmd = msg
        self.have_guidance = True

    def vehicle_state_callback(self, msg):
        self.vehicle_state = msg
        self.have_vehicle_state = True

    def state_callback(self, msg):
        self.mavros_state = msg
        self.have_mavros_state = True

    def nav_enabled_callback(self, msg):
        if msg.data and not self.nav_enabled:
            self.get_logger().info("Navigation ENABLED")
            # Capture position at the moment navigation starts (for first ALIGN)
            self.align_hold_x = self.vehicle_state.x
            self.align_hold_y = self.vehicle_state.y
            self.in_align_mode = True
        elif not msg.data and self.nav_enabled:
            self.get_logger().info("Navigation DISABLED")
            self.in_align_mode = False
        self.nav_enabled = msg.data

    def waypoint_callback(self, msg):
        self.current_waypoint = msg
        self.target_altitude = msg.z
        self.have_waypoint = True

    # ------------------------------------------
    # Compute Tanh Yaw Rate
    # ------------------------------------------
    def compute_yaw_rate(self):
        """
        Smooth tanh yaw rate controller.

        heading_error = desired_yaw - current_yaw (both in ENU)
        yaw_rate = MAX_YAW_RATE * tanh(K * heading_error)

        Output is in ENU convention (CCW positive) — MAVROS converts
        to NED internally.
        """
        heading_error = normalize_angle(
            self.guidance_cmd.desired_yaw - self.vehicle_state.yaw
        )

        yaw_rate_enu = MAX_YAW_RATE * math.tanh(YAW_TANH_K * heading_error)

        return yaw_rate_enu

    # ------------------------------------------
    # Publish: ALIGN mode (position hold XY)
    # ------------------------------------------
    def publish_align_setpoint(self):
        """
        ALIGN mode: drone is rotating in place (forward_speed ≈ 0).
        Hold XY position + altitude + command yaw_rate for rotation.
        PX4 position controller prevents drift during rotation.
        """
        setpoint = PositionTarget()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Position XYZ + yaw_rate
        # Ignore: velocities, accelerations, absolute yaw
        setpoint.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
            # YAW_RATE NOT ignored
        )

        # Hold at captured position (ENU)
        setpoint.position.x = self.align_hold_x     # East
        setpoint.position.y = self.align_hold_y     # North
        setpoint.position.z = self.target_altitude  # Up

        # Tanh yaw rate for smooth rotation
        setpoint.yaw_rate = self.compute_yaw_rate()

        self.setpoint_pub.publish(setpoint)

    # ------------------------------------------
    # Publish: MOVE mode (velocity XY)
    # ------------------------------------------
    def publish_move_setpoint(self):
        """
        MOVE mode: drone is flying forward toward waypoint.
        Velocity XY (decomposed from forward speed) + altitude hold + yaw_rate.
        """
        setpoint = PositionTarget()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Velocity XY + Position Z + yaw_rate
        # Ignore: PX, PY, VZ, accelerations, absolute yaw
        setpoint.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            # PZ NOT ignored — altitude hold
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
            # YAW_RATE NOT ignored
        )

        # Forward speed decomposed into ENU
        enu_yaw = self.vehicle_state.yaw
        forward_speed = self.guidance_cmd.forward_speed

        setpoint.velocity.x = forward_speed * math.cos(enu_yaw)   # East
        setpoint.velocity.y = forward_speed * math.sin(enu_yaw)   # North

        # Altitude hold
        setpoint.position.z = self.target_altitude

        # Tanh yaw rate for small heading corrections while moving
        setpoint.yaw_rate = self.compute_yaw_rate()

        self.setpoint_pub.publish(setpoint)

    # ------------------------------------------
    # Main Control Loop
    # ------------------------------------------
    def control_loop(self):
        if not self.nav_enabled:
            return
        if not self.have_mavros_state:
            return
        if not self.mavros_state.connected:
            return
        if not self.have_guidance:
            return
        if not self.have_vehicle_state:
            return

        forward_speed = self.guidance_cmd.forward_speed

        if forward_speed < ALIGN_SPEED_THRESHOLD:
            # ALIGN mode: hold position, rotate in place
            if not self.in_align_mode:
                # Capture current position when entering ALIGN
                self.align_hold_x = self.vehicle_state.x
                self.align_hold_y = self.vehicle_state.y
                self.in_align_mode = True
            self.publish_align_setpoint()
        else:
            # MOVE mode: fly forward
            if self.in_align_mode:
                self.in_align_mode = False
            self.publish_move_setpoint()


def main(args=None):
    rclpy.init(args=args)
    node = CommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
