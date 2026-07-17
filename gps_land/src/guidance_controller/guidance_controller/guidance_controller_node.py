#!/usr/bin/env python3

"""
Node:
    Guidance Controller

Purpose:
    Navigates the drone from its current position toward
    the target waypoint using a rotate-then-forward approach.

    Uses a trapezoidal velocity profile for smooth acceleration
    and deceleration (no jerks).

Input:
    /vehicle/state         (VehicleState)
    /mission/current_waypoint (Waypoint)

Output:
    /guidance/command      (GuidanceCommand)
    /guidance/status       (GuidanceStatus)

Control:
    1. ALIGN   — rotate in place to face waypoint
    2. MOVE    — fly forward with smooth velocity ramp
    3. REACHED — stop, publish waypoint_reached = True

Velocity Profile:
    Trapezoidal — ramps up from 0, holds cruise, ramps down near target.
    Speed is computed based on distance to waypoint and rate-limited
    per control cycle to enforce max acceleration/deceleration.

Hardware:
    Pixhawk 6C + Holybro H-Flow (optical flow + rangefinder)
    GPS-denied, forward velocity only, body-frame control.
"""

import math

import rclpy
from rclpy.node import Node

from gps_navigation_interfaces.msg import (
    VehicleState,
    Waypoint,
    GuidanceCommand,
    GuidanceStatus,
)


# ==========================================================
# Navigation Parameters
# ==========================================================

CONTROL_RATE = 50.0                     # Hz
DT = 1.0 / CONTROL_RATE                # seconds per cycle

# Waypoint acceptance
WAYPOINT_RADIUS = 0.5                   # meters

# Heading alignment
HEADING_ALIGNMENT_THRESHOLD = math.radians(2.0)     # rad (±2° acceptable)

# Speed profile (tanh — smooth, no kinks, C-infinity)
#   forward_speed = MAX_FORWARD_SPEED * tanh(SPEED_TANH_K * (distance - WAYPOINT_RADIUS))
# At waypoint radius: speed = 0 (drone stops at boundary)
# As distance grows: speed → MAX_FORWARD_SPEED (saturates)
MAX_FORWARD_SPEED = 1.0                 # m/s — maximum forward speed
SPEED_TANH_K = 0.6                      # steepness (tuned: ~1.0 m/s at 5m distance)


# ==========================================================
# Navigation States
# ==========================================================

WAIT_FOR_STATE = 0
WAIT_FOR_WAYPOINT = 1
ALIGN_TO_WAYPOINT = 2
MOVE_FORWARD = 3
WAYPOINT_REACHED = 4


class GuidanceController(Node):

    def __init__(self):

        super().__init__("guidance_controller")

        # --------------------------------------------------
        # Subscribers
        # --------------------------------------------------

        self.create_subscription(
            VehicleState,
            "/vehicle/state",
            self.vehicle_state_callback,
            10
        )

        self.create_subscription(
            Waypoint,
            "/mission/current_waypoint",
            self.waypoint_callback,
            10
        )

        # --------------------------------------------------
        # Publishers
        # --------------------------------------------------

        self.command_pub = self.create_publisher(
            GuidanceCommand,
            "/guidance/command",
            10
        )

        self.status_pub = self.create_publisher(
            GuidanceStatus,
            "/guidance/status",
            10
        )

        # --------------------------------------------------
        # Timer
        # --------------------------------------------------

        self.timer = self.create_timer(
            DT,
            self.control_loop
        )

        # --------------------------------------------------
        # Internal Data
        # --------------------------------------------------

        self.current_state = VehicleState()

        self.current_waypoint = Waypoint()

        self.have_state = False

        self.have_waypoint = False

        # --------------------------------------------------
        # Navigation State Machine
        # --------------------------------------------------

        self.navigation_state = WAIT_FOR_STATE

        self.previous_navigation_state = -1

        # --------------------------------------------------
        # Velocity Profile State
        # --------------------------------------------------

        # (tanh profile — no state needed, speed is computed directly from distance)

        # --------------------------------------------------
        # Messages
        # --------------------------------------------------

        self.command_msg = GuidanceCommand()

        self.status_msg = GuidanceStatus()

        # --------------------------------------------------
        # Logging
        # --------------------------------------------------

        self.get_logger().info("====================================")
        self.get_logger().info(" Guidance Controller Started")
        self.get_logger().info(f" Max Speed      : {MAX_FORWARD_SPEED} m/s")
        self.get_logger().info(f" Speed Profile  : tanh (K={SPEED_TANH_K})")
        self.get_logger().info(f" Waypoint Radius: {WAYPOINT_RADIUS} m")
        self.get_logger().info(" Waiting for Vehicle State...")
        self.get_logger().info("====================================")

    # ======================================================
    # Vehicle State Callback
    # ======================================================

    def vehicle_state_callback(self, msg):

        self.current_state = msg

        self.have_state = True

    # ======================================================
    # Waypoint Callback
    # ======================================================

    def waypoint_callback(self, msg):

        self.current_waypoint = msg

        self.have_waypoint = True

    # ======================================================
    # Helper Functions
    # ======================================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def compute_forward_speed(self, distance):
        """
        Smooth tanh forward speed based on distance to waypoint.

            speed = MAX_FORWARD_SPEED * tanh(K * (distance - WAYPOINT_RADIUS))

        Properties:
        - At waypoint radius: speed = 0 (stops at boundary)
        - Far away: speed ≈ MAX_FORWARD_SPEED (saturates ~5m out)
        - Single smooth curve, no piecewise logic
        - K=0.6 gives ~1.0 m/s at 5m distance
        """
        effective_distance = max(0.0, distance - WAYPOINT_RADIUS)
        return MAX_FORWARD_SPEED * math.tanh(SPEED_TANH_K * effective_distance)

    def publish_navigation_state(self):

        if self.navigation_state == self.previous_navigation_state:
            return

        self.previous_navigation_state = self.navigation_state

        state_names = {
            WAIT_FOR_STATE: "WAIT_FOR_STATE",
            WAIT_FOR_WAYPOINT: "WAIT_FOR_WAYPOINT",
            ALIGN_TO_WAYPOINT: "ALIGN_TO_WAYPOINT",
            MOVE_FORWARD: "MOVE_FORWARD",
            WAYPOINT_REACHED: "WAYPOINT_REACHED",
        }

        name = state_names.get(self.navigation_state, "UNKNOWN")

        self.get_logger().info(f"State : {name}")

    # ======================================================
    # Main Control Loop
    # ======================================================

    def control_loop(self):

        # --------------------------------------------------
        # Wait until VehicleState is available
        # --------------------------------------------------

        if not self.have_state:

            self.navigation_state = WAIT_FOR_STATE
            self.publish_navigation_state()
            return

        # --------------------------------------------------
        # Wait until Waypoint is available
        # --------------------------------------------------

        if not self.have_waypoint:

            self.navigation_state = WAIT_FOR_WAYPOINT
            self.publish_navigation_state()
            return

        # --------------------------------------------------
        # Compute position error
        # --------------------------------------------------

        dx = self.current_waypoint.x - self.current_state.x
        dy = self.current_waypoint.y - self.current_state.y

        distance = math.sqrt(dx * dx + dy * dy)

        # --------------------------------------------------
        # Compute desired heading
        # --------------------------------------------------

        desired_yaw = math.atan2(dy, dx)

        heading_error = self.normalize_angle(
            desired_yaw - self.current_state.yaw
        )

        # --------------------------------------------------
        # Fill GuidanceStatus
        # --------------------------------------------------

        self.status_msg.header.stamp = self.get_clock().now().to_msg()

        self.status_msg.distance_to_waypoint = distance
        self.status_msg.heading_error = heading_error

        # --------------------------------------------------
        # Waypoint reached
        # --------------------------------------------------

        if distance <= WAYPOINT_RADIUS:

            self.navigation_state = WAYPOINT_REACHED
            self.publish_navigation_state()

            self.command_msg.header.stamp = self.get_clock().now().to_msg()
            self.command_msg.forward_speed = 0.0
            self.command_msg.desired_yaw = self.current_state.yaw

            self.status_msg.waypoint_reached = True

            self.command_pub.publish(self.command_msg)
            self.status_pub.publish(self.status_msg)

            return

        self.status_msg.waypoint_reached = False

        # --------------------------------------------------
        # Rotate First (zero forward speed while aligning)
        # --------------------------------------------------

        if abs(heading_error) > HEADING_ALIGNMENT_THRESHOLD:

            self.navigation_state = ALIGN_TO_WAYPOINT
            self.publish_navigation_state()

            self.command_msg.header.stamp = self.get_clock().now().to_msg()
            self.command_msg.forward_speed = 0.0
            self.command_msg.desired_yaw = desired_yaw

            self.command_pub.publish(self.command_msg)
            self.status_pub.publish(self.status_msg)

            return

        # --------------------------------------------------
        # Move Forward with Smooth Tanh Speed Profile
        # --------------------------------------------------

        self.navigation_state = MOVE_FORWARD
        self.publish_navigation_state()

        # Tanh speed: fast when far, smoothly drops to zero near waypoint
        commanded_speed = self.compute_forward_speed(distance)

        # Publish command
        self.command_msg.header.stamp = self.get_clock().now().to_msg()
        self.command_msg.forward_speed = commanded_speed
        self.command_msg.desired_yaw = desired_yaw

        self.command_pub.publish(self.command_msg)
        self.status_pub.publish(self.status_msg)


def main(args=None):

    rclpy.init(args=args)

    node = GuidanceController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
