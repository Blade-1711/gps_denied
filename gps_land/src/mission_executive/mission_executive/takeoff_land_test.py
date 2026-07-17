#!/usr/bin/env python3
"""
Takeoff & Land Test — PositionTarget altitude hold during hover.

Flight profile:
  1. CLIMB   — velocity setpoint (TwistStamped), climb at constant rate
  2. HOVER   — PositionTarget position hold (PX4 locks XYZ, no drift)
  3. DESCEND — velocity setpoint (TwistStamped), tanh descent profile

The HOVER phase uses PositionTarget in LOCAL_NED frame with absolute
position (x, y, z) so PX4's position controller holds altitude — same
approach used by command_bridge during navigation.

NOTE: All PositionTarget values are in ENU (MAVROS converts to NED internally).

Usage:
    ros2 run mission_executive takeoff_land_test --ros-args -p alt:=2.0 -p hover:=5.0 -p climb_rate:=0.5

Hardware:
    Pixhawk 6C, Holybro H-Flow, Jetson Orin Nano, Flysky 6CH RC
"""

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State, VfrHud, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from sensor_msgs.msg import Range


# ==========================================================
# Tunable Parameters
# ==========================================================

SETPOINT_RATE = 20.0            # Hz — setpoint publish rate
STATE_MACHINE_RATE = 5.0        # Hz — FSM tick rate
PREFLIGHT_WAIT = 3.0            # seconds to wait after FCU connect
OFFBOARD_TIMEOUT = 8.0          # seconds max to get OFFBOARD
ARM_TIMEOUT = 8.0               # seconds max to arm
CLIMB_TIMEOUT = 25.0            # seconds max to reach altitude
LAND_TIMEOUT = 45.0             # seconds max for landing

# Horizontal drift correction (used during CLIMB and DESCEND only)
KP_HORIZONTAL = 0.6             # P-gain: horizontal position error -> velocity
MAX_HORIZONTAL_VEL = 0.5        # m/s — cap on drift-correction velocity

# Controlled soft descent (same continuous tanh profile as the mission)
# H-Flow reads ~0.21 m at ground contact (sensor 21 cm above ground).
TOUCHDOWN_HEIGHT = 0.21         # meters — rangefinder reading at ground contact
TOUCHDOWN_MARGIN = 0.03         # meters — touchdown when agl <= HEIGHT + margin
DESCENT_TANH_K = 1.2            # descent curve steepness
MAX_DESCENT_VEL = 0.4           # m/s — descent speed up high
MIN_DESCENT_VEL = 0.08          # m/s — gentle touchdown creep


class TakeoffLandTest(Node):

    def __init__(self):

        super().__init__('takeoff_land_node')

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------

        self.declare_parameter('alt', 2.0)
        self.declare_parameter('hover', 5.0)
        self.declare_parameter('climb_rate', 0.5)
        self.declare_parameter('climb_timeout', CLIMB_TIMEOUT)

        self.target_alt = float(self.get_parameter('alt').value)
        self.hover_time = float(self.get_parameter('hover').value)
        self.climb_rate = float(self.get_parameter('climb_rate').value)
        self.climb_timeout = float(self.get_parameter('climb_timeout').value)

        self.get_logger().info(
            f'Config: alt={self.target_alt}m hover={self.hover_time}s '
            f'climb={self.climb_rate}m/s timeout={self.climb_timeout}s'
        )

        # --------------------------------------------------
        # State
        # --------------------------------------------------

        self.current_state = State()
        self.current_alt = 0.0          # rangefinder AGL
        self.current_x = 0.0            # EKF local x (ENU East)
        self.current_y = 0.0            # EKF local y (ENU North)
        self.current_z = 0.0            # EKF local z (ENU Up, raw from pose)
        self.current_yaw = 0.0          # ENU yaw from pose quaternion
        self.hold_x = 0.0               # captured takeoff x (ENU)
        self.hold_y = 0.0               # captured takeoff y (ENU)
        self.hold_z = 0.0               # captured altitude z (ENU, at target)
        self.hold_yaw = 0.0             # captured yaw at hold entry
        self.hold_captured = False
        self.throttle_pct = 0.0
        self.phase = 'init'
        self.phase_start = 0.0

        # --------------------------------------------------
        # QoS
        # --------------------------------------------------

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # --------------------------------------------------
        # Subscribers
        # --------------------------------------------------

        self.create_subscription(
            State, '/mavros/state', self.state_cb, 10)

        self.create_subscription(
            Range, '/mavros/rangefinder_pub', self.range_cb, qos)

        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_pos_cb, qos)

        self.create_subscription(
            VfrHud, '/mavros/vfr_hud', self.vfr_hud_cb, qos)

        # --------------------------------------------------
        # Publishers
        # --------------------------------------------------

        # Velocity setpoint (for CLIMB and DESCEND)
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # PositionTarget setpoint (for HOVER — position hold)
        self.pos_target_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)

        # --------------------------------------------------
        # Service Clients
        # --------------------------------------------------

        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')
        self.cmd_client = self.create_client(
            CommandLong, '/mavros/cmd/command')

        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service(timeout_sec=15.0)
        self.set_mode_client.wait_for_service(timeout_sec=15.0)
        self.cmd_client.wait_for_service(timeout_sec=15.0)
        self.get_logger().info('Services ready.')

        # --------------------------------------------------
        # Timers
        # --------------------------------------------------

        self.setpoint_timer = self.create_timer(
            1.0 / SETPOINT_RATE, self.send_setpoint)

        self.main_timer = self.create_timer(
            1.0 / STATE_MACHINE_RATE, self.main_loop)

    # ==========================================================
    # Callbacks
    # ==========================================================

    def state_cb(self, msg):
        self.current_state = msg

    def range_cb(self, msg):
        if msg.range > msg.min_range and msg.range < msg.max_range:
            self.current_alt = msg.range

    def local_pos_cb(self, msg):
        self.current_x = msg.pose.position.x    # ENU East
        self.current_y = msg.pose.position.y    # ENU North
        self.current_z = msg.pose.position.z    # ENU Up (raw, positive up)

        # Extract yaw from quaternion (ENU frame)
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def vfr_hud_cb(self, msg):
        self.throttle_pct = msg.throttle * 100.0

    # ==========================================================
    # Setpoint Publisher (20 Hz — runs continuously)
    # ==========================================================

    def send_setpoint(self):
        """
        Publishes the appropriate setpoint based on current phase:
        - CLIMB: PositionTarget (position XY hold + velocity Z climb)
        - HOVER: PositionTarget (full position hold XYZ + yaw)
        - DESCEND: TwistStamped velocity (tanh descent + P-hold)
        - Other: zero velocity (keeps OFFBOARD alive)
        """
        if self.phase == 'climb':
            # PositionTarget: PX4 position controller holds XY
            # while climbing at commanded velocity Z
            msg = PositionTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

            # Position XY + Velocity Z + Yaw hold
            # Ignore: PZ (use VZ instead), VX/VY (use PX/PY), accelerations, yaw_rate
            msg.type_mask = (
                PositionTarget.IGNORE_PZ |
                PositionTarget.IGNORE_VX |
                PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )

            # Hold at takeoff XY (ENU, MAVROS converts to NED internally)
            msg.position.x = self.hold_x        # East
            msg.position.y = self.hold_y        # North

            # Climb velocity (ENU: positive = up)
            msg.velocity.z = self.climb_rate

            # Hold heading
            msg.yaw = self.hold_yaw

            self.pos_target_pub.publish(msg)

        elif self.phase == 'hover':
            # PositionTarget: PX4 position controller holds XYZ + yaw
            msg = PositionTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

            # Position hold + yaw hold. Ignore velocities, accelerations, yaw_rate.
            msg.type_mask = (
                PositionTarget.IGNORE_VX |
                PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )

            # All values in ENU (MAVROS converts to NED internally)
            msg.position.x = self.hold_x        # East
            msg.position.y = self.hold_y        # North
            msg.position.z = self.hold_z        # Up (positive = up)
            msg.yaw = self.hold_yaw             # ENU yaw

            self.pos_target_pub.publish(msg)

        elif self.phase == 'descend':
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.z = -self.descent_speed()   # DOWN in ENU
            vx, vy = self.horizontal_hold()
            msg.twist.linear.x = vx
            msg.twist.linear.y = vy
            self.vel_pub.publish(msg)

        else:
            # Pre-arm / other: zero velocity (satisfies OFFBOARD requirement)
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.z = 0.0
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            self.vel_pub.publish(msg)

    def descent_speed(self):
        """
        Continuous smooth descent (tanh) — fast up high, gentle at touchdown.
        Same profile the mission uses for intermediate-waypoint landings.
        """
        height_above_ground = self.current_alt - TOUCHDOWN_HEIGHT
        span = MAX_DESCENT_VEL - MIN_DESCENT_VEL
        v = MIN_DESCENT_VEL + span * math.tanh(
            DESCENT_TANH_K * height_above_ground
        )
        return max(MIN_DESCENT_VEL, min(v, MAX_DESCENT_VEL))

    def horizontal_hold(self):
        """
        P-controller that counters horizontal drift during climb/descend.
        Returns (vx, vy) velocities pulling toward captured takeoff XY.
        Used only during velocity-setpoint phases (CLIMB, DESCEND).
        """
        if not self.hold_captured:
            return 0.0, 0.0

        err_x = self.hold_x - self.current_x
        err_y = self.hold_y - self.current_y

        vx = KP_HORIZONTAL * err_x
        vy = KP_HORIZONTAL * err_y

        # Clamp to max correction velocity
        vx = max(-MAX_HORIZONTAL_VEL, min(vx, MAX_HORIZONTAL_VEL))
        vy = max(-MAX_HORIZONTAL_VEL, min(vy, MAX_HORIZONTAL_VEL))

        return vx, vy

    # ==========================================================
    # Service Calls
    # ==========================================================

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def arm(self, value=True):
        req = CommandBool.Request()
        req.value = value
        self.arming_client.call_async(req)

    def force_disarm(self):
        """Force disarm via MAV_CMD_COMPONENT_ARM_DISARM with force param."""
        req = CommandLong.Request()
        req.command = 400
        req.param1 = 0.0        # disarm
        req.param2 = 21196.0    # force
        self.cmd_client.call_async(req)

    # ==========================================================
    # Main State Machine (5 Hz)
    # ==========================================================

    def main_loop(self):
        now = time.time()

        # ==============================
        # INIT — wait for FCU
        # ==============================
        if self.phase == 'init':
            if not self.current_state.connected:
                return
            self.get_logger().info('[ok] Connected to FCU')
            self.phase = 'preflight'
            self.phase_start = now

        # ==============================
        # PREFLIGHT — wait for sensors
        # ==============================
        elif self.phase == 'preflight':
            if now - self.phase_start < PREFLIGHT_WAIT:
                return

            if self.current_state.armed:
                self.get_logger().error('[ABORT] Already armed — disarm first.')
                self.shutdown_node()
                return

            # Check rangefinder
            if self.current_alt < 0.01:
                self.get_logger().warn('Waiting for rangefinder data...')
                if now - self.phase_start > 10.0:
                    self.get_logger().error('[ABORT] No rangefinder — H-Flow not working.')
                    self.shutdown_node()
                    return
                return

            self.get_logger().info(
                f'[ok] Preflight OK — range={self.current_alt:.2f}m')
            self.phase = 'offboard'
            self.phase_start = now

        # ==============================
        # OFFBOARD — switch mode
        # ==============================
        elif self.phase == 'offboard':
            # Let setpoints stream for 1.5s first
            if now - self.phase_start < 1.5:
                return

            if self.current_state.mode != 'OFFBOARD':
                self.get_logger().info('[..] Switching to OFFBOARD')
                self.set_mode('OFFBOARD')
                if now - self.phase_start > OFFBOARD_TIMEOUT:
                    self.get_logger().error('[ABORT] Cannot enter OFFBOARD')
                    self.shutdown_node()
                    return
                return

            self.get_logger().info('[ok] OFFBOARD mode active')
            self.phase = 'arm'
            self.phase_start = now

        # ==============================
        # ARM
        # ==============================
        elif self.phase == 'arm':
            if not self.current_state.armed:
                self.get_logger().info('[..] Arming')
                self.arm(True)
                if now - self.phase_start > ARM_TIMEOUT:
                    self.get_logger().error('[ABORT] Arming rejected')
                    self.shutdown_node()
                    return
                return

            self.get_logger().info('[ok] ARMED -> climbing')
            # Capture takeoff XY + yaw so we hold it during climb
            self.hold_x = self.current_x
            self.hold_y = self.current_y
            self.hold_yaw = self.current_yaw
            self.hold_captured = True
            self.get_logger().info(
                f'[ok] Holding takeoff XY: x={self.hold_x:.2f} y={self.hold_y:.2f}')
            self.phase = 'climb'
            self.phase_start = now

        # ==============================
        # CLIMB
        # ==============================
        elif self.phase == 'climb':
            alt = self.current_alt
            self.get_logger().info(
                f'   climb  alt={alt:.2f}m  target={self.target_alt}m  '
                f'throttle={self.throttle_pct:.0f}%')

            if alt >= 0.90 * self.target_alt:
                self.get_logger().info(
                    f'[ok] Reached {self.target_alt}m -> hover {self.hover_time}s')

                # Capture position for PositionTarget hold
                # Use current ENU position (MAVROS converts to NED internally)
                self.hold_x = self.current_x        # ENU East
                self.hold_y = self.current_y        # ENU North
                self.hold_z = self.current_z        # ENU Up (current altitude)
                self.hold_yaw = self.current_yaw    # ENU yaw (hold heading)

                self.get_logger().info(
                    f'[ok] Hold position: x={self.hold_x:.2f} y={self.hold_y:.2f} '
                    f'z={self.hold_z:.2f} yaw={math.degrees(self.hold_yaw):.0f}°')

                self.phase = 'hover'
                self.phase_start = now
                return

        # ==============================
        # HOVER — PX4 position controller holds XYZ + yaw
        # ==============================
        elif self.phase == 'hover':
            alt = self.current_alt
            elapsed = now - self.phase_start
            self.get_logger().info(
                f'   hover  alt={alt:.2f}m  t={elapsed:.1f}/{self.hover_time}s  '
                f'throttle={self.throttle_pct:.0f}%')

            if elapsed >= self.hover_time:
                self.get_logger().info('[..] Controlled descent')
                # Re-capture XY for horizontal_hold during descent
                self.hold_x = self.current_x
                self.hold_y = self.current_y
                self.phase = 'descend'
                self.phase_start = now

        # ==============================
        # DESCEND — controlled soft descent (velocity, stays in OFFBOARD)
        # ==============================
        elif self.phase == 'descend':
            alt = self.current_alt
            self.get_logger().info(
                f'   descend  alt={alt:.2f}m  v={self.descent_speed():.2f}m/s  '
                f'throttle={self.throttle_pct:.0f}%  '
                f'pos=({self.current_x:.2f}, {self.current_y:.2f})')

            if alt <= (TOUCHDOWN_HEIGHT + TOUCHDOWN_MARGIN):
                self.get_logger().info(
                    f'[ok] Touchdown (AGL={alt:.2f}m) -> force disarm')
                self.force_disarm()
                self.phase = 'wait_disarm'
                self.phase_start = now
                return

            if now - self.phase_start > LAND_TIMEOUT:
                self.get_logger().warn('[!] Descent timeout -> force disarm')
                self.force_disarm()
                self.phase = 'wait_disarm'
                self.phase_start = now

        # ==============================
        # WAIT_DISARM — wait for force disarm to take effect
        # ==============================
        elif self.phase == 'wait_disarm':
            if not self.current_state.armed:
                self.get_logger().info('[ok] Disarmed. Test complete.')
                self.phase = 'done'
                self.shutdown_node()
                return

            elapsed = now - self.phase_start
            if elapsed > 5.0:
                # Retry force disarm
                self.get_logger().warn('[!] Retrying force disarm...')
                self.force_disarm()
                self.phase_start = now

        # ==============================
        # DONE
        # ==============================
        elif self.phase == 'done':
            pass

    def shutdown_node(self):
        self.setpoint_timer.cancel()
        self.main_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffLandTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().warn('Interrupted — controlled descent + disarm')

        # Use controlled descent (same tanh profile).
        # Keep spinning so callbacks update sensor data while we descend.
        import time as _time
        deadline = _time.time() + 45.0
        landed = False

        while _time.time() < deadline:
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except (KeyboardInterrupt, SystemExit):
                # Double Ctrl+C → force disarm immediately
                node.get_logger().warn("Double interrupt — force disarm")
                node.force_disarm()
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.2)
                break

            # Publish descent velocity setpoint
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            vx, vy = node.horizontal_hold()
            msg.twist.linear.x = vx
            msg.twist.linear.y = vy
            msg.twist.linear.z = -node.descent_speed()
            node.vel_pub.publish(msg)

            # Check touchdown
            if node.current_alt <= (TOUCHDOWN_HEIGHT + TOUCHDOWN_MARGIN):
                node.get_logger().info('[ok] Touchdown -> force disarm')
                node.force_disarm()
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.2)
                    if not node.current_state.armed:
                        node.get_logger().info('[ok] Disarmed after interrupt.')
                        landed = True
                        break
                break

        if not landed and node.current_state.armed:
            node.get_logger().warn('Timeout — force disarm')
            node.force_disarm()
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.2)

    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
