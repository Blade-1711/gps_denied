#!/usr/bin/env python3
"""
Mission Executive — GPS + Precision Landing Integration

Orchestrates: OFFBOARD → ARM → TAKEOFF → NAVIGATE → PRECISION LAND → RETURN HOME

During NAVIGATE, the camera actively scans for ArUco markers. If a marker is
detected (and it's not one we've already landed on), the drone stops navigation
and hands off to the precision_landing_node for visual alignment + descent.

If no marker is detected mid-transit, the drone reaches the waypoint via guidance,
then enters SEARCH_HOVER to scan. If still not found, climbs in steps up to max
scan altitude.

After each landing, pose correction resets drift: the next leg's target is computed
as current_position + relative_vector(WP_n → WP_n+1).
"""

import json
import math
import time as _time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from gps_navigation_interfaces.msg import (
    VehicleState,
    Waypoint,
    GuidanceStatus,
)

from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import Range, Image
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.msg import VfrHud
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import SetMode, CommandBool, CommandLong
from cv_bridge import CvBridge


# ==========================================================
# Mission States
# ==========================================================

STATE_IDLE = "IDLE"
STATE_PREFLIGHT = "PREFLIGHT"
STATE_STREAM_SETPOINTS = "STREAM_SETPOINTS"
STATE_SET_OFFBOARD = "SET_OFFBOARD"
STATE_ARM = "ARM"
STATE_CLIMB = "CLIMB"
STATE_HOLD = "HOLD"
STATE_NAVIGATE = "NAVIGATE"
STATE_SEARCH_HOVER = "SEARCH_HOVER"
STATE_SEARCH_CLIMB = "SEARCH_CLIMB"
STATE_PRE_LAND_HOVER = "PRE_LAND_HOVER"
STATE_PRECISION_LAND_HANDOFF = "PRECISION_LAND_HANDOFF"
STATE_WAIT_FOR_LANDING = "WAIT_FOR_LANDING"
STATE_POST_LAND = "POST_LAND"
STATE_BLIND_LAND = "BLIND_LAND"
STATE_LANDED = "LANDED"
STATE_FINAL_LAND = "FINAL_LAND"
STATE_DISARM = "DISARM"
STATE_COMPLETE = "COMPLETE"
STATE_ABORT = "ABORT"
STATE_ERROR = "ERROR"


# ==========================================================
# Tunable Parameters
# ==========================================================

CONTROL_RATE = 20.0

# Takeoff
DEFAULT_MISSION_ALTITUDE = 3.0
CLIMB_VELOCITY = 0.4
ALTITUDE_REACHED_FRACTION = 0.95
INITIAL_STREAM_COUNT = 50
HOLD_STABILIZE_TIME = 2.0
CLIMB_TIMEOUT = 25.0

# Horizontal drift correction
KP_HORIZONTAL = 0.6
MAX_HORIZONTAL_VEL = 0.5

# Landing (blind fallback)
TOUCHDOWN_HEIGHT = 0.21
TOUCHDOWN_MARGIN = 0.03
DESCENT_TANH_K = 1.2
MAX_DESCENT_VEL = 0.4
MIN_DESCENT_VEL = 0.08
DESCEND_TIMEOUT = 40.0
POST_LAND_WAIT = 3.0

# Navigation
WAYPOINT_REACHED_CONFIRMATIONS = 10

# Precision landing integration
PRE_LAND_HOVER_TIME = 3.0
NAV_MARKER_CONFIRM = 3
EXCLUSION_RADIUS = 1.5
SEARCH_HOVER_TIMEOUT = 3.0
MAX_SCAN_ALTITUDE = 5.0
SCAN_CLIMB_STEP = 1.0

# Camera FOV — UPDATE AFTER CALIBRATION (must match precision_landing_node values)
CAMERA_FOV_H_DEG = 70.0
CAMERA_FOV_V_DEG = 43.0
CAMERA_HFOV_TAN = math.tan(math.radians(CAMERA_FOV_H_DEG / 2.0))
CAMERA_VFOV_TAN = math.tan(math.radians(CAMERA_FOV_V_DEG / 2.0))

# Logging
LOG_INTERVAL = 2.0
TELEM_INTERVAL = 0.5


# ==========================================================
# Mission Executive Node
# ==========================================================

class MissionExecutive(Node):

    def __init__(self):
        super().__init__("mission_executive")

        # ROS Parameters
        self.declare_parameter("mission_altitude", DEFAULT_MISSION_ALTITUDE)
        self.declare_parameter("climb_velocity", CLIMB_VELOCITY)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("return_home", True)
        self.declare_parameter("detection_mode", "aruco")   # "aruco", "color", or "model"
        self.declare_parameter("model_path", "")            # path to YOLOv8 .pt or .engine

        self.mission_altitude = float(self.get_parameter("mission_altitude").value)
        self.climb_velocity = float(self.get_parameter("climb_velocity").value)
        self.waypoints_file = str(self.get_parameter("waypoints_file").value)
        self.return_home = bool(self.get_parameter("return_home").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.detection_mode = str(self.get_parameter("detection_mode").value)
        self.model_path = str(self.get_parameter("model_path").value)

        # QoS
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # State Machine
        self.state = STATE_IDLE
        self.previous_state = None
        self.state_entry_time = self.get_clock().now()

        # Mission Data
        self.current_wp_index = 0
        self.total_waypoints = 0
        self.waypoints_received = False
        self.mission_waypoints = []

        # Sensor Data
        self.fcu_connected = False
        self.fcu_armed = False
        self.fcu_mode = ""
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_z_enu = 0.0
        self.current_yaw = 0.0
        self.position_ready = False
        self.current_agl = 0.0
        self.rangefinder_ready = False
        self.current_throttle = 0.0

        # Takeoff Reference
        self.takeoff_x = 0.0
        self.takeoff_y = 0.0
        self.takeoff_position_captured = False
        self.hold_z_enu = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_captured = False
        self.stream_counter = 0

        # Navigation (relative vector approach)
        self.waypoint_reached_count = 0
        self.is_final_landing = False
        self.current_leg_target_x = 0.0
        self.current_leg_target_y = 0.0

        # Precision landing integration
        self.bridge = CvBridge()
        self.landed_positions = []
        self.exclusion_positions = []
        self.nav_marker_frames = 0
        self.precision_landing_status = "IDLE"
        self.scan_target_alt = 0.0

        # ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # YOLO model (for detection_mode == "model")
        self.yolo_model = None
        if self.detection_mode == 'model':
            if self.model_path:
                try:
                    from ultralytics import YOLO
                    self.yolo_model = YOLO(self.model_path)
                    self.get_logger().info(f"YOLOv8 model loaded: {self.model_path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to load YOLO model '{self.model_path}': {e}")
                    self.get_logger().error("Falling back to aruco detection")
                    self.detection_mode = 'aruco'
            else:
                self.get_logger().error("detection_mode='model' but no model_path specified!")
                self.get_logger().error("Falling back to aruco detection")
                self.detection_mode = 'aruco'

        # Color detection HSV range (loaded from waypoints file if available)
        self.hsv_low = np.array([0, 100, 100])
        self.hsv_high = np.array([10, 255, 255])

        # Timing
        self.last_log_time = self.get_clock().now()
        self.last_telem_time = self.get_clock().now()

        # Subscribers
        self.create_subscription(MavrosState, "/mavros/state", self.mavros_state_cb, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.pose_cb, self.sensor_qos)
        self.create_subscription(Range, "/mavros/rangefinder_pub", self.range_cb, self.sensor_qos)
        self.create_subscription(VfrHud, "/mavros/vfr_hud", self.vfr_cb, 10)
        self.create_subscription(GuidanceStatus, "/guidance/status", self.guidance_status_cb, 10)
        self.create_subscription(Int32, "/mission/total_waypoints", self.total_waypoints_cb, 10)
        self.create_subscription(Waypoint, "/gps_to_local/waypoints", self.waypoint_store_cb, 10)
        self.create_subscription(String, "/mission/start", self.start_cb, 10)
        self.create_subscription(Bool, "/mission/abort", self.abort_cb, 10)
        self.create_subscription(Image, "/down/image_raw", self.nav_image_cb, self.sensor_qos)
        self.create_subscription(String, "/precision_landing/status", self.pl_status_cb, 10)

        # Publishers
        self.vel_pub = self.create_publisher(TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self.pos_target_pub = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)
        self.waypoint_pub = self.create_publisher(Waypoint, "/mission/current_waypoint", 10)
        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.nav_enabled_pub = self.create_publisher(Bool, "/mission/nav_enabled", 10)
        self.pl_activate_pub = self.create_publisher(Bool, "/precision_landing/activate", 10)

        # Service Clients
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cmd_client = self.create_client(CommandLong, "/mavros/cmd/command")

        # Load waypoints
        if self.waypoints_file:
            self.load_waypoints_file(self.waypoints_file)

        # Timer
        self.timer = self.create_timer(1.0 / CONTROL_RATE, self.state_machine)

        self.get_logger().info("=" * 50)
        self.get_logger().info(" Mission Executive (GPS + Precision Landing)")
        self.get_logger().info(f"  Altitude     : {self.mission_altitude} m")
        self.get_logger().info(f"  Return Home  : {self.return_home}")
        self.get_logger().info(f"  Detection    : {self.detection_mode}")
        if self.detection_mode == 'model':
            self.get_logger().info(f"  Model Path   : {self.model_path}")
        self.get_logger().info(f"  Max Scan Alt : {MAX_SCAN_ALTITUDE} m")
        self.get_logger().info(f"  Exclusion R  : {EXCLUSION_RADIUS} m")
        self.get_logger().info(f"  Camera Stream: http://<jetson_ip>:5000")
        self.get_logger().info("=" * 50)


    # ==========================================================
    # Callbacks
    # ==========================================================

    def mavros_state_cb(self, msg):
        if msg.connected and not self.fcu_connected:
            self.get_logger().info("FCU Connected")
        self.fcu_connected = msg.connected
        self.fcu_armed = msg.armed
        self.fcu_mode = msg.mode

    def pose_cb(self, msg):
        z = msg.pose.position.z
        if not math.isnan(z):
            self.current_x = msg.pose.position.x
            self.current_y = msg.pose.position.y
            self.current_z = -z
            self.current_z_enu = z
            q = msg.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
            if not self.position_ready:
                self.get_logger().info("Local Position Connected")
            self.position_ready = True

    def range_cb(self, msg):
        if msg.range > msg.min_range and msg.range < msg.max_range:
            self.current_agl = float(msg.range)
            if not self.rangefinder_ready:
                self.get_logger().info("Rangefinder Connected")
            self.rangefinder_ready = True

    def vfr_cb(self, msg):
        self.current_throttle = float(msg.throttle)

    def guidance_status_cb(self, msg):
        if msg.waypoint_reached:
            self.waypoint_reached_count += 1
        else:
            self.waypoint_reached_count = 0

    def total_waypoints_cb(self, msg):
        if not self.waypoints_received:
            self.total_waypoints = msg.data
            self.waypoints_received = True
            self.get_logger().info(f"Mission received: {self.total_waypoints} waypoints")

    def waypoint_store_cb(self, msg):
        while len(self.mission_waypoints) <= msg.id:
            self.mission_waypoints.append(None)
        self.mission_waypoints[msg.id] = msg

    def start_cb(self, msg):
        if self.state == STATE_IDLE:
            self.get_logger().info("Start command received!")
            self.transition_to(STATE_PREFLIGHT)

    def abort_cb(self, msg):
        if not msg.data:
            return
        if self.state in (STATE_ABORT, STATE_COMPLETE, STATE_DISARM, STATE_IDLE):
            return
        self.get_logger().warn("!!! ABORT received — controlled descent !!!")
        # Also deactivate precision landing if active
        deactivate = Bool()
        deactivate.data = False
        self.pl_activate_pub.publish(deactivate)
        self.transition_to(STATE_ABORT)

    def pl_status_cb(self, msg):
        self.precision_landing_status = msg.data

    def nav_image_cb(self, msg):
        """Marker detection during NAVIGATE and SEARCH_HOVER states."""
        if self.state not in (STATE_NAVIGATE, STATE_SEARCH_HOVER):
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        detection = self._detect_marker(frame)
        if detection is None:
            self.nav_marker_frames = 0
            return

        cx, cy, area = detection
        # Estimate marker ENU position
        marker_enu = self._estimate_marker_enu(cx, cy)

        # Exclusion check
        if self._near_landed_position(marker_enu):
            self.nav_marker_frames = 0
            return

        self.nav_marker_frames += 1
        if self.nav_marker_frames >= NAV_MARKER_CONFIRM:
            self.get_logger().info(
                f"Marker detected during {self.state}! Transitioning to precision landing.")
            self.nav_marker_frames = 0
            self.takeoff_x = self.current_x
            self.takeoff_y = self.current_y
            self.transition_to(STATE_PRE_LAND_HOVER)


    # ==========================================================
    # Detection & Estimation Helpers
    # ==========================================================

    def _detect_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            return None
        c = corners[0][0]
        cx = np.mean(c[:, 0])
        cy = np.mean(c[:, 1])
        area = cv2.contourArea(c.astype(np.int32))
        return (cx, cy, area)

    def _detect_color(self, frame):
        """Detect marker using HSV color filtering."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 200:
            return None
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None
        return (M["m10"] / M["m00"], M["m01"] / M["m00"], area)

    def _detect_model(self, frame):
        """Detect marker using YOLOv8 model inference."""
        if self.yolo_model is None:
            return None
        try:
            results = self.yolo_model.predict(frame, conf=0.5, verbose=False)
            if results and len(results[0].boxes) > 0:
                boxes = results[0].boxes
                best_idx = boxes.conf.argmax()
                x1, y1, x2, y2 = boxes.xyxy[best_idx].cpu().numpy()
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                area = (x2 - x1) * (y2 - y1)
                return (float(cx), float(cy), float(area))
        except Exception:
            pass
        return None

    def _detect_marker(self, frame):
        """General marker detection dispatcher based on detection_mode."""
        if self.detection_mode == 'aruco':
            return self._detect_aruco(frame)
        elif self.detection_mode == 'color':
            return self._detect_color(frame)
        elif self.detection_mode == 'model':
            return self._detect_model(frame)
        return self._detect_aruco(frame)

    def _estimate_marker_enu(self, cx, cy):
        """Estimate marker world position from pixel coords + altitude + yaw."""
        cam_err_x = (cx - 320.0) / 320.0
        cam_err_y = (cy - 240.0) / 240.0
        body_x = cam_err_y
        body_y = -cam_err_x
        yaw = self.current_yaw
        offset_e = body_x * self.current_agl * CAMERA_VFOV_TAN * math.cos(yaw) + \
                   body_y * self.current_agl * CAMERA_HFOV_TAN * math.sin(yaw)
        offset_n = body_x * self.current_agl * CAMERA_VFOV_TAN * math.sin(yaw) - \
                   body_y * self.current_agl * CAMERA_HFOV_TAN * math.cos(yaw)
        return (self.current_x + offset_e, self.current_y + offset_n)

    def _near_landed_position(self, marker_enu):
        mx, my = marker_enu
        for (lx, ly) in self.exclusion_positions:
            if math.sqrt((mx - lx)**2 + (my - ly)**2) < EXCLUSION_RADIUS:
                return True
        return False

    # ==========================================================
    # Waypoint & Navigation Helpers
    # ==========================================================

    def load_waypoints_file(self, path):
        try:
            with open(path, "r") as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            self.get_logger().error(f"Could not read waypoints file '{path}': {e}")
            return
        wp_list = data.get("waypoints", [])
        self.total_waypoints = int(data.get("num_waypoints", 0))
        self.mission_waypoints = []
        for entry in wp_list:
            wp = Waypoint()
            wp.id = int(entry["id"])
            wp.x = float(entry["x"])
            wp.y = float(entry["y"])
            wp.z = float(entry["z"])
            while len(self.mission_waypoints) <= wp.id:
                self.mission_waypoints.append(None)
            self.mission_waypoints[wp.id] = wp
        self.waypoints_received = True
        self.get_logger().info(f"Loaded {self.total_waypoints} waypoints from file")

        # Load detection config from waypoints file (if present)
        if "detection_mode" in data:
            file_mode = str(data["detection_mode"])
            # Only override if not already set via launch parameter
            if self.detection_mode == 'aruco' and file_mode != 'aruco':
                self.detection_mode = file_mode
                self.get_logger().info(f"Detection mode from file: {self.detection_mode}")
        if "model_path" in data and data["model_path"]:
            if not self.model_path:
                self.model_path = str(data["model_path"])
                # Load model now if needed
                if self.detection_mode == 'model' and self.yolo_model is None:
                    try:
                        from ultralytics import YOLO
                        self.yolo_model = YOLO(self.model_path)
                        self.get_logger().info(f"YOLOv8 model loaded from file config: {self.model_path}")
                    except Exception as e:
                        self.get_logger().error(f"Failed to load YOLO model: {e}")
                        self.detection_mode = 'aruco'
        if "color_hsv_low" in data:
            self.hsv_low = np.array(data["color_hsv_low"], dtype=np.uint8)
        if "color_hsv_high" in data:
            self.hsv_high = np.array(data["color_hsv_high"], dtype=np.uint8)

    def compute_leg_target(self):
        """
        Compute target in EKF frame using relative vector approach.
        target = current_position + (WP_next - WP_prev)

        Also rebuilds exclusion_positions: all landed positions EXCEPT
        the current target's vicinity. This allows detecting the target
        waypoint's marker while ignoring all previously-visited markers.
        """
        if self.current_wp_index == 0:
            # Return home: vector from last WP to home
            last_wp = self.mission_waypoints[self.total_waypoints]
            home = self.mission_waypoints[0]
            dx = home.x - last_wp.x
            dy = home.y - last_wp.y
        elif self.current_wp_index == 1:
            # First leg: home → WP1
            home = self.mission_waypoints[0]
            wp1 = self.mission_waypoints[1]
            dx = wp1.x - home.x
            dy = wp1.y - home.y
        else:
            # WP_n-1 → WP_n
            prev_wp = self.mission_waypoints[self.current_wp_index - 1]
            curr_wp = self.mission_waypoints[self.current_wp_index]
            dx = curr_wp.x - prev_wp.x
            dy = curr_wp.y - prev_wp.y

        self.current_leg_target_x = self.current_x + dx
        self.current_leg_target_y = self.current_y + dy

        # Rebuild exclusion list: exclude all landed positions EXCEPT
        # positions near the current target (so we CAN detect that marker)
        target_x = self.current_leg_target_x
        target_y = self.current_leg_target_y
        self.exclusion_positions = []
        for (lx, ly) in self.landed_positions:
            dist_to_target = math.sqrt((lx - target_x)**2 + (ly - target_y)**2)
            if dist_to_target > EXCLUSION_RADIUS:
                # This landed position is far from our target — exclude its marker
                self.exclusion_positions.append((lx, ly))
            # If close to target — don't exclude (we want to detect it)

        self.get_logger().info(
            f"Leg target: current({self.current_x:.2f},{self.current_y:.2f}) + "
            f"delta({dx:.2f},{dy:.2f}) = ({self.current_leg_target_x:.2f},{self.current_leg_target_y:.2f}) | "
            f"Exclusions: {len(self.exclusion_positions)} positions")

    def target_altitude(self):
        if 0 <= self.current_wp_index < len(self.mission_waypoints):
            wp = self.mission_waypoints[self.current_wp_index]
            if wp is not None:
                return float(wp.z)
        return self.mission_altitude

    # ==========================================================
    # State Helpers
    # ==========================================================

    def transition_to(self, new_state):
        self.previous_state = self.state
        self.state = new_state
        self.state_entry_time = self.get_clock().now()
        self.get_logger().info(f"State: {self.previous_state} → {new_state}")
        state_msg = String()
        state_msg.data = new_state
        self.state_pub.publish(state_msg)

    def time_in_state(self):
        elapsed = self.get_clock().now() - self.state_entry_time
        return elapsed.nanoseconds / 1e9

    def log_throttled(self, message):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 >= LOG_INTERVAL:
            self.get_logger().info(message)
            self.last_log_time = now

    def log_telem(self, message):
        now = self.get_clock().now()
        if (now - self.last_telem_time).nanoseconds / 1e9 >= TELEM_INTERVAL:
            self.get_logger().info(message)
            self.last_telem_time = now

    # ==========================================================
    # Service Calls
    # ==========================================================

    def request_offboard(self):
        if not self.mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = "OFFBOARD"
        self.mode_client.call_async(req)

    def request_arm(self):
        if not self.arm_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = True
        self.arm_client.call_async(req)

    def request_disarm(self):
        if not self.arm_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = False
        self.arm_client.call_async(req)

    def force_disarm(self):
        if not self.cmd_client.service_is_ready():
            return
        req = CommandLong.Request()
        req.command = 400
        req.param1 = 0.0
        req.param2 = 21196.0
        self.cmd_client.call_async(req)

    # ==========================================================
    # Setpoint Publishing
    # ==========================================================

    def publish_idle_setpoint(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.vel_pub.publish(msg)

    def horizontal_hold(self):
        err_x = self.takeoff_x - self.current_x
        err_y = self.takeoff_y - self.current_y
        vx = max(-MAX_HORIZONTAL_VEL, min(MAX_HORIZONTAL_VEL, KP_HORIZONTAL * err_x))
        vy = max(-MAX_HORIZONTAL_VEL, min(MAX_HORIZONTAL_VEL, KP_HORIZONTAL * err_y))
        return vx, vy

    def publish_climb_setpoint(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        vx, vy = self.horizontal_hold()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = self.climb_velocity
        self.vel_pub.publish(msg)

    def publish_hold_setpoint(self):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE)
        msg.position.x = self.takeoff_x
        msg.position.y = self.takeoff_y
        msg.position.z = self.hold_z_enu
        msg.yaw = float(self.current_yaw)
        self.pos_target_pub.publish(msg)

    def publish_hold_at_altitude(self, target_z_enu):
        """Hold XY + specific altitude (for SEARCH_CLIMB)."""
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE)
        msg.position.x = self.takeoff_x
        msg.position.y = self.takeoff_y
        msg.position.z = target_z_enu
        msg.yaw = float(self.current_yaw)
        self.pos_target_pub.publish(msg)

    def descent_speed_value(self):
        height_above_ground = self.current_agl - TOUCHDOWN_HEIGHT
        span = MAX_DESCENT_VEL - MIN_DESCENT_VEL
        v = MIN_DESCENT_VEL + span * math.tanh(DESCENT_TANH_K * height_above_ground)
        return max(MIN_DESCENT_VEL, min(v, MAX_DESCENT_VEL))

    def publish_descend_setpoint(self):
        descent = self.descent_speed_value()
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        vx, vy = self.horizontal_hold()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = -descent
        self.vel_pub.publish(msg)


    # ==========================================================
    # Main State Machine
    # ==========================================================

    def state_machine(self):

        # Publish state and nav_enabled gate
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        nav_msg = Bool()
        nav_msg.data = (self.state == STATE_NAVIGATE)
        self.nav_enabled_pub.publish(nav_msg)

        # ── IDLE ──
        if self.state == STATE_IDLE:
            if self.auto_start and self.waypoints_received:
                self.transition_to(STATE_PREFLIGHT)

        # ── PREFLIGHT ──
        elif self.state == STATE_PREFLIGHT:
            if not self.fcu_connected:
                self.log_throttled("Waiting for FCU...")
                return
            if not self.position_ready:
                self.log_throttled("Waiting for local position...")
                return
            if not self.rangefinder_ready:
                self.log_throttled("Waiting for rangefinder...")
                return
            if not self.waypoints_received:
                self.log_throttled("Waiting for waypoints...")
                return
            stored = sum(1 for w in self.mission_waypoints if w is not None)
            if stored < (self.total_waypoints + 1):
                self.log_throttled(f"Loading waypoints... {stored}/{self.total_waypoints + 1}")
                return

            self.get_logger().info("Preflight OK!")
            self.takeoff_x = self.current_x
            self.takeoff_y = self.current_y
            self.takeoff_position_captured = True
            self.origin_x = self.current_x
            self.origin_y = self.current_y
            self.origin_captured = True
            self.current_wp_index = 1
            self.stream_counter = 0
            # Add home to landed_positions (prevent landing on home marker during outbound legs)
            self.landed_positions.append((self.current_x, self.current_y))
            self.exclusion_positions = list(self.landed_positions)
            self.transition_to(STATE_STREAM_SETPOINTS)

        # ── STREAM_SETPOINTS ──
        elif self.state == STATE_STREAM_SETPOINTS:
            self.publish_idle_setpoint()
            self.stream_counter += 1
            if self.stream_counter >= INITIAL_STREAM_COUNT:
                self.get_logger().info(f"Streamed {INITIAL_STREAM_COUNT} setpoints")
                self.transition_to(STATE_SET_OFFBOARD)

        # ── SET_OFFBOARD ──
        elif self.state == STATE_SET_OFFBOARD:
            self.publish_idle_setpoint()
            if self.fcu_mode == "OFFBOARD":
                self.get_logger().info("OFFBOARD enabled")
                self.transition_to(STATE_ARM)
            else:
                self.request_offboard()
                self.log_throttled("Requesting OFFBOARD...")

        # ── ARM ──
        elif self.state == STATE_ARM:
            self.publish_idle_setpoint()
            if self.fcu_armed:
                self.get_logger().info("Vehicle armed!")
                self.transition_to(STATE_CLIMB)
            else:
                self.request_arm()
                self.log_throttled("Requesting ARM...")

        # ── CLIMB ──
        elif self.state == STATE_CLIMB:
            self.publish_climb_setpoint()
            target_alt = self.target_altitude()
            if self.current_agl >= (ALTITUDE_REACHED_FRACTION * target_alt):
                self.get_logger().info(f"Altitude reached: {self.current_agl:.2f}m")
                self.hold_z_enu = self.current_z_enu
                self.takeoff_x = self.current_x
                self.takeoff_y = self.current_y
                self.transition_to(STATE_HOLD)
            elif self.time_in_state() > CLIMB_TIMEOUT:
                self.get_logger().warn("Climb timeout — aborting")
                self.takeoff_x = self.current_x
                self.takeoff_y = self.current_y
                self.transition_to(STATE_ABORT)
            else:
                self.log_telem(
                    f"CLIMB | AGL={self.current_agl:.2f}/{target_alt:.2f}m | "
                    f"pos=({self.current_x - self.origin_x:.2f}, {self.current_y - self.origin_y:.2f}) | "
                    f"yaw={math.degrees(self.current_yaw):.0f}°")

        # ── HOLD ──
        elif self.state == STATE_HOLD:
            self.publish_hold_setpoint()
            if self.time_in_state() >= HOLD_STABILIZE_TIME:
                self.get_logger().info("Hold complete. Starting navigation.")
                self.waypoint_reached_count = 0
                self.compute_leg_target()
                self.transition_to(STATE_NAVIGATE)

        # ── NAVIGATE ──
        elif self.state == STATE_NAVIGATE:
            # Publish target waypoint for guidance_controller
            target_wp = Waypoint()
            target_wp.id = self.current_wp_index
            target_wp.x = self.current_leg_target_x
            target_wp.y = self.current_leg_target_y
            target_wp.z = self.hold_z_enu
            self.waypoint_pub.publish(target_wp)

            # Check waypoint reached (guidance confirms)
            if self.waypoint_reached_count >= WAYPOINT_REACHED_CONFIRMATIONS:
                self.get_logger().info(f"WP{self.current_wp_index} reached! Searching for marker...")
                self.waypoint_reached_count = 0
                self.takeoff_x = self.current_x
                self.takeoff_y = self.current_y
                self.transition_to(STATE_SEARCH_HOVER)
            else:
                # Detailed navigation telemetry
                dx = self.current_leg_target_x - self.current_x
                dy = self.current_leg_target_y - self.current_y
                dist = math.sqrt(dx*dx + dy*dy)
                target_yaw = math.atan2(dy, dx)

                # Heading error (shortest path, normalized to [-pi, pi])
                heading_error = target_yaw - self.current_yaw
                while heading_error > math.pi:
                    heading_error -= 2.0 * math.pi
                while heading_error < -math.pi:
                    heading_error += 2.0 * math.pi

                # Rotation direction
                if abs(heading_error) < math.radians(5.0):
                    direction = "ALIGNED ✓"
                elif heading_error > 0:
                    direction = "← CCW (turn LEFT)"
                else:
                    direction = "→ CW (turn RIGHT)"

                cur_deg = math.degrees(self.current_yaw)
                tgt_deg = math.degrees(target_yaw)
                err_deg = math.degrees(heading_error)

                self.log_telem(
                    f"NAV WP{self.current_wp_index} | "
                    f"pos=({self.current_x - self.origin_x:.2f}, {self.current_y - self.origin_y:.2f}) "
                    f"target=({self.current_leg_target_x - self.origin_x:.2f}, {self.current_leg_target_y - self.origin_y:.2f}) | "
                    f"dist={dist:.2f}m | AGL={self.current_agl:.2f}m | "
                    f"yaw {cur_deg:.0f}°→{tgt_deg:.0f}° err={err_deg:.0f}° "
                    f"| {direction}")


        # ── SEARCH_HOVER — hold position, scan for marker ──
        elif self.state == STATE_SEARCH_HOVER:
            self.publish_hold_setpoint()
            # Marker detection happens in nav_image_cb (active for this state)
            # If marker found, nav_image_cb transitions to PRE_LAND_HOVER
            if self.time_in_state() > SEARCH_HOVER_TIMEOUT:
                # Not found at this altitude — climb higher
                if self.current_agl < MAX_SCAN_ALTITUDE - SCAN_CLIMB_STEP:
                    self.scan_target_alt = self.current_agl + SCAN_CLIMB_STEP
                    self.get_logger().info(
                        f"Marker not found at {self.current_agl:.1f}m → "
                        f"climbing to {self.scan_target_alt:.1f}m")
                    self.transition_to(STATE_SEARCH_CLIMB)
                else:
                    self.get_logger().warn("Max scan altitude — blind landing")
                    self.transition_to(STATE_BLIND_LAND)

        # ── SEARCH_CLIMB — climb +1m to widen FOV ──
        elif self.state == STATE_SEARCH_CLIMB:
            target_z = self.hold_z_enu + (self.scan_target_alt - self.target_altitude())
            self.publish_hold_at_altitude(target_z)
            if self.current_agl >= self.scan_target_alt * 0.95:
                self.get_logger().info(f"Reached {self.current_agl:.2f}m — scanning...")
                self.hold_z_enu = self.current_z_enu
                self.transition_to(STATE_SEARCH_HOVER)

        # ── PRE_LAND_HOVER — kill momentum before precision landing handoff ──
        elif self.state == STATE_PRE_LAND_HOVER:
            self.publish_hold_setpoint()
            if self.time_in_state() >= PRE_LAND_HOVER_TIME:
                self.get_logger().info("Pre-land hover done — handing off to precision landing")
                self.transition_to(STATE_PRECISION_LAND_HANDOFF)
            else:
                self.log_telem(
                    f"PRE_LAND_HOVER | t={self.time_in_state():.1f}/{PRE_LAND_HOVER_TIME:.1f}s | "
                    f"pos=({self.current_x - self.origin_x:.2f}, {self.current_y - self.origin_y:.2f}) | "
                    f"AGL={self.current_agl:.2f}m | yaw={math.degrees(self.current_yaw):.0f}°")

        # ── PRECISION_LAND_HANDOFF — activate precision landing node ──
        elif self.state == STATE_PRECISION_LAND_HANDOFF:
            # Stop publishing our own setpoints — precision_landing_node takes over
            activate_msg = Bool()
            activate_msg.data = True
            self.pl_activate_pub.publish(activate_msg)
            self.transition_to(STATE_WAIT_FOR_LANDING)

        # ── WAIT_FOR_LANDING — precision_landing_node controls the drone ──
        elif self.state == STATE_WAIT_FOR_LANDING:
            # Do NOT publish any setpoints — precision_landing_node owns them
            if self.precision_landing_status == "LANDED" or \
               (not self.fcu_armed and self.time_in_state() > 2.0):
                self.get_logger().info("Precision landing complete!")
                self.transition_to(STATE_POST_LAND)
            else:
                self.log_telem(
                    f"WAIT_FOR_LANDING | PL status: {self.precision_landing_status} | "
                    f"AGL={self.current_agl:.2f}m | pos=({self.current_x - self.origin_x:.2f}, {self.current_y - self.origin_y:.2f}) | "
                    f"yaw={math.degrees(self.current_yaw):.0f}°")


        # ── POST_LAND — drift logging, pose correction, advance waypoint ──
        elif self.state == STATE_POST_LAND:
            self.publish_idle_setpoint()

            if self.time_in_state() >= POST_LAND_WAIT:
                if self.fcu_armed:
                    self.force_disarm()
                    return

                # Log drift error
                if 0 < self.current_wp_index < len(self.mission_waypoints):
                    wp = self.mission_waypoints[self.current_wp_index]
                    if wp is not None:
                        planned_x = wp.x + self.origin_x
                        planned_y = wp.y + self.origin_y
                        drift_x = planned_x - self.current_x
                        drift_y = planned_y - self.current_y
                        drift_total = math.sqrt(drift_x**2 + drift_y**2)
                        self.get_logger().info(
                            f"DRIFT WP{self.current_wp_index}: "
                            f"dx={drift_x:.3f}m dy={drift_y:.3f}m total={drift_total:.3f}m")

                # Record landed position for exclusion
                self.landed_positions.append((self.current_x, self.current_y))

                # Re-capture takeoff position for next leg
                self.takeoff_x = self.current_x
                self.takeoff_y = self.current_y

                # Advance waypoint or finish
                if self.current_wp_index == 0:
                    # Arrived home
                    self.get_logger().info("Home reached — mission complete!")
                    self.transition_to(STATE_COMPLETE)
                elif self.current_wp_index >= self.total_waypoints:
                    # Last waypoint done
                    if self.return_home:
                        self.current_wp_index = 0
                        self.get_logger().info("All WPs done. Returning home...")
                        self.stream_counter = 0
                        self.transition_to(STATE_STREAM_SETPOINTS)
                    else:
                        self.get_logger().info("All WPs done (no return home). Complete.")
                        self.transition_to(STATE_COMPLETE)
                else:
                    # More waypoints
                    self.current_wp_index += 1
                    self.get_logger().info(f"Next: WP{self.current_wp_index}")
                    self.stream_counter = 0
                    self.transition_to(STATE_STREAM_SETPOINTS)

        # ── BLIND_LAND — fallback if no marker found at max altitude ──
        elif self.state == STATE_BLIND_LAND:
            self.publish_descend_setpoint()
            if self.current_agl <= (TOUCHDOWN_HEIGHT + TOUCHDOWN_MARGIN):
                self.get_logger().info(f"Blind touchdown (AGL={self.current_agl:.2f}m)")
                self.force_disarm()
                self.transition_to(STATE_POST_LAND)
            elif self.time_in_state() > DESCEND_TIMEOUT:
                self.get_logger().warn("Blind descent timeout — force disarm")
                self.force_disarm()
                self.transition_to(STATE_POST_LAND)

        # ── FINAL_LAND — home landing (same as blind) ──
        elif self.state == STATE_FINAL_LAND:
            self.publish_descend_setpoint()
            if self.current_agl <= (TOUCHDOWN_HEIGHT + TOUCHDOWN_MARGIN):
                self.get_logger().info("Final touchdown — force disarm")
                self.force_disarm()
                self.transition_to(STATE_DISARM)
            elif self.time_in_state() > DESCEND_TIMEOUT:
                self.force_disarm()
                self.transition_to(STATE_DISARM)

        # ── LANDED (legacy — not used in new flow, kept for safety) ──
        elif self.state == STATE_LANDED:
            self.publish_idle_setpoint()
            if not self.fcu_armed or self.time_in_state() > POST_LAND_WAIT:
                self.transition_to(STATE_POST_LAND)

        # ── ABORT ──
        elif self.state == STATE_ABORT:
            if self.time_in_state() < 0.1:
                self.takeoff_x = self.current_x
                self.takeoff_y = self.current_y
            self.publish_descend_setpoint()
            if self.current_agl <= (TOUCHDOWN_HEIGHT + TOUCHDOWN_MARGIN):
                self.get_logger().info("ABORT: touchdown → force disarm")
                self.force_disarm()
                self.transition_to(STATE_DISARM)
            elif self.time_in_state() > DESCEND_TIMEOUT:
                self.force_disarm()
                self.transition_to(STATE_DISARM)

        # ── DISARM ──
        elif self.state == STATE_DISARM:
            if not self.fcu_armed:
                self.transition_to(STATE_COMPLETE)
            else:
                self.request_disarm()
                self.log_throttled("Disarming...")

        # ── COMPLETE ──
        elif self.state == STATE_COMPLETE:
            if self.previous_state != STATE_COMPLETE:
                self.get_logger().info("=" * 40)
                self.get_logger().info(" ✓ MISSION COMPLETE")
                self.get_logger().info("=" * 40)
                self.previous_state = STATE_COMPLETE


# ==========================================================
# Main
# ==========================================================

def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutive()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().warn("!!! INTERRUPT — controlled descent + force disarm !!!")

        # Deactivate precision landing
        deactivate = Bool()
        deactivate.data = False
        node.pl_activate_pub.publish(deactivate)

        # Controlled descent
        node.takeoff_x = node.current_x
        node.takeoff_y = node.current_y
        deadline = _time.time() + 45.0

        while _time.time() < deadline:
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except (KeyboardInterrupt, SystemExit):
                node.get_logger().warn("Double interrupt — force disarm")
                node.force_disarm()
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.2)
                break
            node.publish_descend_setpoint()
            if node.current_agl <= (TOUCHDOWN_HEIGHT + TOUCHDOWN_MARGIN):
                node.get_logger().info("Touchdown → force disarm")
                node.force_disarm()
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.2)
                    if not node.fcu_armed:
                        break
                break

        if node.fcu_armed:
            node.force_disarm()
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
