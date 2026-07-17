#!/usr/bin/env python3
"""
Precision Landing Node — Activation-Based

Waits in IDLE until activated via /precision_landing/activate (Bool).
When activated (while drone is already flying in OFFBOARD):
    1. SEARCHING — scan for marker below
    2. SCAN_CLIMB — climb higher if marker not found
    3. ALIGNING — P-controller centers drone over marker
    4. DESCENDING — smooth tanh descent while maintaining alignment
    5. LANDED — force disarm, return to IDLE

This node does NOT handle takeoff, arming, or OFFBOARD switching.
Those are managed by mission_executive.

Hardware:
    Pixhawk 6C, Jetson Orin Nano, Holybro H-Flow, down-facing camera

Topics:
    Subscribes:
        /down/image_raw                 (Image — down-facing camera)
        /mavros/state                   (State — FCU armed/connected/mode)
        /mavros/rangefinder_pub         (Range — AGL altitude)
        /mavros/local_position/pose     (PoseStamped — local position)
        /precision_landing/activate     (Bool — activation trigger)

    Publishes:
        /mavros/setpoint_velocity/cmd_vel (TwistStamped — velocity commands)
        /precision_landing/status         (String — current state)
        /precision_landing/debug          (Image — annotated camera feed)

Parameters:
    alt             : reference altitude for scan calculations (default 1.5 m)
    hover_time      : not used directly (kept for param compat)
    marker_id       : ArUco marker ID (-1 = any, default -1)
    aruco_dict      : ArUco dictionary (default 4X4_250)
    align_tolerance : normalized tolerance for "centered" (default 0.05)
    max_align_speed : max horizontal correction m/s (default 0.3)
    kp_align        : P-gain for alignment (default 0.15)
    detection_mode  : "aruco", "color", or "model" (default "aruco")
    model_path      : path to YOLOv8 model file (.pt or .engine) for "model" mode
    color_h_low/high, color_s_low/high, color_v_low/high : HSV range for color
"""

import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, Range
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import String, Bool
from mavros_msgs.srv import CommandLong
from cv_bridge import CvBridge

import threading
import socket
from flask import Flask, Response, render_template_string

# ==========================================================
# Web UI (MJPEG stream — access from laptop over SSH/WiFi)
# ==========================================================
WEB_PORT = 5000
_web_frame = None
_web_frame_lock = threading.Lock()

_flask_app = Flask(__name__)

_WEB_PAGE = """<!DOCTYPE html><html><head><title>Precision Landing</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{background:#111;color:#eee;font-family:monospace;text-align:center;padding:10px}
img{width:100%;max-width:720px;border:2px solid #0cf;border-radius:8px}
h2{color:#0cf}</style></head><body>
<h2>&#128681; Precision Landing — Live Feed</h2>
<img src="/video_feed"><br>
<p style="color:#888">Green = marker detected | Red = not found | White crosshair = center target</p>
</body></html>"""

@_flask_app.route('/')
def _index():
    return render_template_string(_WEB_PAGE)

@_flask_app.route('/video_feed')
def _video_feed():
    return Response(_gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def _gen_frames():
    while True:
        with _web_frame_lock:
            f = _web_frame
        if f is None:
            placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, 'Waiting for camera...', (150, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
            _, jpg = cv2.imencode('.jpg', placeholder)
        else:
            _, jpg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 75])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')
        time.sleep(0.033)

def _start_web():
    _flask_app.run(host='0.0.0.0', port=WEB_PORT, threaded=True, use_reloader=False)


# ==========================================================
# Tunables
# ==========================================================

CONTROL_RATE = 20.0             # Hz

# Horizontal drift correction during hold
KP_HORIZONTAL = 0.6
MAX_HORIZONTAL_VEL = 0.5

# Descent profile (tanh — smooth)
DESCENT_TANH_K = 1.2
MAX_DESCENT_VEL = 0.40
MIN_DESCENT_VEL = 0.08
TOUCHDOWN_HEIGHT = 0.21         # H-Flow ground reading
TOUCHDOWN_MARGIN = 0.03

# Camera FOV — UPDATE AFTER CALIBRATION
# Current: estimated for Logitech C922 at 640x480 (non-wide mode, autofocus off)
# Calibrate: place camera 1m from wall, measure visible width W and height H
#   FOV_H = 2 * atan(W / 2)  in degrees
#   FOV_V = 2 * atan(H / 2)  in degrees
CAMERA_FOV_H_DEG = 70.0
CAMERA_FOV_V_DEG = 43.0
CAMERA_HFOV_TAN = math.tan(math.radians(CAMERA_FOV_H_DEG / 2.0))
CAMERA_VFOV_TAN = math.tan(math.radians(CAMERA_FOV_V_DEG / 2.0))

# Fallback convergence (when marker lost, converge to estimated pose)
KP_CONVERGE = 0.3
MAX_CONVERGE_VEL = 0.3

# Alignment profile (tanh — fast approach, smooth deceleration)
MAX_ALIGN_SPEED = 0.3
K_ALIGN = 2.0

# ArUco detection
ALIGNED_FRAMES_REQUIRED = 5
LOST_TIMEOUT = 3.0
LOST_ABORT_TIMEOUT = 15.0

# Altitude scanning — if marker not found, climb higher to see more ground
SCAN_CLIMB_STEP = 0.5
SCAN_MAX_ALTITUDE = 5.0
SCAN_WAIT_AFTER_CLIMB = 2.0

# States
ST_IDLE = "IDLE"
ST_SEARCHING = "SEARCHING"
ST_SCAN_CLIMB = "SCAN_CLIMB"
ST_ALIGNING = "ALIGNING"
ST_DESCENDING = "DESCENDING"
ST_LANDED = "LANDED"


class PrecisionLanding(Node):

    def __init__(self):
        super().__init__('precision_landing')

        # Parameters
        self.declare_parameter('alt', 1.5)
        self.declare_parameter('hover_time', 3.0)
        self.declare_parameter('marker_id', -1)
        self.declare_parameter('align_tolerance', 0.05)
        self.declare_parameter('max_align_speed', 0.15)
        self.declare_parameter('kp_align', 0.15)
        self.declare_parameter('detection_mode', 'aruco')   # "aruco", "color", or "model"
        self.declare_parameter('model_path', '')             # path to YOLOv8 .pt or .engine
        self.declare_parameter('camera_topic', '/down/image_raw')
        self.declare_parameter('color_h_low', 0)
        self.declare_parameter('color_h_high', 10)
        self.declare_parameter('color_s_low', 100)
        self.declare_parameter('color_s_high', 255)
        self.declare_parameter('color_v_low', 100)
        self.declare_parameter('color_v_high', 255)

        self.target_alt = self.get_parameter('alt').value
        self.hover_time = self.get_parameter('hover_time').value
        self.marker_id = self.get_parameter('marker_id').value
        self.align_tolerance = self.get_parameter('align_tolerance').value
        self.max_align_speed = self.get_parameter('max_align_speed').value
        self.kp_align = self.get_parameter('kp_align').value
        self.detection_mode = self.get_parameter('detection_mode').value
        self.model_path = self.get_parameter('model_path').value
        self.camera_topic = self.get_parameter('camera_topic').value

        self.hsv_low = np.array([
            self.get_parameter('color_h_low').value,
            self.get_parameter('color_s_low').value,
            self.get_parameter('color_v_low').value])
        self.hsv_high = np.array([
            self.get_parameter('color_h_high').value,
            self.get_parameter('color_s_high').value,
            self.get_parameter('color_v_high').value])

        # Load YOLO model if detection_mode is "model"
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

        # State
        self.state = ST_IDLE
        self.state_entry_time = time.time()
        self.bridge = CvBridge()

        self.fcu_connected = False
        self.fcu_armed = False
        self.fcu_mode = ""

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z_enu = 0.0
        self.current_yaw = 0.0
        self.current_agl = 0.0
        self.rangefinder_ready = False
        self.position_ready = False

        self.takeoff_x = 0.0
        self.takeoff_y = 0.0
        self.hold_z_enu = 0.0

        # Detection state
        self.aligned_count = 0
        self.last_detection_time = 0.0
        self.marker_cx = 0.0
        self.marker_cy = 0.0
        self.marker_found = False
        self.align_confidence = 0.0
        self.scan_target_alt = 0.0
        self.frame_w = 640
        self.frame_h = 480
        self.last_frame = None
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_vz = 0.0

        # Marker pose estimation (ENU world frame)
        self.marker_enu_x = 0.0
        self.marker_enu_y = 0.0
        self.marker_pose_valid = False

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=5)

        # Subscribers
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(Range, '/mavros/rangefinder_pub', self.range_cb, sensor_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, sensor_qos)
        self.create_subscription(Image, self.camera_topic, self.image_cb, sensor_qos)
        self.create_subscription(Bool, '/precision_landing/activate', self.activate_cb, 10)

        # Publishers
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.setpoint_raw_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.status_pub = self.create_publisher(String, '/precision_landing/status', 10)
        self.debug_pub = self.create_publisher(Image, '/precision_landing/debug', 5)

        # Service clients
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')

        # Timer
        self.create_timer(1.0 / CONTROL_RATE, self.control_loop)

        # Web UI (MJPEG stream on port 5000)
        threading.Thread(target=_start_web, daemon=True).start()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            local_ip = s.getsockname()[0]
            s.close()
        except Exception:
            local_ip = '127.0.0.1'
        self.get_logger().info(f"  Web UI: http://{local_ip}:{WEB_PORT}")

        # File logging
        import os
        from datetime import datetime
        log_dir = os.path.expanduser('~/gps_land/logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file_path = os.path.join(log_dir, f'precision_landing_{timestamp}.log')
        self.log_file = open(self.log_file_path, 'w')
        self.log_file.write(f"# Precision Landing Log — {timestamp}\n")
        self.log_file.write(f"# Alt={self.target_alt}m, MarkerID={self.marker_id}, Tol={self.align_tolerance}\n")
        self.log_file.write(f"# time_s, state, agl_m, marker_found, offset_x, offset_y, vx, vy, vz\n")
        self.log_file.flush()
        self._log_start_time = time.time()

        self.get_logger().info("=" * 50)
        self.get_logger().info(" Precision Landing Node (activation-based)")
        self.get_logger().info(f"  Altitude ref  : {self.target_alt} m")
        self.get_logger().info(f"  Marker ID     : {self.marker_id} (-1=any)")
        self.get_logger().info(f"  Detection     : {self.detection_mode}")
        if self.detection_mode == 'model':
            self.get_logger().info(f"  Model path    : {self.model_path}")
        self.get_logger().info(f"  Align tol     : {self.align_tolerance}")
        self.get_logger().info(f"  Camera        : {self.camera_topic}")
        self.get_logger().info(f"  Log file      : {self.log_file_path}")
        self.get_logger().info(f"  State         : {self.state} (waiting for activation)")
        self.get_logger().info("=" * 50)

    # ==========================================================
    # Callbacks
    # ==========================================================

    def activate_cb(self, msg):
        if msg.data and self.state == ST_IDLE:
            self.get_logger().info('Activation received — starting precision landing')
            self.takeoff_x = self.current_x
            self.takeoff_y = self.current_y
            self.hold_z_enu = self.current_z_enu
            self.transition(ST_SEARCHING)
        elif not msg.data and self.state not in (ST_IDLE, ST_LANDED):
            self.get_logger().info('Deactivation received — aborting')
            self.transition(ST_IDLE)

    def state_cb(self, msg):
        self.fcu_connected = msg.connected
        self.fcu_armed = msg.armed
        self.fcu_mode = msg.mode

    def range_cb(self, msg):
        if not (math.isinf(msg.range) or math.isnan(msg.range) or msg.range <= 0.0 or msg.range > 8.0):
            self.current_agl = msg.range
            self.rangefinder_ready = True

    def pose_cb(self, msg):
        z = msg.pose.position.z
        if not math.isnan(z):
            self.current_x = msg.pose.position.x
            self.current_y = msg.pose.position.y
            self.current_z_enu = z

            # Extract yaw from quaternion (ENU frame)
            q = msg.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

            self.position_ready = True

    def image_cb(self, msg):
        """Detect ArUco marker in camera frame."""
        if self.state not in (ST_SEARCHING, ST_SCAN_CLIMB, ST_ALIGNING, ST_DESCENDING):
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        self.frame_h, self.frame_w = frame.shape[:2]
        self.last_frame = frame

        # Detect
        detection = None
        if self.detection_mode == 'aruco':
            detection = self._detect_aruco(frame)
        elif self.detection_mode == 'color':
            detection = self._detect_color(frame)
        elif self.detection_mode == 'model':
            detection = self._detect_model(frame)

        if detection is not None:
            self.marker_cx, self.marker_cy, _ = detection
            self.marker_found = True
            self.last_detection_time = time.time()
        else:
            self.marker_found = False

        # Publish debug image
        self._publish_debug(frame)

    # ==========================================================
    # Detection
    # ==========================================================

    def _detect_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            return None
        for i, mid in enumerate(ids.flatten()):
            if self.marker_id == -1 or mid == self.marker_id:
                c = corners[i][0]
                cx = np.mean(c[:, 0])
                cy = np.mean(c[:, 1])
                area = cv2.contourArea(c.astype(np.int32))
                return (cx, cy, area)
        return None

    def _detect_color(self, frame):
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
                # Take the highest confidence detection
                boxes = results[0].boxes
                best_idx = boxes.conf.argmax()
                x1, y1, x2, y2 = boxes.xyxy[best_idx].cpu().numpy()
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                area = (x2 - x1) * (y2 - y1)
                return (float(cx), float(cy), float(area))
        except Exception as e:
            self.get_logger().warn(f"YOLO inference error: {e}", throttle_duration_sec=5.0)
        return None

    def _publish_debug(self, frame):
        global _web_frame
        h, w = frame.shape[:2]
        color = (0, 255, 0) if self.marker_found else (0, 0, 255)

        # Center crosshair
        cv2.line(frame, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 255, 255), 1)
        cv2.line(frame, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 255, 255), 1)

        if self.marker_found:
            cv2.circle(frame, (int(self.marker_cx), int(self.marker_cy)), 8, color, 2)
            cv2.line(frame, (w//2, h//2), (int(self.marker_cx), int(self.marker_cy)), color, 2)

        cv2.putText(frame, f"{self.state} AGL:{self.current_agl:.2f}m",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        if self.state in (ST_ALIGNING, ST_DESCENDING):
            conf_color = (0, 255, 0) if self.align_confidence > 90 else (0, 255, 255) if self.align_confidence > 50 else (0, 0, 255)
            cv2.putText(frame, f"Confidence: {self.align_confidence:.0f}%",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, conf_color, 2)

        # Feed web UI
        with _web_frame_lock:
            _web_frame = frame.copy()

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        except Exception:
            pass

    # ==========================================================
    # Terminal Status
    # ==========================================================

    _last_terminal_print = 0.0

    def _print_terminal_display(self):
        """Print one-line status with velocity commands every 1s."""
        now = time.time()
        if now - self._last_terminal_print < 1.0:
            return
        self._last_terminal_print = now

        if self.state == ST_IDLE:
            return

        if self.marker_found:
            err_x = (self.marker_cx - self.frame_w / 2.0) / (self.frame_w / 2.0)
            err_y = (self.marker_cy - self.frame_h / 2.0) / (self.frame_h / 2.0)
            marker_info = f"● X={err_x:+.2f} Y={err_y:+.2f} conf={self.align_confidence:.0f}%"
        else:
            marker_info = "✗ no marker"

        self.get_logger().info(
            f"[{self.state:<12}] AGL={self.current_agl:.2f}m | "
            f"Vel: vx={self._last_vx:+.2f} vy={self._last_vy:+.2f} vz={self._last_vz:+.2f} | "
            f"Marker: {marker_info}")

    # ==========================================================
    # Helpers
    # ==========================================================

    def transition(self, new_state):
        self.get_logger().info(f"State: {self.state} → {new_state}")
        self._log_to_file(f"# TRANSITION: {self.state} → {new_state}")
        self.state = new_state
        self.state_entry_time = time.time()

    def _log_to_file(self, line=None):
        """Log current state data to CSV file."""
        if not hasattr(self, 'log_file') or self.log_file.closed:
            return
        elapsed = time.time() - self._log_start_time
        if line:
            self.log_file.write(f"{line}\n")
        else:
            marker_found_int = 1 if self.marker_found else 0
            err_x = (self.marker_cx - self.frame_w / 2.0) / (self.frame_w / 2.0) if self.marker_found else 0.0
            err_y = (self.marker_cy - self.frame_h / 2.0) / (self.frame_h / 2.0) if self.marker_found else 0.0
            self.log_file.write(
                f"{elapsed:.2f}, {self.state}, {self.current_agl:.3f}, "
                f"{marker_found_int}, {err_x:.3f}, {err_y:.3f}, "
                f"{self._last_vx:.3f}, {self._last_vy:.3f}, {self._last_vz:.3f}\n")
        self.log_file.flush()

    def time_in_state(self):
        return time.time() - self.state_entry_time

    def publish_velocity(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        self.vel_pub.publish(msg)

        self._last_vx = vx
        self._last_vy = vy
        self._last_vz = vz

        # Log data every 0.5s
        if not hasattr(self, '_last_file_log'):
            self._last_file_log = 0.0
        now = time.time()
        if now - self._last_file_log > 0.5 and self.state != ST_IDLE:
            self._last_file_log = now
            self._log_to_file()

    def horizontal_hold(self):
        """P-controller countering horizontal drift."""
        err_x = self.takeoff_x - self.current_x
        err_y = self.takeoff_y - self.current_y
        vx = max(-MAX_HORIZONTAL_VEL, min(MAX_HORIZONTAL_VEL, KP_HORIZONTAL * err_x))
        vy = max(-MAX_HORIZONTAL_VEL, min(MAX_HORIZONTAL_VEL, KP_HORIZONTAL * err_y))
        return vx, vy

    def publish_hold_setpoint(self):
        """Position hold via PositionTarget at captured XY + EKF z + yaw."""
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        msg.position.x = self.takeoff_x
        msg.position.y = self.takeoff_y
        msg.position.z = self.hold_z_enu
        msg.yaw = float(self.current_yaw)
        self.setpoint_raw_pub.publish(msg)

    def descent_speed(self):
        """Tanh descent profile."""
        hag = max(self.current_agl - TOUCHDOWN_HEIGHT, 0.0)
        span = MAX_DESCENT_VEL - MIN_DESCENT_VEL
        v = MIN_DESCENT_VEL + span * math.tanh(DESCENT_TANH_K * hag)
        return max(MIN_DESCENT_VEL, min(v, MAX_DESCENT_VEL))

    def force_disarm(self):
        if self.cmd_client.service_is_ready():
            req = CommandLong.Request()
            req.command = 400
            req.param1 = 0.0
            req.param2 = 21196.0
            self.cmd_client.call_async(req)

    # ==========================================================
    # State Machine
    # ==========================================================

    def control_loop(self):
        # Publish status always (so mission_executive can monitor)
        status_msg = String()
        if self.state in (ST_ALIGNING, ST_DESCENDING) and self.marker_found:
            status_msg.data = f"{self.state} | confidence: {self.align_confidence:.0f}%"
        else:
            status_msg.data = self.state
        self.status_pub.publish(status_msg)

        # Terminal visual feed
        self._print_terminal_display()

        # ── IDLE — do nothing, mission_executive owns setpoints ──
        if self.state == ST_IDLE:
            return

        # ── SEARCHING (hover, scanning for marker — climb higher if not found) ──
        elif self.state == ST_SEARCHING:
            self.publish_hold_setpoint()
            if self.marker_found:
                self.get_logger().info(
                    f"✓ Marker detected at AGL={self.current_agl:.2f}m! Aligning...")
                self.aligned_count = 0
                self.transition(ST_ALIGNING)
            elif self.time_in_state() > SCAN_WAIT_AFTER_CLIMB + 1.0:
                # Not found at this altitude — climb higher
                if self.current_agl < SCAN_MAX_ALTITUDE - SCAN_CLIMB_STEP:
                    self.scan_target_alt = self.current_agl + SCAN_CLIMB_STEP
                    self.get_logger().info(
                        f"Marker not found at {self.current_agl:.1f}m "
                        f"→ climbing to {self.scan_target_alt:.1f}m")
                    self.transition(ST_SCAN_CLIMB)
                else:
                    # Already at max altitude — blind land
                    self.get_logger().warn(
                        f"Marker not found up to {SCAN_MAX_ALTITUDE}m — blind landing")
                    self.transition(ST_DESCENDING)

        # ── SCAN_CLIMB (climbing to next scan altitude) ──
        elif self.state == ST_SCAN_CLIMB:
            msg = PositionTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = (
                PositionTarget.IGNORE_VX |
                PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )
            msg.position.x = self.takeoff_x
            msg.position.y = self.takeoff_y
            msg.position.z = self.hold_z_enu + (self.scan_target_alt - self.target_alt)
            msg.yaw = float(self.current_yaw)
            self.setpoint_raw_pub.publish(msg)

            if self.current_agl >= self.scan_target_alt * 0.95:
                self.get_logger().info(
                    f"Reached {self.current_agl:.2f}m — scanning...")
                self.transition(ST_SEARCHING)
            if self.marker_found:
                self.get_logger().info(
                    f"✓ Marker spotted during climb at AGL={self.current_agl:.2f}m!")
                self.aligned_count = 0
                self.transition(ST_ALIGNING)

        # ── ALIGNING (centering over marker) ──
        elif self.state == ST_ALIGNING:
            if not self.marker_found:
                self.align_confidence = 0.0
                if time.time() - self.last_detection_time > LOST_TIMEOUT:
                    self.get_logger().warn("Marker lost — back to searching")
                    self.transition(ST_SEARCHING)
                elif self.marker_pose_valid:
                    err_e = self.marker_enu_x - self.current_x
                    err_n = self.marker_enu_y - self.current_y
                    vx = max(-MAX_CONVERGE_VEL, min(MAX_CONVERGE_VEL, KP_CONVERGE * err_e))
                    vy = max(-MAX_CONVERGE_VEL, min(MAX_CONVERGE_VEL, KP_CONVERGE * err_n))
                    self.publish_velocity(vx, vy, 0.0)
                else:
                    vx, vy = self.horizontal_hold()
                    self.publish_velocity(vx, vy, 0.0)
                return

            # Compute normalized camera error
            frame_w, frame_h = self.frame_w, self.frame_h
            cam_err_x = (self.marker_cx - frame_w / 2.0) / (frame_w / 2.0)
            cam_err_y = (self.marker_cy - frame_h / 2.0) / (frame_h / 2.0)

            # Confidence score
            distance = math.sqrt(cam_err_x * cam_err_x + cam_err_y * cam_err_y)
            self.align_confidence = max(0.0, (1.0 - distance / 1.414)) * 100.0

            # Camera → Body frame (FRD: +X forward, +Y right)
            # camera_y (down in image) → body +X (forward)
            # camera_x (right in image) → body -Y (left)
            body_x = cam_err_y
            body_y = -cam_err_x
            yaw = self.current_yaw
            vx_enu = body_x * math.cos(yaw) + body_y * math.sin(yaw)
            vy_enu = body_x * math.sin(yaw) - body_y * math.cos(yaw)

            # Tanh alignment: fast when far, smooth deceleration near center
            enu_mag = math.sqrt(vx_enu * vx_enu + vy_enu * vy_enu)
            if enu_mag > 1e-6:
                align_speed = MAX_ALIGN_SPEED * math.tanh(K_ALIGN * enu_mag)
                vx_enu = (vx_enu / enu_mag) * align_speed
                vy_enu = (vy_enu / enu_mag) * align_speed
            else:
                vx_enu, vy_enu = 0.0, 0.0

            # Update marker ENU pose estimate
            offset_body_x = cam_err_y * self.current_agl * CAMERA_VFOV_TAN
            offset_body_y = -cam_err_x * self.current_agl * CAMERA_HFOV_TAN
            offset_east = offset_body_x * math.cos(yaw) + offset_body_y * math.sin(yaw)
            offset_north = offset_body_x * math.sin(yaw) - offset_body_y * math.cos(yaw)
            self.marker_enu_x = self.current_x + offset_east
            self.marker_enu_y = self.current_y + offset_north
            self.marker_pose_valid = True

            if self.align_confidence >= 85.0:
                self.aligned_count += 1
                if self.aligned_count >= ALIGNED_FRAMES_REQUIRED:
                    self.get_logger().info(
                        f"ALIGNED (confidence={self.align_confidence:.0f}%)! Starting descent...")
                    self.transition(ST_DESCENDING)
                self.publish_velocity(vx_enu, vy_enu, 0.0)
            else:
                self.aligned_count = 0
                self.publish_velocity(vx_enu, vy_enu, 0.0)

        # ── DESCENDING (aligned descent on marker) ──
        elif self.state == ST_DESCENDING:
            if self.marker_found:
                frame_w, frame_h = self.frame_w, self.frame_h
                cam_err_x = (self.marker_cx - frame_w / 2.0) / (frame_w / 2.0)
                cam_err_y = (self.marker_cy - frame_h / 2.0) / (frame_h / 2.0)
                distance = math.sqrt(cam_err_x * cam_err_x + cam_err_y * cam_err_y)
                self.align_confidence = max(0.0, (1.0 - distance / 1.414)) * 100.0

                # Camera → Body (FRD) → ENU
                body_x = cam_err_y
                body_y = -cam_err_x
                yaw = self.current_yaw
                vx_enu = body_x * math.cos(yaw) + body_y * math.sin(yaw)
                vy_enu = body_x * math.sin(yaw) - body_y * math.cos(yaw)
                # Tanh alignment during descent
                enu_mag = math.sqrt(vx_enu * vx_enu + vy_enu * vy_enu)
                if enu_mag > 1e-6:
                    align_speed = MAX_ALIGN_SPEED * math.tanh(K_ALIGN * enu_mag)
                    vx = (vx_enu / enu_mag) * align_speed
                    vy = (vy_enu / enu_mag) * align_speed
                else:
                    vx, vy = 0.0, 0.0

                # Update marker ENU pose estimate
                offset_body_x = cam_err_y * self.current_agl * CAMERA_VFOV_TAN
                offset_body_y = -cam_err_x * self.current_agl * CAMERA_HFOV_TAN
                offset_east = offset_body_x * math.cos(yaw) + offset_body_y * math.sin(yaw)
                offset_north = offset_body_x * math.sin(yaw) - offset_body_y * math.cos(yaw)
                self.marker_enu_x = self.current_x + offset_east
                self.marker_enu_y = self.current_y + offset_north
                self.marker_pose_valid = True
            elif self.marker_pose_valid:
                self.align_confidence = 0.0
                err_e = self.marker_enu_x - self.current_x
                err_n = self.marker_enu_y - self.current_y
                vx = max(-MAX_CONVERGE_VEL, min(MAX_CONVERGE_VEL, KP_CONVERGE * err_e))
                vy = max(-MAX_CONVERGE_VEL, min(MAX_CONVERGE_VEL, KP_CONVERGE * err_n))
            else:
                self.align_confidence = 0.0
                vx, vy = self.horizontal_hold()

            vz = -self.descent_speed()
            self.publish_velocity(vx, vy, vz)

            # Touchdown
            if self.current_agl <= (TOUCHDOWN_HEIGHT + TOUCHDOWN_MARGIN):
                self.get_logger().info(f"TOUCHDOWN! AGL={self.current_agl:.2f}m")
                self.force_disarm()
                self.transition(ST_LANDED)

        # ── LANDED ──
        elif self.state == ST_LANDED:
            self.publish_velocity(0.0, 0.0, 0.0)
            if not self.fcu_armed or self.time_in_state() > 3.0:
                self.get_logger().info('=' * 40)
                self.get_logger().info(' ✓ PRECISION LANDING COMPLETE')
                self.get_logger().info('=' * 40)
                if hasattr(self, 'log_file') and not self.log_file.closed:
                    self._log_to_file("# MISSION COMPLETE")
                    self.log_file.close()
                    self.get_logger().info(f"Log saved: {self.log_file_path}")
                self.transition(ST_IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLanding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn('INTERRUPT — force disarm!')
        node.force_disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
