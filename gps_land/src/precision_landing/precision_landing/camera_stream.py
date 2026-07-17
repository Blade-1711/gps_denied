#!/usr/bin/env python3
"""
Camera Stream Node — Always-On MJPEG Web Stream

Subscribes to:
    /down/image_raw             — raw camera (always available)
    /precision_landing/debug    — annotated feed from precision landing node
    /mission/debug              — annotated feed from mission executive (if published)

Logic:
    - If an annotated frame is available (from precision_landing or mission_executive),
      show that (it has marker overlay, crosshair, state info)
    - Otherwise show the raw camera feed
    - Always streaming, regardless of mission state

Web stream:
    http://<jetson_ip>:5000

This node is independent — doesn't affect any other node's behavior.
"""

import time
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from flask import Flask, Response, render_template_string
import socket


# ==========================================================
# Web Server
# ==========================================================

WEB_PORT = 5000

_web_frame = None
_web_frame_lock = threading.Lock()

_flask_app = Flask(__name__)

_WEB_PAGE = """<!DOCTYPE html><html><head><title>Mission Camera</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{background:#111;color:#eee;font-family:monospace;text-align:center;padding:10px}
img{width:100%;max-width:720px;border:2px solid #0cf;border-radius:8px}
h2{color:#0cf}</style></head><body>
<h2>&#128249; Mission Camera — Live Feed</h2>
<img src="/video_feed"><br>
<p style="color:#888">Annotated when precision landing active | Raw camera otherwise</p>
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
# Stream Node
# ==========================================================

class CameraStream(Node):

    def __init__(self):
        super().__init__('camera_stream')

        self.bridge = CvBridge()

        # Track which source is active
        self._last_annotated_time = 0.0
        self._annotated_timeout = 1.0  # seconds — if no annotated frame for 1s, show raw

        # QoS for camera topics
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=5)

        # Subscribe to raw camera (always available)
        self.create_subscription(
            Image, '/down/image_raw', self._raw_cb, sensor_qos)

        # Subscribe to annotated feeds (available when nodes are active)
        self.create_subscription(
            Image, '/precision_landing/debug', self._annotated_cb, sensor_qos)

        # Start web server
        threading.Thread(target=_start_web, daemon=True).start()

        # Get local IP for display
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            local_ip = s.getsockname()[0]
            s.close()
        except Exception:
            local_ip = '127.0.0.1'

        self.get_logger().info("=" * 50)
        self.get_logger().info(" Camera Stream Node (always-on)")
        self.get_logger().info(f"  Stream URL : http://{local_ip}:{WEB_PORT}")
        self.get_logger().info(f"  Raw topic  : /down/image_raw")
        self.get_logger().info(f"  Debug topic: /precision_landing/debug")
        self.get_logger().info("=" * 50)

    def _raw_cb(self, msg):
        """Raw camera — show only if no recent annotated frame."""
        global _web_frame

        now = time.time()
        if now - self._last_annotated_time < self._annotated_timeout:
            # Annotated feed is active — skip raw
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        # Add a small "LIVE" indicator so user knows stream is working
        h, w = frame.shape[:2]
        cv2.circle(frame, (w - 20, 20), 6, (0, 0, 255), -1)
        cv2.putText(frame, "LIVE", (w - 55, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        with _web_frame_lock:
            _web_frame = frame

    def _annotated_cb(self, msg):
        """Annotated frame from precision_landing — always preferred."""
        global _web_frame

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        self._last_annotated_time = time.time()

        with _web_frame_lock:
            _web_frame = frame


def main(args=None):
    rclpy.init(args=args)
    node = CameraStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
