#!/usr/bin/env python3
"""
Camera Publisher — Auto-detects RealSense D435i or USB webcam (Logitech C922)

Publishes:
  /down/image_raw           — color image (sensor_msgs/Image)
  /down/depth/image_raw     — depth image (sensor_msgs/Image, 16UC1) [RealSense only]

Auto-detection order:
  1. Intel RealSense D435i (color + depth)
  2. USB webcam (Logitech C922 or any V4L2 camera, color only)

For USB cameras:
  - Autofocus is disabled (fixed focus at infinity)
  - Auto-exposure is left on (more reliable outdoors)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Camera settings
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30
COLOR_TOPIC = '/down/image_raw'
DEPTH_TOPIC = '/down/depth/image_raw'


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.color_pub = self.create_publisher(Image, COLOR_TOPIC, 10)
        self.depth_pub = self.create_publisher(Image, DEPTH_TOPIC, 10)
        self.bridge = CvBridge()
        self.pipeline = None
        self.cap = None
        self.use_realsense = False
        self.has_depth = False
        self.camera_name = "NONE"

        self.get_logger().info("=" * 50)
        self.get_logger().info(" Camera Publisher — Auto-detecting camera...")
        self.get_logger().info("=" * 50)

        # Try RealSense first
        if self._try_realsense():
            pass
        elif self._try_usb_camera():
            pass
        else:
            self.get_logger().error("No camera found! (checked RealSense + USB)")
            self.get_logger().error("  Plug in RealSense D435i or Logitech C922")
            return

        self.get_logger().info("")
        self.get_logger().info(f"  Camera    : {self.camera_name}")
        self.get_logger().info(f"  Resolution: {FRAME_WIDTH}x{FRAME_HEIGHT} @ {FPS}fps")
        self.get_logger().info(f"  Color     : {COLOR_TOPIC}")
        if self.has_depth:
            self.get_logger().info(f"  Depth     : {DEPTH_TOPIC}")
        self.get_logger().info("=" * 50)

        self.timer = self.create_timer(1.0 / FPS, self.publish_frame)

    def _try_realsense(self):
        """Try to initialize Intel RealSense D435i."""
        try:
            import pyrealsense2 as rs
            self.rs = rs
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                return False

            device_name = devices[0].get_info(rs.camera_info.name)
            self.get_logger().info(f"  Found RealSense: {device_name}")

            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FPS)
            config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FPS)
            self.pipeline.start(config)
            self.use_realsense = True
            self.has_depth = True
            self.camera_name = f"Intel RealSense {device_name}"
            return True
        except ImportError:
            self.get_logger().info("  pyrealsense2 not installed — skipping RealSense")
            return False
        except Exception as e:
            self.get_logger().info(f"  RealSense not available: {e}")
            return False

    def _try_usb_camera(self):
        """Try to initialize USB webcam (Logitech C922 or any V4L2 camera)."""
        # Try device indices 0-3
        for idx in range(4):
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                # Set resolution and FPS
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
                cap.set(cv2.CAP_PROP_FPS, FPS)

                # Disable autofocus (critical for stable detection at altitude)
                cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                cap.set(cv2.CAP_PROP_FOCUS, 0)  # 0 = infinity

                # Verify we can read a frame
                ret, frame = cap.read()
                if not ret:
                    cap.release()
                    continue

                actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

                # Try to get camera name
                backend = cap.getBackendName()
                self.camera_name = f"USB Camera /dev/video{idx} ({backend}) [{actual_w}x{actual_h}]"
                self.get_logger().info(f"  Found USB camera at /dev/video{idx}")
                self.get_logger().info(f"  Autofocus: DISABLED (fixed at infinity)")

                self.cap = cap
                self.use_realsense = False
                self.has_depth = False
                return True
            cap.release()

        return False

    def publish_frame(self):
        stamp = self.get_clock().now().to_msg()

        if self.use_realsense:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()

                if not color_frame:
                    return

                color_img = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
                color_msg.header.stamp = stamp
                color_msg.header.frame_id = 'camera_down'
                self.color_pub.publish(color_msg)

                if depth_frame:
                    depth_img = np.asanyarray(depth_frame.get_data())
                    depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
                    depth_msg.header.stamp = stamp
                    depth_msg.header.frame_id = 'camera_down'
                    self.depth_pub.publish(depth_msg)

            except Exception as e:
                self.get_logger().warn(f'Frame error: {e}')

        elif self.cap is not None:
            ret, frame = self.cap.read()
            if not ret:
                return
            color_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = 'camera_down'
            self.color_pub.publish(color_msg)

    def destroy_node(self):
        if self.use_realsense and self.pipeline:
            self.pipeline.stop()
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
