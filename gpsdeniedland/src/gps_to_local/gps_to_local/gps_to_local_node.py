#!/usr/bin/env python3

"""
Node:
    GPS to Local Converter

Purpose:
    Collects GPS waypoints (lat, lon) interactively at startup, converts
    them to local XY (home = origin), collects marker detection config,
    and WRITES everything to a JSON file that the mission_executive reads.

    Writing to a file (instead of publishing on a topic) means this
    interactive step is fully decoupled from the launch system — no
    timing race, and the launch file doesn't need to capture stdin.

Input:
    Terminal input (interactive at startup)

Output:
    A JSON waypoints file (default: /tmp/gps_mission.json), e.g.:
    {
      "num_waypoints": 3,
      "default_altitude": 3.0,
      "detection_mode": "model",
      "model_path": "/home/TerraWings/models/marker.pt",
      "color_hsv_low": [0, 100, 100],
      "color_hsv_high": [10, 255, 255],
      "waypoints": [
        {"id": 0, "x": 0.0, "y": 0.0, "z": 3.0},
        {"id": 1, "x": 5.5, "y": 3.3, "z": 3.0},
        ...
      ],
      "segment_altitudes": [3.0, 3.0, 3.0, 3.0]
    }

Conversion (flat-earth, accurate < 10 km):
    X = (lon - lon_home) * cos(lat_home) * 111320.0   (East)
    Y = (lat - lat_home) * 110540.0                    (North)

Usage:
    ros2 run gps_to_local gps_to_local_node
    ros2 run gps_to_local gps_to_local_node --ros-args -p output_file:=/tmp/gps_mission.json
"""

import json
import math
import sys

import rclpy
from rclpy.node import Node


# ==========================================================
# Tunable Constants
# ==========================================================

METERS_PER_DEG_LAT = 110540.0           # meters per degree latitude
METERS_PER_DEG_LON = 111320.0           # meters per degree longitude (equator)

DEFAULT_OUTPUT_FILE = "/tmp/gps_mission.json"


class GpsToLocal(Node):

    def __init__(self):

        super().__init__("gps_to_local")

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------

        self.declare_parameter("output_file", DEFAULT_OUTPUT_FILE)
        self.output_file = str(self.get_parameter("output_file").value)

        # --------------------------------------------------
        # Mission Data
        # --------------------------------------------------

        self.home_lat = 0.0
        self.home_lon = 0.0
        self.waypoints = []             # list of dicts {id, x, y, z}
        self.segment_altitudes = []     # per-leg altitude (incl. return home)
        self.num_waypoints = 0
        self.default_altitude = 3.0

        # Detection config
        self.detection_mode = "aruco"
        self.model_path = ""
        self.color_hsv_low = [0, 100, 100]
        self.color_hsv_high = [10, 255, 255]
        self.color2_hsv_low = [0, 0, 0]
        self.color2_hsv_high = [180, 255, 75]

        # --------------------------------------------------
        # Collect input + write file
        # --------------------------------------------------

        self.get_mission_input()
        self.get_marker_input()
        self.write_waypoints_file()

    # ==========================================================
    # Terminal Input — Waypoints
    # ==========================================================

    def get_mission_input(self):

        print("")
        print("============================================")
        print("   GPS to Local — Mission Waypoint Input")
        print("============================================")
        print("")

        # Number of waypoints
        while True:
            try:
                self.num_waypoints = int(
                    input("  Number of waypoints (excluding home): ")
                )
                if self.num_waypoints < 1:
                    print("  Must be at least 1.")
                    continue
                break
            except ValueError:
                print("  Enter an integer.")

        # Default altitude
        while True:
            try:
                self.default_altitude = float(
                    input("  Default mission altitude (m): ")
                )
                if self.default_altitude <= 0.0:
                    print("  Must be positive.")
                    continue
                break
            except ValueError:
                print("  Enter a number.")

        print("")

        # Home GPS
        while True:
            try:
                home_input = input("  Home (lat, lon): ")
                parts = home_input.replace(",", " ").split()
                self.home_lat = float(parts[0])
                self.home_lon = float(parts[1])
                break
            except (ValueError, IndexError):
                print("  Format: lat, lon  (e.g., 19.0760, 72.8777)")

        # Home waypoint (id 0, always local origin)
        self.waypoints.append({
            "id": 0, "x": 0.0, "y": 0.0, "z": self.default_altitude
        })

        print("")

        # Each waypoint + per-segment altitude
        for i in range(1, self.num_waypoints + 1):

            while True:
                try:
                    wp_input = input(f"  WP{i} (lat, lon): ")
                    parts = wp_input.replace(",", " ").split()
                    lat = float(parts[0])
                    lon = float(parts[1])
                    break
                except (ValueError, IndexError):
                    print("  Format: lat, lon")

            label = f"Home->WP{i}" if i == 1 else f"WP{i-1}->WP{i}"
            alt_str = input(
                f"    Altitude {label} [{self.default_altitude}]: "
            ).strip()

            seg_alt = self.default_altitude
            if alt_str != "":
                try:
                    v = float(alt_str)
                    seg_alt = v if v > 0.0 else self.default_altitude
                except ValueError:
                    pass

            local_x, local_y = self.gps_to_local(lat, lon)

            self.waypoints.append({
                "id": i, "x": local_x, "y": local_y, "z": seg_alt
            })
            self.segment_altitudes.append(seg_alt)
            print("")

        # Return-home altitude
        alt_str = input(
            f"    Altitude WP{self.num_waypoints}->Home "
            f"[{self.default_altitude}]: "
        ).strip()

        return_alt = self.default_altitude
        if alt_str != "":
            try:
                v = float(alt_str)
                return_alt = v if v > 0.0 else self.default_altitude
            except ValueError:
                pass

        self.segment_altitudes.append(return_alt)
        self.waypoints[0]["z"] = return_alt   # home altitude = return altitude

        print("")
        print("  ✓ Waypoints entered")

    # ==========================================================
    # Terminal Input — Marker Detection Config
    # ==========================================================

    def get_marker_input(self):

        print("")
        print("============================================")
        print("   Marker Detection Configuration")
        print("============================================")
        print("")
        print("  Detection modes:")
        print("    1. aruco  — ArUco marker (DICT_4X4_250)")
        print("    2. color  — Color blob (HSV range)")
        print("    3. model  — YOLOv8 trained model (.pt or .engine)")
        print("")

        while True:
            mode_input = input("  Detection mode [aruco]: ").strip().lower()
            if mode_input == "" or mode_input == "1" or mode_input == "aruco":
                self.detection_mode = "aruco"
                break
            elif mode_input == "2" or mode_input == "color":
                self.detection_mode = "color"
                break
            elif mode_input == "3" or mode_input == "model":
                self.detection_mode = "model"
                break
            else:
                print("  Enter: aruco, color, or model (or 1/2/3)")

        # Color-specific config
        if self.detection_mode == "color":
            print("")
            print("  Dual-color detection: board color + print color")
            print("  (Finds the board, confirms print inside, checks symmetry)")
            print("")
            print("  Preset colors:")
            print("    white  — H:0-180 S:0-50  V:170-255")
            print("    black  — H:0-180 S:0-255 V:0-75")
            print("    red    — H:0-12  S:70-255 V:50-255")
            print("    blue   — H:95-135 S:70-255 V:40-255")
            print("    yellow — H:18-38 S:70-255 V:60-255")
            print("")

            presets = {
                "white":  ([0, 0, 170], [180, 50, 255]),
                "black":  ([0, 0, 0], [180, 255, 75]),
                "red":    ([0, 70, 50], [12, 255, 255]),
                "blue":   ([95, 70, 40], [135, 255, 255]),
                "yellow": ([18, 70, 60], [38, 255, 255]),
                "green":  ([35, 70, 40], [85, 255, 255]),
                "orange": ([8, 70, 70], [18, 255, 255]),
                "grey":   ([0, 0, 60], [180, 45, 180]),
            }

            # Board color
            while True:
                board_input = input("  Board color [white]: ").strip().lower()
                if board_input == "":
                    board_input = "white"
                if board_input in presets:
                    low, high = presets[board_input]
                    self.color_hsv_low = low
                    self.color_hsv_high = high
                    break
                print(f"  Options: {', '.join(presets.keys())}")

            # Print color
            while True:
                print_input = input("  Print color [black]: ").strip().lower()
                if print_input == "":
                    print_input = "black"
                if print_input in presets:
                    low, high = presets[print_input]
                    self.color2_hsv_low = low
                    self.color2_hsv_high = high
                    break
                print(f"  Options: {', '.join(presets.keys())}")

            print(f"    Board: {board_input} → HSV {self.color_hsv_low} to {self.color_hsv_high}")
            print(f"    Print: {print_input} → HSV {self.color2_hsv_low} to {self.color2_hsv_high}")

        # Model-specific config
        elif self.detection_mode == "model":
            print("")
            while True:
                model_input = input("  Model file path (.pt or .engine): ").strip()
                if model_input:
                    self.model_path = model_input
                    break
                else:
                    print("  Path cannot be empty.")

        print("")
        print(f"  ✓ Detection: {self.detection_mode}")
        if self.detection_mode == "model":
            print(f"    Model: {self.model_path}")
        elif self.detection_mode == "color":
            print(f"    HSV low:  {self.color_hsv_low}")
            print(f"    HSV high: {self.color_hsv_high}")
        print("============================================")

    # ==========================================================
    # GPS -> Local Conversion
    # ==========================================================

    def gps_to_local(self, lat, lon):
        dy = (lat - self.home_lat) * METERS_PER_DEG_LAT
        dx = (lon - self.home_lon) * METERS_PER_DEG_LON * math.cos(
            math.radians(self.home_lat)
        )
        return dx, dy

    # ==========================================================
    # Write JSON file
    # ==========================================================

    def write_waypoints_file(self):

        data = {
            "num_waypoints": self.num_waypoints,
            "default_altitude": self.default_altitude,
            "home_lat": self.home_lat,
            "home_lon": self.home_lon,
            "detection_mode": self.detection_mode,
            "model_path": self.model_path,
            "color_hsv_low": self.color_hsv_low,
            "color_hsv_high": self.color_hsv_high,
            "color2_hsv_low": self.color2_hsv_low,
            "color2_hsv_high": self.color2_hsv_high,
            "waypoints": self.waypoints,
            "segment_altitudes": self.segment_altitudes,
        }

        try:
            with open(self.output_file, "w") as f:
                json.dump(data, f, indent=2)
        except OSError as e:
            self.get_logger().error(f"Failed to write {self.output_file}: {e}")
            return

        self.get_logger().info("========================================")
        self.get_logger().info(" GPS to Local — Mission Config Saved")
        self.get_logger().info(f" File       : {self.output_file}")
        self.get_logger().info(f" Home GPS   : ({self.home_lat:.6f}, {self.home_lon:.6f})")
        self.get_logger().info(f" Waypoints  : {self.num_waypoints} (+ home)")
        self.get_logger().info(f" Detection  : {self.detection_mode}")
        if self.detection_mode == "model":
            self.get_logger().info(f" Model      : {self.model_path}")
        elif self.detection_mode == "color":
            self.get_logger().info(f" HSV Low    : {self.color_hsv_low}")
            self.get_logger().info(f" HSV High   : {self.color_hsv_high}")
        self.get_logger().info("----------------------------------------")
        for wp in self.waypoints:
            self.get_logger().info(
                f"  WP{wp['id']:2d} : x={wp['x']:8.2f} m, "
                f"y={wp['y']:8.2f} m, alt={wp['z']:.1f} m"
            )
        self.get_logger().info("========================================")


def main(args=None):

    rclpy.init(args=args)
    node = GpsToLocal()
    # Waypoints are written in __init__ — no need to spin.
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
