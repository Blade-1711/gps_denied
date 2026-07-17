# GPS + Precision Landing Workspace

Autonomous drone navigation + ArUco marker-based precision landing for a
**GPS-denied** environment. Combines waypoint navigation (optical flow) with
visual servoing for precise marker landings at each waypoint.

**Mission:** Home → WP1 → WP2 → … → WP5 → Home, with precision ArUco marker
landing at each waypoint.

---

## What It Does

```
Enter GPS waypoints → Navigate to each one → Detect ArUco marker → Align → Land precisely
                                                                              ↓
                                                            Re-arm → Fly to next waypoint
```

### Key Behaviors

1. **Waypoint navigation** — the drone flies to each waypoint using optical flow
   position estimation and a rotate-then-move strategy.

2. **Mid-transit marker detection** — during navigation, the camera actively
   scans for ArUco markers. If one is detected (that wasn't already landed on),
   the drone stops and precision-lands on it immediately.

3. **Post-waypoint marker search** — if the drone reaches a waypoint without
   seeing a marker en route, it hovers and scans. If not found, it climbs
   in +1m steps (up to 5m max) to widen the camera FOV.

4. **Precision visual landing** — once a marker is found, a tanh-profile
   controller aligns the drone over it using camera→body→ENU frame transforms
   with yaw compensation, then descends on a smooth tanh curve.

5. **Pose correction** — after each landing, the drone treats its current
   position as the planned waypoint. The next leg uses a relative vector
   (WP_next − WP_current), eliminating accumulated drift between legs.

6. **Smart exclusion** — per-leg exclusion list prevents re-landing on
   previously visited markers while allowing detection of the current target
   (including home marker on return leg).

---

## Hardware

| Component | Model | Purpose |
|-----------|-------|---------|
| Flight controller | Pixhawk 6C (PX4) | FCU |
| Companion computer | Jetson Orin Nano | Runs all ROS 2 nodes |
| Optical flow + rangefinder | Holybro H-Flow (DroneCAN) | Position + altitude |
| Down-facing camera | **Logitech C922** or Intel RealSense D435i | ArUco marker detection |
| RC transmitter | Flysky 6-channel | Manual override |
| ArUco markers | DICT_4X4_250, ≥15cm | Printed markers at each waypoint |

### Camera Auto-Detection

The `camera_publisher` node auto-detects whichever camera is connected:
1. **Intel RealSense D435i** — color + depth (preferred if available)
2. **Logitech C922 / any USB webcam** — color only

For USB cameras:
- **Autofocus is automatically disabled** (fixed focus at infinity)
- No manual setup required — just plug in and run

---

## Prerequisites

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- MAVROS: `sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras`
- Python packages: `pip3 install pyrealsense2 --break-system-packages` (optional, for D435i)
- YOLO detection: `pip3 install ultralytics --break-system-packages` (for YOLO model detection)
- OpenCV (included with ROS 2)

---

## Setup

```bash
cd ~/gps_land
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## Required PX4 Parameter

Set via QGroundControl **before flying**:

| Parameter | Value | Why |
|-----------|-------|-----|
| `COM_DISARM_LAND` | `0` | Keeps drone armed between waypoints |

---

## Running

### Full Mission (recommended):

```bash
cd ~/gps_land
./scripts/run_mission.sh              # default 3.0m altitude
./scripts/run_mission.sh 2.0          # 2.0m altitude
```

The script will:
1. Auto-detect Pixhawk
2. Show camera stream URL
3. Launch `gps_to_local` — you enter waypoints + marker detection config interactively
4. Launch all nodes (MAVROS + navigation + precision landing)
5. Safety checklist + ENTER to start
6. Stream live telemetry to terminal
7. Ctrl+C = controlled abort (descent + disarm)

### Input Format

Waypoints are entered as **decimal degrees** (not DMS):
```
Home: 31.783097, 77.229500
WP1:  31.783150, 77.003800
```

After waypoints, you'll be asked for marker detection mode:
```
Detection modes:
  1. aruco  — ArUco marker (DICT_4X4_250)
  2. color  — Color blob (HSV range)
  3. model  — YOLOv8 trained model (.pt or .engine)

Detection mode [aruco]: model
Model file path: /home/TerraWings/models/marker.pt
```

---

## Camera Stream (Live View)

The precision landing node hosts a live MJPEG camera feed:

```
http://<jetson_ip>:5000
```

Shows: annotated camera with crosshair, marker position, confidence %, state, AGL.
The URL is displayed in the startup banner before you enter waypoints.

---

## Architecture

### Packages

| Package | Role |
|---------|------|
| `gps_navigation_interfaces` | Custom messages (Waypoint, VehicleState, GuidanceCommand, GuidanceStatus) |
| `gps_to_local` | Interactive GPS→local ENU converter; writes JSON waypoint file |
| `mission_executive` | Top-level state machine — orchestrates everything |
| `guidance_controller` | Waypoint navigation: rotate-then-move with tanh speed profile |
| `command_bridge` | Converts guidance commands → MAVROS PositionTarget setpoints |
| `px4_adapter` | state_bridge: MAVROS odometry → `/vehicle/state` |
| `precision_landing` | Activation-based ArUco detection + visual alignment + precision descent |

---

## State Machine

```
IDLE → PREFLIGHT → STREAM → OFFBOARD → ARM → CLIMB → HOLD → NAVIGATE ──┐
                                                                          │
                                         ┌────────────────────────────────┤
                                         │                                │
                                  marker detected                  waypoint reached
                                  (not excluded)                          │
                                         │                         SEARCH_HOVER
                                         │                         (scan for marker)
                                         │                                │
                                         │                      found / climb+retry / max
                                         │                                │
                                         ▼                                ▼
                                  PRE_LAND_HOVER (3s momentum kill)
                                         │
                                         ▼
                                  PRECISION_LAND_HANDOFF
                                  (activate precision_landing_node)
                                         │
                                         ▼
                                  WAIT_FOR_LANDING
                                  (precision_landing_node owns setpoints)
                                         │
                                         ▼
                                  POST_LAND
                                  (log drift, exclusion, advance WP)
                                         │
                                  more WPs? → STREAM → ARM → CLIMB → NAVIGATE
                                         │
                                  done → COMPLETE
```

---

## Navigation: Relative Vector Approach

After landing at WP_n, the next target is computed as:

```
target = current_EKF_position + (WP_n+1_planned - WP_n_planned)
```

Each leg is independent — drift from previous legs doesn't accumulate.

---

## Precision Landing Details

### Alignment: Tanh Profile

```
speed = MAX_ALIGN_SPEED × tanh(K_ALIGN × error_magnitude)

MAX_ALIGN_SPEED = 0.3 m/s
K_ALIGN = 2.0

At error 1.0 (frame edge): 0.29 m/s (fast approach)
At error 0.5 (halfway):    0.23 m/s (still fast)
At error 0.1 (near center): 0.06 m/s (gentle)
At error 0.05 (tolerance):  0.03 m/s (creeping)
```

### Frame Transform (Camera → Body → ENU)

```
Camera → Body (FRD):
    body_x = cam_err_y     (image down → body forward)
    body_y = -cam_err_x    (image right → body left)

Body → ENU (using current yaw):
    vx_enu = body_x × cos(yaw) + body_y × sin(yaw)
    vy_enu = body_x × sin(yaw) - body_y × cos(yaw)
```

### Descent Profile (tanh)

```
speed = 0.08 + 0.32 × tanh(1.2 × height_above_ground)

At 3.0m: ~0.39 m/s (fast)
At 1.0m: ~0.34 m/s
At 0.5m: ~0.24 m/s
At 0.1m: ~0.12 m/s (gentle creep)
Touchdown at rangefinder ≤ 0.24m → force disarm
```

### Fallback Convergence (marker lost)

When marker is not visible but position was previously estimated:
```
velocity = KP_CONVERGE × distance_to_estimate, clamped to MAX_CONVERGE_VEL
KP_CONVERGE = 0.3, MAX_CONVERGE_VEL = 0.3 m/s
```

---

## Smart Exclusion Logic

At the start of each leg, `exclusion_positions` is rebuilt:
- All previously landed positions are excluded
- EXCEPT positions near the current target waypoint

This means:
- Flying Home→WP1: home marker excluded, WP1 detectable ✓
- Flying WP1→WP2: home+WP1 excluded, WP2 detectable ✓
- Flying WP_last→Home: all WPs excluded, home marker detectable ✓

---

## Marker Detection Modes

The system supports three detection methods, selectable during mission setup:

### ArUco (default)
Detects ArUco markers from `DICT_4X4_250`. Best for development and testing.
```bash
./scripts/run_mission.sh    # select 'aruco' when prompted
```

### Color Blob
Detects a colored marker using HSV filtering. Configure the HSV range for your marker color.
```
Detection mode: color
HSV range [0 10 100 255 100 255]: 0 10 100 255 100 255   # red marker
```

### YOLO Model (recommended for competition)
Uses a trained YOLOv8n model to detect any marker type (helipad, bullseye, painted X, etc.).

**Pre-flight setup:**
```bash
# On laptop: train on marker images
pip install ultralytics
yolo train model=yolov8n.pt data=marker.yaml epochs=50 imgsz=640

# Copy to Jetson
scp runs/detect/train/weights/best.pt jetson:~/models/marker.pt
```

**On Jetson (one-time):**
```bash
pip3 install ultralytics --break-system-packages
```

**During mission setup:**
```
Detection mode: model
Model file path: /home/TerraWings/models/marker.pt
```

If the model fails to load at runtime, the system automatically falls back to ArUco detection.

---

## Camera FOV Calibration

The FOV values affect marker position estimation accuracy. After changing cameras:

1. Place camera exactly 1m from a wall
2. Measure visible width `W` and height `H` in meters
3. Calculate:
   ```
   FOV_H = 2 × atan(W / 2) × (180/π)  degrees
   FOV_V = 2 × atan(H / 2) × (180/π)  degrees
   ```
4. Update `CAMERA_FOV_H_DEG` and `CAMERA_FOV_V_DEG` in:
   - `precision_landing/precision_landing_node.py`
   - `mission_executive/mission_executive_node.py`

Current values (Logitech C922 estimated): **70° × 43°**

---

## Telemetry (log output)

All states show detailed pose info relative to home (0,0):

```
CLIMB | AGL=1.50/3.00m | pos=(0.00, 0.00) | yaw=45°
NAV WP1 | pos=(0.12, 0.50) target=(5.00, 3.00) | dist=4.95m | AGL=3.00m | yaw 45°→31° err=-14° | → CW (turn RIGHT)
PRE_LAND_HOVER | t=1.5/3.0s | pos=(4.98, 2.97) | AGL=3.00m | yaw=31°
WAIT_FOR_LANDING | PL status: ALIGNING | AGL=2.15m | pos=(5.01, 3.02) | yaw=31°
DRIFT WP1: dx=0.15m dy=-0.08m total=0.17m
```

---

## Parameters

### Alignment & Descent (precision_landing_node)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `MAX_ALIGN_SPEED` | 0.3 m/s | Maximum alignment speed |
| `K_ALIGN` | 2.0 | Tanh alignment steepness |
| `KP_CONVERGE` | 0.3 | Convergence P-gain (marker lost) |
| `MAX_CONVERGE_VEL` | 0.3 m/s | Max convergence speed |
| `MAX_DESCENT_VEL` | 0.4 m/s | Descent speed at height |
| `MIN_DESCENT_VEL` | 0.08 m/s | Touchdown creep speed |
| `DESCENT_TANH_K` | 1.2 | Descent curve steepness |
| `align_tolerance` | 0.05 | Normalized centering tolerance |
| `detection_mode` | aruco | Detection method: aruco, color, or model |
| `model_path` | (empty) | Path to YOLOv8 .pt or .engine file |
| `marker_id` | -1 | ArUco ID (-1 = any) |

### Navigation (mission_executive)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `mission_altitude` | 3.0 m | Navigation altitude |
| `detection_mode` | aruco | Detection method: aruco, color, or model |
| `model_path` | (empty) | Path to YOLOv8 .pt or .engine file |
| `EXCLUSION_RADIUS` | 1.5 m | Ignore markers within this of previous landings |
| `NAV_MARKER_CONFIRM` | 3 frames | Consecutive detections before triggering |
| `PRE_LAND_HOVER_TIME` | 3.0 s | Momentum kill before handoff |
| `MAX_SCAN_ALTITUDE` | 5.0 m | Max altitude for marker search |
| `SCAN_CLIMB_STEP` | 1.0 m | Step per search attempt |
| `MAX_FORWARD_SPEED` | 1.0 m/s | Navigation speed (guidance) |
| `WAYPOINT_RADIUS` | 0.5 m | Waypoint reached distance |

---

## Monitoring

```bash
# Watch mission state:
ros2 topic echo /mission/state

# Watch precision landing:
ros2 topic echo /precision_landing/status

# View camera feed (browser):
http://<jetson_ip>:5000
```

---

## Safety

- **Ctrl+C** → publishes /mission/abort → controlled descent + disarm
- **Double Ctrl+C** → force kills all nodes
- **RC override** → switch out of OFFBOARD for manual control
- **Marker not found** → climbs up to 5m, then blind descent fallback

---

## Directory Structure

```
gps_land/
├── src/
│   ├── gps_navigation_interfaces/
│   ├── gps_to_local/
│   ├── guidance_controller/
│   ├── command_bridge/
│   ├── px4_adapter/
│   ├── mission_executive/
│   │   ├── mission_executive/
│   │   │   └── mission_executive_node.py
│   │   ├── launch/
│   │   │   └── mission.launch.py
│   │   └── config/
│   │       └── mavros.yaml
│   └── precision_landing/
│       ├── precision_landing/
│       │   ├── precision_landing_node.py
│       │   └── camera_publisher.py
│       ├── launch/
│       └── config/
├── scripts/
│   └── run_mission.sh
├── logs/
└── README.md
```

---

## Quick Reference

```bash
# Build
cd ~/gps_land && colcon build && source install/setup.bash

# Run mission
./scripts/run_mission.sh              # 3m altitude
./scripts/run_mission.sh 2.0          # 2m altitude

# Manual start (after launch):
ros2 topic pub --once /mission/start std_msgs/String "data: go"

# Abort:
ros2 topic pub --once /mission/abort std_msgs/msg/Bool "data: true"

# Camera stream:
http://<jetson_ip>:5000
```
