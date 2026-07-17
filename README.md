# GPS-Denied Navigation + Precision Landing

Autonomous drone system for GPS-denied environments using optical flow navigation
and ArUco/YOLO marker-based precision landing.

## Workspaces

| Directory | Description |
|-----------|-------------|
| `gpsdeniedland/` | Final integrated system — navigation + precision landing + smart exclusion |
| `gps_land/` | Identical to gpsdeniedland (synced) |

Both workspaces are identical and contain the full system.

## Quick Start

```bash
cd gpsdeniedland   # or gps_land
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
./scripts/run_mission.sh
```

See each workspace's README.md for full details.
