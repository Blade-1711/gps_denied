#!/bin/bash

# ==========================================================
#  Takeoff & Land Test — single command, reliable
# ==========================================================
#
#  Uses ros2 launch (not backgrounded ros2 run) so MAVROS is
#  cleanly shut down on Ctrl+C — no leftover mavros_node holding
#  the serial port. This is the robust fix for the port-busy bug.
#
#  Usage:
#    ./test_takeoff_land.sh
#    ./test_takeoff_land.sh 0.5          # altitude 0.5 m
#    ./test_takeoff_land.sh 0.5 8.0      # altitude 0.5 m, hover 8 s
#
#  Logs saved to: ~/gps_navigation_ws/logs/takeoff_test_<timestamp>.log
#
# ==========================================================

# ----------------------------------------------------------
# Tunable Parameters
# ----------------------------------------------------------
ALTITUDE=$(echo "${1:-2.0}" | awk '{printf "%.1f", $1}')  # ensure float
HOLD_TIME=$(echo "${2:-5.0}" | awk '{printf "%.1f", $1}')  # ensure float
CLIMB_VEL="0.4"                 # m/s

FCU_URL=""                      # auto-detected below
BAUD="921600"

# ----------------------------------------------------------
# Colors
# ----------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
NC='\033[0m'

# ----------------------------------------------------------
# Source ROS 2 and workspace
# ----------------------------------------------------------
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo -e "${RED}ERROR: ROS 2 Jazzy not found${NC}"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
elif [ -f "$HOME/gps_navigation_ws/install/setup.bash" ]; then
    source "$HOME/gps_navigation_ws/install/setup.bash"
    WS_DIR="$HOME/gps_navigation_ws"
else
    echo -e "${RED}ERROR: Workspace not built. Run 'colcon build' first.${NC}"
    exit 1
fi

# ----------------------------------------------------------
# Auto-detect Pixhawk port
# ----------------------------------------------------------
detect_pixhawk() {
    for port in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1; do
        if [ -e "$port" ]; then
            echo "$port"
            return 0
        fi
    done
    return 1
}

if [ -z "$FCU_URL" ]; then
    DETECTED_PORT=$(detect_pixhawk) || true
    if [ -n "$DETECTED_PORT" ]; then
        FCU_URL="${DETECTED_PORT}:${BAUD}"
        echo -e "  ${GREEN}✓ Pixhawk detected: ${DETECTED_PORT}${NC}"
    else
        echo -e "${RED}  ✗ No Pixhawk found (checked /dev/ttyACM*, /dev/ttyUSB*)${NC}"
        exit 1
    fi
fi

# ----------------------------------------------------------
# Logging
# ----------------------------------------------------------
LOG_DIR="$WS_DIR/logs"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/takeoff_test_${TIMESTAMP}.log"

# ----------------------------------------------------------
# Header
# ----------------------------------------------------------
echo ""
echo -e "${CYAN}╔══════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║     TAKEOFF & LAND TEST                  ║${NC}"
echo -e "${CYAN}╠══════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC}  Altitude   : ${BOLD}${ALTITUDE} m${NC}"
echo -e "${CYAN}║${NC}  Hold Time  : ${BOLD}${HOLD_TIME} s${NC}"
echo -e "${CYAN}║${NC}  Climb Vel  : ${BOLD}${CLIMB_VEL} m/s${NC}"
echo -e "${CYAN}║${NC}  FCU URL    : ${BOLD}${FCU_URL}${NC}"
echo -e "${CYAN}║${NC}  Log File   : ${DIM}${LOG_FILE}${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════╝${NC}"
echo ""

# ----------------------------------------------------------
# Clear stale MAVROS + set port permissions
# ----------------------------------------------------------
if pgrep -f mavros_node > /dev/null 2>&1; then
    echo -e "  ${YELLOW}Clearing stale mavros_node...${NC}"
    pkill -9 -f mavros_node 2>/dev/null || true
    sleep 3
fi

PORT_PATH=$(echo "$FCU_URL" | cut -d: -f1)
if [ -e "$PORT_PATH" ]; then
    sudo chmod 666 "$PORT_PATH" 2>/dev/null || true
fi

# ----------------------------------------------------------
# Safety check
# ----------------------------------------------------------
echo -e "  ${YELLOW}PRE-FLIGHT CHECK:${NC}"
echo -e "    □ RC transmitter ON and linked?"
echo -e "    □ Propellers secure?"
echo -e "    □ Area clear?"
echo -e "    □ Battery OK?"
echo ""
echo -e "  ${GREEN}Press ENTER to start (Ctrl+C to abort)${NC}"
read -r
echo ""

# ----------------------------------------------------------
# Launch (foreground) — ros2 launch manages MAVROS + node
# lifecycle and cleans up cleanly on Ctrl+C (no zombies).
# ----------------------------------------------------------
echo -e "${CYAN}──────────────────────────────────────────${NC}"
echo -e "${CYAN}  Launching MAVROS + Takeoff/Land test${NC}"
echo -e "${CYAN}  (MAVROS takes a few seconds to connect)${NC}"
echo -e "${CYAN}──────────────────────────────────────────${NC}"
echo ""

# Force line-buffered logging so telemetry appears in real time
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1

ros2 launch mission_executive takeoff_test.launch.py \
    fcu_url:="${FCU_URL}" \
    alt:="${ALTITUDE}" \
    hover:="${HOLD_TIME}" \
    climb_rate:="${CLIMB_VEL}" \
    2>&1 | tee -a "$LOG_FILE"

# ----------------------------------------------------------
# Cleanup after launch exits
# ----------------------------------------------------------
echo ""
echo -e "${YELLOW}  Launch exited — clearing any leftover mavros_node...${NC}"
pkill -9 -f mavros_node 2>/dev/null || true

echo ""
echo -e "${GREEN}  Log saved: ${LOG_FILE}${NC}"
if grep -qi "error\|fail\|exception\|traceback" "$LOG_FILE" 2>/dev/null; then
    echo -e "${RED}  ⚠ Errors found — check log${NC}"
fi
echo ""
