#!/bin/bash

# ==========================================================
#  GPS-Denied Navigation — Full Mission (single command)
# ==========================================================
#
#  Flow (all from one terminal):
#    1. Clear stale MAVROS, set port permissions
#    2. gps_to_local — you type the waypoints; it writes them to a JSON file
#    3. ros2 launch — starts MAVROS + all flight nodes + mission_executive
#       (mission_executive reads the waypoints file)
#    4. You confirm, then it sends the start command
#
#  Uses ros2 launch (not backgrounded ros2 run) so MAVROS is cleanly
#  shut down on Ctrl+C — no leftover mavros_node holding the port.
#
#  Usage:
#    ./run_mission.sh
#    ./run_mission.sh 3.0        # default mission altitude 3.0 m
#
#  Logs: ~/gps_navigation_ws/logs/mission_<timestamp>.log
# ==========================================================

# ----------------------------------------------------------
# Tunable Parameters
# ----------------------------------------------------------
MISSION_ALTITUDE=${1:-3.0}      # meters (default flight altitude)
CLIMB_VELOCITY="0.4"            # m/s
WAYPOINTS_FILE="/tmp/gps_mission.json"

FCU_URL=""                      # auto-detected
BAUD="921600"

# ----------------------------------------------------------
# Colors
# ----------------------------------------------------------
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; DIM='\033[2m'; NC='\033[0m'

# ----------------------------------------------------------
# Source ROS 2 + workspace
# ----------------------------------------------------------
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo -e "${RED}ERROR: ROS 2 Jazzy not found${NC}"; exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
elif [ -f "$HOME/gps_land/install/setup.bash" ]; then
    source "$HOME/gps_land/install/setup.bash"
    WS_DIR="$HOME/gps_land"
else
    echo -e "${RED}ERROR: Workspace not built. Run 'colcon build' first.${NC}"; exit 1
fi

# ----------------------------------------------------------
# Auto-detect Pixhawk port
# ----------------------------------------------------------
detect_pixhawk() {
    for port in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1; do
        [ -e "$port" ] && { echo "$port"; return 0; }
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
LOG_DIR="$WS_DIR/logs"; mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/mission_${TIMESTAMP}.log"

echo ""
echo -e "${CYAN}╔══════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║   GPS-DENIED NAVIGATION MISSION          ║${NC}"
echo -e "${CYAN}╠══════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC}  FCU URL     : ${BOLD}${FCU_URL}${NC}"
echo -e "${CYAN}║${NC}  Altitude    : ${BOLD}${MISSION_ALTITUDE} m${NC}"
echo -e "${CYAN}║${NC}  Waypoints   : ${BOLD}${WAYPOINTS_FILE}${NC}"
echo -e "${CYAN}║${NC}  Log File    : ${DIM}${LOG_FILE}${NC}"
echo -e "${CYAN}║${NC}  Camera Feed : ${BOLD}http://$(hostname -I | awk '{print $1}'):5000${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════╝${NC}"
echo ""

# ----------------------------------------------------------
# Cleanup / KILL SWITCH
# On Ctrl+C: command a controlled DESCENT at the current
# position, wait for the drone to land & disarm, THEN shut down.
# (We do NOT just kill the nodes — that would drop a flying drone.)
# ----------------------------------------------------------
LAUNCH_PID=""
TAIL_PID=""
ABORTING=0
cleanup() {
    # Stop the telemetry stream
    [ -n "$TAIL_PID" ] && kill "$TAIL_PID" 2>/dev/null || true

    # Guard against re-entry (double Ctrl+C)
    if [ "$ABORTING" -eq 1 ]; then
        echo -e "${RED}  Force stopping...${NC}"
        pkill -9 -f mavros_node 2>/dev/null || true
        pkill -9 -f mission_executive_node 2>/dev/null || true
        pkill -9 -f guidance_controller_node 2>/dev/null || true
        pkill -9 -f command_bridge_node 2>/dev/null || true
        pkill -9 -f state_bridge 2>/dev/null || true
        pkill -9 -f precision_landing_node 2>/dev/null || true
        pkill -9 -f camera_publisher 2>/dev/null || true
        exit 1
    fi
    ABORTING=1

    echo ""
    echo -e "${RED}${BOLD}  ⚠ KILL SWITCH — controlled descent at current position${NC}"

    # Tell mission_executive to abort -> controlled descent (publish a few times)
    for _ in 1 2 3; do
        ros2 topic pub --once /mission/abort std_msgs/msg/Bool "data: true" \
            >> "$LOG_FILE" 2>&1
        sleep 0.3
    done

    # Wait for disarm (landed). RC transmitter can override anytime.
    echo -e "${YELLOW}  Landing... (waiting for disarm, Ctrl+C again to force stop)${NC}"
    for i in $(seq 1 40); do
        ARMED=$(ros2 topic echo /mavros/state mavros_msgs/msg/State \
            --once --timeout 2 2>/dev/null | grep -m1 "armed:" | awk '{print $2}')
        if [ "$ARMED" = "false" ]; then
            echo -e "${GREEN}  ✓ Landed & disarmed${NC}"
            break
        fi
        echo -e "${DIM}  landing... ${i}s${NC}"
        sleep 1
    done

    echo -e "${YELLOW}  Shutting down nodes...${NC}"
    [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 3
    pkill -9 -f mavros_node 2>/dev/null || true
    pkill -9 -f mission_executive_node 2>/dev/null || true
    pkill -9 -f guidance_controller_node 2>/dev/null || true
    pkill -9 -f command_bridge_node 2>/dev/null || true
    pkill -9 -f state_bridge 2>/dev/null || true
    pkill -9 -f precision_landing_node 2>/dev/null || true
    pkill -9 -f camera_publisher 2>/dev/null || true
    echo -e "${GREEN}  ✓ Stopped. Log: ${LOG_FILE}${NC}"
    echo ""
    exit 0
}
trap cleanup SIGINT SIGTERM

# ----------------------------------------------------------
# Step 1: Clear stale MAVROS + all mission nodes + port permissions
# ----------------------------------------------------------
# Kill ANY leftover nodes from a previous/crashed run. A stale
# mission_executive would publish /mission/nav_enabled=False in parallel
# with the new one, causing command_bridge to flip-flop.
STALE_PATTERNS="mavros_node mission_executive_node guidance_controller_node command_bridge_node state_bridge gps_to_local precision_landing_node camera_publisher"
FOUND_STALE=0
for pat in $STALE_PATTERNS; do
    if pgrep -f "$pat" > /dev/null 2>&1; then
        FOUND_STALE=1
    fi
done
if [ "$FOUND_STALE" -eq 1 ]; then
    echo -e "  ${YELLOW}Clearing stale nodes from a previous run...${NC}"
    for pat in $STALE_PATTERNS; do
        pkill -9 -f "$pat" 2>/dev/null || true
    done
    sleep 3
fi
PORT_PATH=$(echo "$FCU_URL" | cut -d: -f1)
[ -e "$PORT_PATH" ] && sudo chmod 666 "$PORT_PATH" 2>/dev/null || true

# ----------------------------------------------------------
# Step 2: Enter waypoints (interactive) -> writes JSON file
# ----------------------------------------------------------
echo -e "${CYAN}──────────────────────────────────────────${NC}"
echo -e "${CYAN}  STEP 1: Enter Mission Waypoints${NC}"
echo -e "${CYAN}──────────────────────────────────────────${NC}"

ros2 run gps_to_local gps_to_local_node \
    --ros-args -p output_file:="${WAYPOINTS_FILE}" 2>&1 | tee -a "$LOG_FILE"

if [ ! -f "$WAYPOINTS_FILE" ]; then
    echo -e "${RED}  ✗ Waypoints file not created — aborting.${NC}"
    exit 1
fi
echo -e "  ${GREEN}✓ Waypoints saved${NC}"
echo ""

# Extract detection config from waypoints JSON
DETECTION_MODE=$(python3 -c "import json; d=json.load(open('${WAYPOINTS_FILE}')); print(d.get('detection_mode','aruco'))" 2>/dev/null || echo "aruco")
MODEL_PATH=$(python3 -c "import json; d=json.load(open('${WAYPOINTS_FILE}')); print(d.get('model_path',''))" 2>/dev/null || echo "")

echo -e "  ${CYAN}Detection: ${BOLD}${DETECTION_MODE}${NC}"
if [ "$DETECTION_MODE" = "model" ] && [ -n "$MODEL_PATH" ]; then
    echo -e "  ${CYAN}Model:     ${BOLD}${MODEL_PATH}${NC}"
    if [ ! -f "$MODEL_PATH" ]; then
        echo -e "  ${RED}⚠ WARNING: Model file not found at ${MODEL_PATH}${NC}"
        echo -e "  ${YELLOW}  Will fall back to ArUco if model fails to load${NC}"
    fi
fi
echo ""

# Ask whether to return home after the last waypoint
RETURN_HOME="true"
read -r -p "  Return to home after last waypoint? [Y/n]: " ANS
case "$ANS" in
    [Nn]*) RETURN_HOME="false"
           echo -e "  ${YELLOW}→ Mission will END at the last waypoint${NC}" ;;
    *)     RETURN_HOME="true"
           echo -e "  ${GREEN}→ Mission will RETURN HOME after last waypoint${NC}" ;;
esac
echo ""

# ----------------------------------------------------------
# Step 3: Launch MAVROS + all flight nodes
# ----------------------------------------------------------
echo -e "${CYAN}──────────────────────────────────────────${NC}"
echo -e "${CYAN}  STEP 2: Launching MAVROS + flight nodes${NC}"
echo -e "${CYAN}  (MAVROS takes a few seconds to connect)${NC}"
echo -e "${CYAN}──────────────────────────────────────────${NC}"
echo ""

# Force line-buffered, unbuffered logging so telemetry appears on screen in
# real time (otherwise ROS logging is block-buffered when redirected to a file
# and appears in delayed bursts, also delaying start-command detection).
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1

# Start the launch in its OWN session (setsid) so that pressing Ctrl+C
# in this terminal does NOT directly kill MAVROS/nodes. Instead our
# trap runs the controlled AUTO.LAND kill switch first, then shuts down.
setsid ros2 launch mission_executive mission.launch.py \
    fcu_url:="${FCU_URL}" \
    waypoints_file:="${WAYPOINTS_FILE}" \
    mission_altitude:="${MISSION_ALTITUDE}" \
    climb_velocity:="${CLIMB_VELOCITY}" \
    return_home:="${RETURN_HOME}" \
    detection_mode:="${DETECTION_MODE}" \
    ${MODEL_PATH:+model_path:="${MODEL_PATH}"} \
    auto_start:=false \
    >> "$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

# Wait for mission_executive to be ready (it prints preflight waiting)
echo -e "  ${DIM}Waiting for nodes to come up...${NC}"
sleep 12

# ----------------------------------------------------------
# Step 4: Safety check + start
# ----------------------------------------------------------
echo -e "  ${YELLOW}SAFETY CHECK:${NC}"
echo -e "    □ RC transmitter ON and linked?"
echo -e "    □ Area clear of people/obstacles?"
echo -e "    □ Battery charged?  □ Propellers secure?"
echo -e "    □ COM_DISARM_LAND = 0 set on FCU (stay armed between WPs)?"
echo ""
echo -e "  ${GREEN}${BOLD}Press ENTER to START mission (Ctrl+C to abort)${NC}"
read -r

# Publish /mission/start repeatedly until mission_executive confirms it
# received it. MAVROS floods DDS discovery, so a single --once publish can
# race and be lost; we retry and verify against the log.
echo -e "  ${DIM}Sending start command...${NC}"
STARTED=0
for attempt in $(seq 1 15); do
    ros2 topic pub --once /mission/start std_msgs/msg/String "data: go" \
        >> "$LOG_FILE" 2>&1
    sleep 0.5
    if grep -q "Start command received" "$LOG_FILE" 2>/dev/null; then
        STARTED=1
        break
    fi
done

if [ "$STARTED" -eq 1 ]; then
    echo -e "  ${GREEN}✓ MISSION STARTED${NC}"
else
    echo -e "  ${RED}✗ mission_executive did not acknowledge start after 15 tries${NC}"
    echo -e "  ${YELLOW}  Check the log; aborting.${NC}"
    cleanup
fi
echo ""

# ----------------------------------------------------------
# Step 5: Stream live mission telemetry to the terminal
# ----------------------------------------------------------
echo -e "${CYAN}──────────────────────────────────────────${NC}"
echo -e "${CYAN}  Mission running — live telemetry${NC}"
echo -e "${CYAN}  (Ctrl+C to abort — controlled descent + disarm)${NC}"
echo -e "${CYAN}──────────────────────────────────────────${NC}"
echo ""

# Stream mission_executive log lines live to the terminal (strip ROS prefix).
# The launch writes everything to the log file; we tail+filter just the
# mission_executive telemetry so the screen isn't flooded with MAVROS spam.
stdbuf -oL tail -n +1 -f "$LOG_FILE" 2>/dev/null \
    | grep --line-buffered "\[mission_executive\]:" \
    | stdbuf -oL sed -u 's/.*\[mission_executive\]: //' &
TAIL_PID=$!

# Wait for mission completion (or launch death). Watch the log for the
# completion banner.
while true; do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo -e "  ${RED}Launch process ended.${NC}"
        break
    fi
    if grep -q "MISSION COMPLETE" "$LOG_FILE" 2>/dev/null; then
        sleep 1
        break
    fi
    sleep 1
done

kill "$TAIL_PID" 2>/dev/null || true
cleanup
