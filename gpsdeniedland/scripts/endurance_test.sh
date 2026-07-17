#!/bin/bash
# ==========================================================
#  Endurance Test — 8 consecutive takeoff+hover+land cycles
#  Each cycle: takeoff → hover 150s → land → wait → repeat
# ==========================================================

ALT=${1:-3.0}           # altitude (m)
HOVER=${2:-150}         # hover time (s) per cycle
CYCLES=8
PAUSE_BETWEEN=3         # seconds between cycles

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

# Source ROS
source /opt/ros/jazzy/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo -e "${RED}Workspace not built. Run: cd $WS_DIR && colcon build${NC}"
    exit 1
fi

# Find the takeoff_land script
TAKEOFF_SCRIPT="$SCRIPT_DIR/test_takeoff_land.sh"
if [ ! -f "$TAKEOFF_SCRIPT" ]; then
    TAKEOFF_SCRIPT="$HOME/gpstest/scripts/test_takeoff_land.sh"
fi
if [ ! -f "$TAKEOFF_SCRIPT" ]; then
    echo -e "${RED}test_takeoff_land.sh not found!${NC}"
    exit 1
fi

echo ""
echo -e "${CYAN}╔═══════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║       ENDURANCE TEST — $CYCLES CYCLES               ║${NC}"
echo -e "${CYAN}╠═══════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC}  Altitude      : ${BOLD}${ALT} m${NC}"
echo -e "${CYAN}║${NC}  Hover time    : ${BOLD}${HOVER} s${NC} per cycle"
echo -e "${CYAN}║${NC}  Cycles        : ${BOLD}${CYCLES}${NC}"
echo -e "${CYAN}║${NC}  Pause between : ${BOLD}${PAUSE_BETWEEN} s${NC}"
echo -e "${CYAN}║${NC}  Total flight  : ${BOLD}~$((CYCLES * (HOVER + 30))) s${NC} (estimated)"
echo -e "${CYAN}║${NC}  Script        : ${TAKEOFF_SCRIPT}"
echo -e "${CYAN}╚═══════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}  SAFETY: Ctrl+C at any time to abort current cycle${NC}"
echo -e "${BOLD}  Press ENTER to begin endurance test...${NC}"
read -r

LOG_DIR="$WS_DIR/logs"
mkdir -p "$LOG_DIR"
SUMMARY_LOG="$LOG_DIR/endurance_$(date +%Y%m%d_%H%M%S).log"

echo "# Endurance Test — $CYCLES cycles, ${ALT}m alt, ${HOVER}s hover" > "$SUMMARY_LOG"
echo "# Started: $(date)" >> "$SUMMARY_LOG"
echo "# cycle, start_time, end_time, duration_s, status" >> "$SUMMARY_LOG"

PASSED=0
FAILED=0

for i in $(seq 1 $CYCLES); do
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BOLD}  CYCLE $i / $CYCLES${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

    START_TIME=$(date +%s)
    START_STR=$(date +%H:%M:%S)

    # Run takeoff_land
    bash "$TAKEOFF_SCRIPT" "$ALT" "$HOVER"
    EXIT_CODE=$?

    END_TIME=$(date +%s)
    END_STR=$(date +%H:%M:%S)
    DURATION=$((END_TIME - START_TIME))

    if [ $EXIT_CODE -eq 0 ]; then
        STATUS="PASS"
        PASSED=$((PASSED + 1))
        echo -e "  ${GREEN}✓ Cycle $i PASSED (${DURATION}s)${NC}"
    else
        STATUS="FAIL(exit=$EXIT_CODE)"
        FAILED=$((FAILED + 1))
        echo -e "  ${RED}✗ Cycle $i FAILED (exit=$EXIT_CODE, ${DURATION}s)${NC}"
    fi

    echo "$i, $START_STR, $END_STR, $DURATION, $STATUS" >> "$SUMMARY_LOG"

    # Pause between cycles (except after last)
    if [ $i -lt $CYCLES ]; then
        echo -e "  ${YELLOW}Pausing ${PAUSE_BETWEEN}s before next cycle...${NC}"
        echo -e "  ${YELLOW}(Ctrl+C to stop here)${NC}"
        sleep "$PAUSE_BETWEEN"
    fi
done

echo "" >> "$SUMMARY_LOG"
echo "# Completed: $(date)" >> "$SUMMARY_LOG"
echo "# Passed: $PASSED / $CYCLES" >> "$SUMMARY_LOG"
echo "# Failed: $FAILED / $CYCLES" >> "$SUMMARY_LOG"

echo ""
echo -e "${CYAN}╔═══════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║       ENDURANCE TEST COMPLETE                 ║${NC}"
echo -e "${CYAN}╠═══════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC}  Passed : ${GREEN}${PASSED}${NC} / ${CYCLES}"
echo -e "${CYAN}║${NC}  Failed : ${RED}${FAILED}${NC} / ${CYCLES}"
echo -e "${CYAN}║${NC}  Log    : ${SUMMARY_LOG}"
echo -e "${CYAN}╚═══════════════════════════════════════════════╝${NC}"
echo ""
