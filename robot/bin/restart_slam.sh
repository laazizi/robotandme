#!/bin/bash
# Relance PROPRE du SLAM seul (kill garanti + verification).
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
pkill -9 -f async_slam_toolbox 2>/dev/null; sleep 3
rm -f "$MOWBOT_LOGS/slam.log"
setsid bash "$MOWBOT_BIN/run_slam.sh" > "$MOWBOT_LOGS/slam.log" 2>&1 < /dev/null &
sleep 20
grep -m1 "SLAM en" "$MOWBOT_LOGS/slam.log"
printf "  scans rejetes : %s\n" "$(grep -c 'expected' "$MOWBOT_LOGS/slam.log")"
printf "  /map          : %s Hz\n" "$(mowbot_hz /map)"
