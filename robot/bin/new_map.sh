#!/bin/bash
# NOUVELLE CARTE : archive l'actuelle et repart en cartographie.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
if ls "$MOWBOT_MAPS"/mowbot.* >/dev/null 2>&1; then
  BK="$MOWBOT_MAPS/archive_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$BK" && mv "$MOWBOT_MAPS"/mowbot.* "$BK"/ 2>/dev/null
  mowbot_log "ancienne carte archivee : $BK"
else
  mowbot_log "aucune carte a archiver"
fi
pkill -9 -f async_slam_toolbox 2>/dev/null; sleep 3
rm -f "$MOWBOT_LOGS/slam.log"
setsid env MOWBOT_SLAM_MODE=mapping bash "$MOWBOT_BIN/run_slam.sh" \
  > "$MOWBOT_LOGS/slam.log" 2>&1 < /dev/null &
sleep 18
grep -m1 "SLAM en" "$MOWBOT_LOGS/slam.log"
printf "  /map : %s Hz\n" "$(mowbot_hz /map)"
echo ">> roule dans le lieu, puis :  mowbot save-map"
