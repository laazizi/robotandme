#!/bin/bash
# SLAM : LOCALISATION si une carte existe, sinon CARTOGRAPHIE.
# Forcer la cartographie : MOWBOT_SLAM_MODE=mapping run_slam.sh
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
MAP="$MOWBOT_MAPS/mowbot.posegraph"
CFG="$MOWBOT_CONFIG/slam_params.yaml"
if [ -f "$MAP" ] && [ "${MOWBOT_SLAM_MODE}" != "mapping" ]; then
  CFG="$MOWBOT_CONFIG/slam_params_loc.yaml"
  mowbot_log "SLAM en LOCALISATION (carte $MAP)"
else
  mowbot_log "SLAM en CARTOGRAPHIE (carte vierge)"
fi
exec ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file "$CFG"
