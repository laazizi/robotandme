#!/bin/bash
# Sauvegarde la carte : posegraph (SLAM) + pgm/yaml (nav2/visualisation).
# Usage : save_map.sh [nom]     (defaut mowbot)
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
NAME="${1:-mowbot}"
mowbot_log "sauvegarde posegraph..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: $MOWBOT_MAPS/$NAME}" 2>&1 | tail -1
mowbot_log "sauvegarde image..."
ros2 run nav2_map_server map_saver_cli -f "$MOWBOT_MAPS/$NAME" \
  --ros-args -p save_map_timeout:=10.0 2>&1 | tail -1
ls -la "$MOWBOT_MAPS" | grep "$NAME"
