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
# Portee du lidar : slam_params.yaml est cale sur le N10 (12 m). Le LD14 ne
# porte qu'a 8 m ; annoncer 12 m ferait considerer comme "espace libre observe"
# une zone que le capteur ne voit pas, et polluerait la carte.
[ -f "$MOWBOT_HOME/lidar_model.env" ] && . "$MOWBOT_HOME/lidar_model.env"
case "${MOWBOT_LIDAR:-}" in
  ld14)       RANGE_ARG="-p max_laser_range:=8.0" ;;
  ld06|ld19)  RANGE_ARG="-p max_laser_range:=12.0" ;;
  *)          RANGE_ARG="" ;;
esac
[ -n "$RANGE_ARG" ] && mowbot_log "portee lidar transmise au SLAM : ${RANGE_ARG#-p max_laser_range:=} m"

exec ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args --params-file "$CFG" $RANGE_ARG
