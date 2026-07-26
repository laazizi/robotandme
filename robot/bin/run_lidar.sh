#!/bin/bash
# Lidar -> /scan_raw, puis scan_fix.py -> /scan.
# TF statiques : voir run_tf.sh (service dedie).
#
# DEUX MODELES SUPPORTES, choisis par $MOWBOT_LIDAR (sinon auto-detection) :
#   n10   LSLidar N10   : 10 Hz, 450 pts/tour, 12 m — driver C++ dans ~/lidar_ws
#   ld14  LDRobot LD14  :  6 Hz, 391 pts/tour,  8 m — noeud Python, rien a compiler
# L'auto-detection retient le N10 si son driver est compile, sinon le LD14
# (cas de la Raspberry Pi, ou compiler du C++ est long et risque en memoire).
# Forcer un modele :  export MOWBOT_LIDAR=ld14   (ou n10)
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"

WS="${MOWBOT_LIDAR_WS:-$HOME/lidar_ws/install}"
N10_BIN="$WS/lslidar_driver/lib/lslidar_driver/lslidar_driver_node"

MODEL="${MOWBOT_LIDAR:-}"
# modele reconnu lors du dernier `mowbot detect` (sonde du flux serie)
if [ -z "$MODEL" ] && [ -f "$MOWBOT_HOME/lidar_model.env" ]; then
  . "$MOWBOT_HOME/lidar_model.env"
  MODEL="${MOWBOT_LIDAR:-}"
  [ -n "$MODEL" ] && mowbot_log "modele issu de la detection : $MODEL"
fi
if [ -z "$MODEL" ]; then
  if [ -x "$N10_BIN" ]; then MODEL=n10; else MODEL=ld14; fi
  mowbot_log "modele de lidar suppose : $MODEL"
fi
# LD06/LD19 : meme protocole que le LD14, seule la vitesse serie change
[ "$MODEL" = "ld06" ] && MODEL=ld14 && LD_BAUD=230400

mowbot_wait_dev "$DEV_LIDAR" 20 || mowbot_log "ATTENTION : $DEV_LIDAR absent"

# scan_fix : nombre de points fixe, masquage du robot, filtre des faux echos
pkill -f scan_fix.py 2>/dev/null; sleep 1
python3 "$MOWBOT_NODES/scan_fix.py" > "$MOWBOT_LOGS/scan_fix.log" 2>&1 &

case "$MODEL" in
  n10)
    if [ ! -x "$N10_BIN" ]; then
      mowbot_log "ERREUR : driver N10 absent ($N10_BIN). Le compiler dans ~/lidar_ws,"
      mowbot_log "         ou basculer sur le LD14 : export MOWBOT_LIDAR=ld14"
      exit 1
    fi
    export LD_LIBRARY_PATH="$WS/lslidar_msgs/lib:$WS/lslidar_driver/lib:$LD_LIBRARY_PATH"
    export AMENT_PREFIX_PATH="$WS/lslidar_driver:$WS/lslidar_msgs:$AMENT_PREFIX_PATH"
    # binaire appele en direct : `ros2 run` echoue en contexte systemd
    exec "$N10_BIN" --ros-args --params-file "$MOWBOT_CONFIG/lidar_n10.yaml"
    ;;
  ld14)
    # LD_BAUD n'est defini que pour un LD06/LD19 (230400) : sinon le YAML decide
    exec python3 "$MOWBOT_NODES/ld14_node.py" \
      --ros-args --params-file "$MOWBOT_CONFIG/lidar_ld14.yaml" \
      ${LD_BAUD:+-p baudrate:=$LD_BAUD}
    ;;
  *)
    mowbot_log "ERREUR : MOWBOT_LIDAR inconnu ('$MODEL'). Attendu : n10 ou ld14."
    exit 1
    ;;
esac
