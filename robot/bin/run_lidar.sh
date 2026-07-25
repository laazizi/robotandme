#!/bin/bash
# Lidar LSLidar N10 : driver -> /scan_raw, scan_fix -> /scan.
# TF statiques : voir run_tf.sh (service dedie).
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
WS="${MOWBOT_LIDAR_WS:-$HOME/lidar_ws/install}"
export LD_LIBRARY_PATH="$WS/lslidar_msgs/lib:$WS/lslidar_driver/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="$WS/lslidar_driver:$WS/lslidar_msgs:$AMENT_PREFIX_PATH"

mowbot_wait_dev "$DEV_LIDAR" 20 || mowbot_log "ATTENTION : $DEV_LIDAR absent"
pkill -f scan_fix.py 2>/dev/null; sleep 1
python3 "$MOWBOT_NODES/scan_fix.py" > "$MOWBOT_LOGS/scan_fix.log" 2>&1 &

# binaire appele en direct : `ros2 run` echoue en contexte systemd
exec "$WS/lslidar_driver/lib/lslidar_driver/lslidar_driver_node" \
  --ros-args --params-file "$MOWBOT_CONFIG/lidar_n10.yaml"
