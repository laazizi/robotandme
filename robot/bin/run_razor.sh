#!/bin/bash
# IMU Razor AHRS -> /imu/data_raw (gyro seul ; magneto ignore pres des moteurs).
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
mowbot_wait_dev "$DEV_IMU" 20 || mowbot_log "ATTENTION : $DEV_IMU absent"
exec python3 -u "$MOWBOT_NODES/razor_imu_node.py" "$DEV_IMU" 57600
