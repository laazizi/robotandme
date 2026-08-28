#!/bin/bash
# EKF robot_localization : /odom (vx) + gyro (vyaw) -> /odometry/filtered + TF.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
# Garde-fou IMU : republie /imu/data_raw sur /imu/data_checked en annulant le
# lacet si le gyro est gele (cf. nodes/imu_guard.py). L'EKF lit le topic filtre.
pkill -f imu_guard.py 2>/dev/null; sleep 1
python3 "$MOWBOT_NODES/imu_guard.py" > "$MOWBOT_LOGS/imu_guard.log" 2>&1 &

exec ros2 run robot_localization ekf_node --ros-args --params-file "$MOWBOT_CONFIG/ekf.yaml"
