#!/bin/bash
# EKF robot_localization : /odom (vx) + gyro (vyaw) -> /odometry/filtered + TF.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
exec ros2 run robot_localization ekf_node --ros-args --params-file "$MOWBOT_CONFIG/ekf.yaml"
