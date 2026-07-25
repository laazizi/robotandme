#!/bin/bash
# TF statiques : base_link -> laser_link et -> imu_link.
# Regroupees ici (service dedie) car : `ros2 run` echoue sous systemd, et
# lancees depuis les scripts capteurs elles s'empilaient a chaque redemarrage
# -> TF concurrentes -> le SLAM rejetait tous les scans.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
# Position du lidar : 10 cm devant le centre, 28 cm de haut.
"$STATIC_TF_BIN" --x 0.10 --y 0 --z 0.28 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id laser_link &
"$STATIC_TF_BIN" --x 0 --y 0 --z 0.05 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id imu_link &
wait
