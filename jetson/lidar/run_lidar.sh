#!/bin/bash
# Lidar LSLidar N10 : driver -> /scan_raw, normaliseur -> /scan, + TF.
# Binaire lance DIRECTEMENT (ros2 run echoue en contexte detache).
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
WS=/home/nvidia/lidar_ws/install
export LD_LIBRARY_PATH="$WS/lslidar_msgs/lib:$WS/lslidar_driver/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="$WS/lslidar_driver:$WS/lslidar_msgs:$AMENT_PREFIX_PATH"

# TF : lidar 10 cm devant le centre, 28 cm de haut, retourne (yaw 180)
setsid ros2 run tf2_ros static_transform_publisher \
  --x 0.10 --y 0 --z 0.28 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id laser_link \
  < /dev/null > /dev/null 2>&1 &

# Normaliseur : le N10 sort un nombre de points variable (449/450/451) et
# slam_toolbox rejette les scans de taille differente -> on fixe a 450.
setsid python3 /home/nvidia/scan_fix.py < /dev/null > /tmp/scan_fix.log 2>&1 &

exec "$WS/lslidar_driver/lib/lslidar_driver/lslidar_driver_node" \
  --ros-args --params-file /home/nvidia/lidar_n10.yaml
