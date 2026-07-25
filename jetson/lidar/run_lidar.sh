#!/bin/bash
# Driver lidar LSLidar N10 -> /scan (frame laser_link) + TF base_link->laser_link.
# Binaire lance DIRECTEMENT (ros2 run echoue en contexte detache).
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
WS=/home/nvidia/lidar_ws/install
export LD_LIBRARY_PATH="$WS/lslidar_msgs/lib:$WS/lslidar_driver/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="$WS/lslidar_driver:$WS/lslidar_msgs:$AMENT_PREFIX_PATH"

# Position du lidar sur le robot (A AJUSTER : x avant, z hauteur, yaw si le
# zero du lidar ne pointe pas vers l'avant -> --yaw 3.14159 si inverse).
setsid ros2 run tf2_ros static_transform_publisher \
  --x 0.10 --y 0 --z 0.28 --roll 0 --pitch 0 --yaw 3.14159 \
  --frame-id base_link --child-frame-id laser_link \
  < /dev/null > /dev/null 2>&1 &

exec "$WS/lslidar_driver/lib/lslidar_driver/lslidar_driver_node" \
  --ros-args --params-file /home/nvidia/lidar_n10.yaml
