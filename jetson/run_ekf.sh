#!/bin/bash
# Lance l'EKF robot_localization (fusion odom + gyro). Utilise par mowbot-ekf.service.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
exec ros2 run robot_localization ekf_node --ros-args --params-file /home/nvidia/ekf.yaml
