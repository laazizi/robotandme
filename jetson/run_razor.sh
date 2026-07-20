#!/bin/bash
# Lance le nœud IMU Razor (publie /imu/data_raw). Utilise par mowbot-razor.service.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
exec python3 -u /home/nvidia/razor_imu_node.py /dev/mowbot_imu 57600
