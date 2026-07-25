#!/bin/bash
# IMU Razor -> /imu/data_raw. TF imu_link : voir le service mowbot-tf.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
exec python3 -u /home/nvidia/razor_imu_node.py /dev/mowbot_imu 57600
