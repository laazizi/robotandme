#!/bin/bash
# Publie le modele URDF (/robot_description + TF roues). Utilise par mowbot-description.service.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
exec ros2 launch /home/nvidia/description.launch.py
