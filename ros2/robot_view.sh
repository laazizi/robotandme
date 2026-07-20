#!/usr/bin/env bash
# PC -> visualisation de l'odometrie dans RViz.
# La TF odom->base_link est fournie par l'EKF (service mowbot-ekf sur le Jetson),
# donc on ne lance PLUS de broadcaster ici (sinon conflit de TF).
# Prerequis : agent + IMU + EKF actifs sur le Jetson (services systemd).
#
# Usage : ./robot_view.sh

set -euo pipefail
cd "$(dirname "$0")"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

echo ">> RViz (TF fournie par l'EKF du Jetson)..."
ros2 run rviz2 rviz2 -d odom.rviz
