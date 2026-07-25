#!/usr/bin/env bash
# PC -> visualisation RViz du robot (disque + 2 roues) et de l'odometrie fusionnee.
# Tout vient du Jetson : /robot_description (mowbot-description), TF odom->base_link
# (mowbot-ekf), base_link->imu_link (mowbot-razor), /odometry/filtered (EKF).
# Prerequis : services mowbot-* actifs sur le Jetson.
#
# Usage : ./robot_view.sh

set -eo pipefail                 # pas de -u : ROS setup.bash n'est pas nounset-clean
cd "$(dirname "$0")"

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

echo ">> RViz (modele + odometrie depuis le Jetson)..."
exec ros2 run rviz2 rviz2 -d odom.rviz
