#!/usr/bin/env bash
# PC -> pilotage clavier du robot (publie /cmd_vel, recu par le Jetson).
# Prerequis : l'agent tourne sur le Jetson (./robot_agent.sh).
#
# Touches : i=avant  ,=arriere  j=gauche  l=droite  k=stop  q/z=vitesse +/-
#
# Usage : ./robot_teleop.sh

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

if ros2 pkg prefix teleop_twist_keyboard >/dev/null 2>&1; then
    exec ros2 run teleop_twist_keyboard teleop_twist_keyboard
else
    echo "teleop_twist_keyboard non installe."
    echo "Installer : sudo apt install ros-jazzy-teleop-twist-keyboard"
    echo
    echo "En attendant, pilotage direct (avance 0.3 m/s, Ctrl+C pour stop) :"
    exec ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
fi
