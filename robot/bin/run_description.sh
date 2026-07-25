#!/bin/bash
# robot_state_publisher : modele URDF + TF des roues.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
exec ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat "$MOWBOT_CONFIG/mowbot.urdf")"
