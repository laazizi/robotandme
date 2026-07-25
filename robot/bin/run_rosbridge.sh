#!/bin/bash
# rosbridge WebSocket 9090 : joystick navigateur et apps mobiles.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
