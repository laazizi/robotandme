#!/bin/bash
# Mesure la vitesse reellement atteinte pour une consigne donnee.
# Usage : speed_test.sh [consigne]   (defaut 0.048 = croisiere nav2)
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
V="${1:-0.048}"
X0=$(timeout 4 ros2 topic echo /odom --once --field pose.pose.position.x 2>/dev/null | head -1)
Y0=$(timeout 4 ros2 topic echo /odom --once --field pose.pose.position.y 2>/dev/null | head -1)
echo ">> consigne $V m/s pendant 8 s"
timeout 8 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $V}}" -r 20 >/dev/null 2>&1
sleep 1
X1=$(timeout 4 ros2 topic echo /odom --once --field pose.pose.position.x 2>/dev/null | head -1)
Y1=$(timeout 4 ros2 topic echo /odom --once --field pose.pose.position.y 2>/dev/null | head -1)
python3 - "$X0" "$Y0" "$X1" "$Y1" "$V" <<'PY'
import sys, math
x0, y0, x1, y1, v = [float(a) for a in sys.argv[1:6]]
d = math.hypot(x1 - x0, y1 - y0)
print(f"  distance : {d*100:.1f} cm en 8 s")
print(f"  vitesse  : {d/8:.3f} m/s   (consigne {v})")
print(f"  rendement: {100*d/8/v:.0f} %")
PY
