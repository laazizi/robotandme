"""Compare /scan et /scan_raw : nombre de points, secteur, frequence."""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
D = 180 / math.pi
rclpy.init(); n = Node("v"); d = {}; c = [0, 0]
def cb_s(m):
    d["s"] = m; c[0] += 1
def cb_r(m):
    d["r"] = m; c[1] += 1
n.create_subscription(LaserScan, "/scan", cb_s, qos_profile_sensor_data)
n.create_subscription(LaserScan, "/scan_raw", cb_r, qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.2)
r, s = d.get("r"), d.get("s")
if r:
    print(f"  /scan_raw : {len(r.ranges):3d} pts  {r.angle_min*D:+7.1f}..{r.angle_max*D:+7.1f} deg  {c[1]/12:.2f} Hz")
else:
    print("  /scan_raw : RIEN")
if s:
    v = sum(1 for x in s.ranges if 0 < x < s.range_max)
    print(f"  /scan     : {len(s.ranges):3d} pts  {s.angle_min*D:+7.1f}..{s.angle_max*D:+7.1f} deg  {c[0]/12:.2f} Hz  valides {v}")
else:
    print("  /scan     : RIEN")
