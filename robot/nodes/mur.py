"""Ou le scan place-t-il l'obstacle le plus proche, dans le repere du ROBOT ?"""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
D = 180 / math.pi
rclpy.init(); n = Node("mur"); buf = Buffer(); TransformListener(buf, n)
scans = []
n.create_subscription(LaserScan, "/scan", lambda m: scans.append(m),
                      qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.2)
if not scans:
    print("  aucun scan"); raise SystemExit
# TF laser -> base, telle qu'elle est REELLEMENT publiee
tf = None
for _ in range(50):
    try:
        tf = buf.lookup_transform("base_link", "laser_link", rclpy.time.Time()); break
    except Exception:
        rclpy.spin_once(n, timeout_sec=0.2)
if tf is None:
    print("  TF base_link<-laser_link absente"); raise SystemExit
q = tf.transform.rotation
lyaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
lx, ly = tf.transform.translation.x, tf.transform.translation.y
print(f"  TF utilisee : lidar a x={lx:+.3f} y={ly:+.3f} cap={lyaw*D:+.0f} deg")
print()
m = scans[-1]
r = np.asarray(m.ranges, dtype=float)
a = m.angle_min + np.arange(len(r)) * m.angle_increment
ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
r, a = r[ok], a[ok]
# dans base_link
px = lx + r * np.cos(a + lyaw)
py = ly + r * np.sin(a + lyaw)
d = np.hypot(px, py)
i = int(np.argmin(d))
ang = math.atan2(py[i], px[i]) * D
print(f"  obstacle le PLUS PROCHE du centre du robot :")
print(f"    distance {d[i]:.3f} m   direction {ang:+.0f} deg")
print(f"    x={px[i]:+.3f} m   y={py[i]:+.3f} m")
sect = ("DEVANT" if abs(ang) < 45 else "DERRIERE" if abs(ang) > 135
        else ("GAUCHE" if ang > 0 else "DROITE"))
print(f"    => {sect}")
print()
# repartition par quadrant : distance minimale dans chaque direction
print("  distance minimale par secteur (repere ROBOT) :")
for nom, lo, hi in (("DEVANT      (-30..+30)", -30, 30),
                    ("GAUCHE      (+60..+120)", 60, 120),
                    ("DERRIERE   (150..-150)", 150, -150),
                    ("DROITE     (-120..-60)", -120, -60)):
    aa = np.arctan2(py, px) * D
    sel = ((aa >= lo) & (aa <= hi)) if lo < hi else ((aa >= lo) | (aa <= hi))
    if sel.any():
        print(f"    {nom} : {d[sel].min():.3f} m")
    else:
        print(f"    {nom} : rien")
