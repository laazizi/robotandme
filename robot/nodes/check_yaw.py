#!/usr/bin/env python3
"""Verifie la TF du lidar : ou l'echo le plus proche tombe-t-il dans base_link ?"""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

rclpy.init(); n = Node('check_yaw'); sc = []
n.create_subscription(LaserScan, '/scan', lambda m: sc.append(m), qos_profile_sensor_data)
buf = Buffer(); TransformListener(buf, n)
t0 = time.time()
while len(sc) < 5 and time.time() - t0 < 15:
    rclpy.spin_once(n, timeout_sec=0.2)
if not sc:
    print("  pas de scan"); raise SystemExit(1)

# la TF est statique : on lit sa translation et sa rotation
t0 = time.time(); tf = None
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
    try:
        tf = buf.lookup_transform('base_link', 'laser_link', rclpy.time.Time()); break
    except Exception: pass
if tf is None:
    print("  TF base_link->laser_link absente"); raise SystemExit(1)
q = tf.transform.rotation
yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
tx, ty = tf.transform.translation.x, tf.transform.translation.y
print(f"  TF lue : x={tx:+.3f} y={ty:+.3f} yaw={math.degrees(yaw):+.1f} deg")

m = sc[-1]
r = np.array(m.ranges, dtype=float)
a = m.angle_min + np.arange(len(r)) * m.angle_increment
ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
if not ok.any():
    print("  aucun echo"); raise SystemExit(1)
i = int(np.argmin(np.where(ok, r, np.inf)))
# composition manuelle : rotation du yaw puis translation
lx, ly = r[i]*math.cos(a[i]), r[i]*math.sin(a[i])
bx = tx + lx*math.cos(yaw) - ly*math.sin(yaw)
by = ty + lx*math.sin(yaw) + ly*math.cos(yaw)
ang = math.degrees(math.atan2(by, bx))
print(f"  echo le plus proche : {r[i]:.2f} m a {math.degrees(a[i]):+.1f} deg (repere lidar)")
print(f"  soit dans base_link : x={bx:+.3f} y={by:+.3f} m, angle {ang:+.1f} deg")
print()
if bx > 0 and abs(ang) < 30:
    print("  >>> DEVANT le robot : la correction de yaw est BONNE")
elif bx < 0:
    print("  >>> encore DERRIERE : le yaw n'est pas applique")
else:
    print(f"  >>> sur le cote ({ang:+.0f} deg) : rotation intermediaire")
