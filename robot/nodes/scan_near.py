#!/usr/bin/env python3
"""Liste les echos PROCHES (<1 m) par secteur de 15 deg, sur /scan (filtre).
Robot immobile : tout ce qui apparait a moins de ~50 cm est suspect
(structure du robot mal masquee) et bloque nav2."""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

class N(Node):
    def __init__(self):
        super().__init__('scan_near')
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, '/scan', self.cb, q)
        self.buf = []; self.m = None
    def cb(self, m):
        self.m = m; self.buf.append(np.array(m.ranges, dtype=float))

rclpy.init(); n = N()
t0 = time.time()
while time.time() - t0 < 5 and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.1)
if not n.buf:
    print("aucun scan"); raise SystemExit
a = np.vstack(n.buf); a[~np.isfinite(a)] = 99.0
r = a.min(axis=0)
ang = (np.degrees(np.linspace(n.m.angle_min, n.m.angle_max, a.shape[1]))) % 360
print(f"{a.shape[0]} scans analyses\n")
print("secteur      echo le + proche")
for s in range(0, 360, 15):
    sel = (ang >= s) & (ang < s + 15)
    if not sel.any():
        continue
    d = r[sel].min()
    if d < 1.0:
        bar = "#" * max(1, int((1.0 - d) * 30))
        flag = "  <<< SUSPECT (structure robot ?)" if d < 0.5 else ""
        print(f"  {s:3d}-{s+15:3d} deg  {d*100:5.1f} cm  {bar}{flag}")
