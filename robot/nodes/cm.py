"""Que contient la costmap AUTOUR et SOUS le robot ?"""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node("cm"); buf = Buffer(); TransformListener(buf, n)
g = {}
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
n.create_subscription(OccupancyGrid, "/local_costmap/costmap", lambda m: g.__setitem__("l", m), q)
n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: g.__setitem__("g", m), q)
n.create_subscription(OccupancyGrid, "/map", lambda m: g.__setitem__("m", m), q)
t0 = time.time()
while time.time() - t0 < 15 and len(g) < 3:
    rclpy.spin_once(n, timeout_sec=0.2)
# pose du robot
pose = None
for _ in range(50):
    try:
        t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
        pose = (t.transform.translation.x, t.transform.translation.y); break
    except Exception:
        rclpy.spin_once(n, timeout_sec=0.2)
for k, nom in (("m", "/map (SLAM)"), ("g", "global_costmap"), ("l", "local_costmap")):
    m = g.get(k)
    if m is None:
        print(f"  {nom:16s} ABSENTE"); continue
    d = np.asarray(m.data, dtype=np.int16)
    tot = len(d)
    print(f"  {nom:16s} {m.info.width}x{m.info.height} a {m.info.resolution:.3f} m/px")
    print(f"      inconnu(-1) {100*np.sum(d<0)/tot:5.1f} %   libre(0) {100*np.sum(d==0)/tot:5.1f} %"
          f"   letal(>=99) {100*np.sum(d>=99)/tot:5.1f} %   intermediaire {100*np.sum((d>0)&(d<99))/tot:5.1f} %")
    # zone de 60 cm autour du robot
    if pose and k in ("g", "l"):
        res = m.info.resolution
        cx = int((pose[0]-m.info.origin.position.x)/res)
        cy = int((pose[1]-m.info.origin.position.y)/res)
        r = int(0.30/res)
        gr = d.reshape(m.info.height, m.info.width)
        y0,y1 = max(0,cy-r), min(m.info.height, cy+r+1)
        x0,x1 = max(0,cx-r), min(m.info.width, cx+r+1)
        z = gr[y0:y1, x0:x1]
        if z.size:
            print(f"      dans 30 cm autour du robot : inconnu {100*np.sum(z<0)/z.size:.0f} %"
                  f"  letal {100*np.sum(z>=99)/z.size:.0f} %  libre {100*np.sum(z==0)/z.size:.0f} %")
