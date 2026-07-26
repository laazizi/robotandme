import math, time
import numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

FP=[(0.22,0.10),(0.08,0.26),(-0.08,0.26),(-0.20,0.14),(-0.27,0.08),
    (-0.27,-0.08),(-0.20,-0.14),(-0.08,-0.26),(0.08,-0.26),(0.22,-0.10)]

class C(Node):
    def __init__(s):
        super().__init__('fpcost'); s.g=None
        s.create_subscription(OccupancyGrid,'/local_costmap/costmap',s.cb,10)
        s.buf=Buffer(); TransformListener(s.buf,s)
    def cb(s,m): s.g=m
rclpy.init(); n=C()
t0=time.time()
while n.g is None and time.time()-t0<20: rclpy.spin_once(n,timeout_sec=0.2)
if n.g is None: print("pas de costmap locale"); raise SystemExit
g=n.g; W,H,res=g.info.width,g.info.height,g.info.resolution
ox,oy=g.info.origin.position.x,g.info.origin.position.y
data=np.array(g.data,dtype=np.int16).reshape(H,W)

pos=None; t0=time.time()
while time.time()-t0<10:
    rclpy.spin_once(n,timeout_sec=0.2)
    try:
        tr=n.buf.lookup_transform(g.header.frame_id,'base_link',rclpy.time.Time())
        q=tr.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        pos=(tr.transform.translation.x,tr.transform.translation.y,yaw); break
    except Exception: pass
if pos is None: print("pas de TF"); raise SystemExit
x,y,th=pos
print(f"repere costmap : {g.header.frame_id}   robot x={x:.2f} y={y:.2f} cap={math.degrees(th):.0f}deg\n")

def cost_at(px,py):
    i=int((px-ox)/res); j=int((py-oy)/res)
    if 0<=i<W and 0<=j<H: return int(data[j,i])
    return -99
def scan_footprint(dx):
    """cout max sur le contour, robot avance de dx metres"""
    worst=-1; where=None
    for k in range(len(FP)):
        ax,ay=FP[k]; bx,by=FP[(k+1)%len(FP)]
        for t in np.linspace(0,1,12):
            lx=ax+(bx-ax)*t+dx; ly=ay+(by-ay)*t
            px=x+lx*math.cos(th)-ly*math.sin(th)
            py=y+lx*math.sin(th)+ly*math.cos(th)
            c=cost_at(px,py)
            if c>worst: worst,where=c,(round(lx,2),round(ly,2))
    return worst,where

print(f"{'projection':>12} | {'cout max':>8} | point du contour")
print("-"*52)
for dx in (0.0,0.03,0.06,0.10):
    c,w=scan_footprint(dx)
    tag = "LIBRE" if c<50 else ("proche" if c<99 else "BLOQUANT")
    print(f"{dx*100:9.0f} cm | {c:8d} | {w}   {tag}")
print("-"*52)
print("echelle : 100=obstacle, 99=le robot toucherait, <50=libre, -1=inconnu")
occ=(data>=99).sum(); tot=W*H
print(f"cellules bloquantes dans la costmap locale : {occ} / {tot} ({100*occ/tot:.1f} %)")
