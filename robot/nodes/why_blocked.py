import math, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

FP=[(0.22,0.10),(0.08,0.26),(-0.08,0.26),(-0.20,0.14),(-0.27,0.08),
    (-0.27,-0.08),(-0.20,-0.14),(-0.08,-0.26),(0.08,-0.26),(0.22,-0.10)]
class C(Node):
    def __init__(s):
        super().__init__('why'); s.g=None; s.scan=None
        s.create_subscription(OccupancyGrid,'/local_costmap/costmap',s.cb,10)
        s.create_subscription(LaserScan,'/scan',s.cs,qos_profile_sensor_data)
        s.buf=Buffer(); TransformListener(s.buf,s)
    def cb(s,m): s.g=m
    def cs(s,m): s.scan=m
rclpy.init(); n=C()
t0=time.time()
while (n.g is None or n.scan is None) and time.time()-t0<20: rclpy.spin_once(n,timeout_sec=0.2)
if n.g is None: print("pas de costmap"); raise SystemExit
g=n.g; W,H,res=g.info.width,g.info.height,g.info.resolution
ox,oy=g.info.origin.position.x,g.info.origin.position.y
d=np.array(g.data,dtype=np.int16).reshape(H,W)
pos=None; t0=time.time()
while time.time()-t0<10:
    rclpy.spin_once(n,timeout_sec=0.2)
    try:
        t=n.buf.lookup_transform(g.header.frame_id,'base_link',rclpy.time.Time())
        q=t.transform.rotation
        pos=(t.transform.translation.x,t.transform.translation.y,
             math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))); break
    except Exception: pass
if pos is None: print("pas de TF"); raise SystemExit
x,y,th=pos

def cost(px,py):
    i=int((px-ox)/res); j=int((py-oy)/res)
    return int(d[j,i]) if 0<=i<W and 0<=j<H else -99

print("Cellules a cout 100 (OBSTACLE REEL) touchant le contour du robot :")
hits=[]
for k in range(len(FP)):
    ax,ay=FP[k]; bx,by=FP[(k+1)%len(FP)]
    for tt in np.linspace(0,1,15):
        lx=ax+(bx-ax)*tt; ly=ay+(by-ay)*tt
        px=x+lx*math.cos(th)-ly*math.sin(th); py=y+lx*math.sin(th)+ly*math.cos(th)
        if cost(px,py)>=100: hits.append((round(lx,2),round(ly,2)))
if hits:
    for h in sorted(set(hits)): 
        ang=math.degrees(math.atan2(h[1],h[0])); dist=math.hypot(*h)
        print(f"   contour ({h[0]:+.2f},{h[1]:+.2f})  = {ang:+6.0f} deg, {dist:.2f} m du centre")
else:
    print("   AUCUNE -> le robot n'est PAS en collision a l'arret")

print("\nEchos du lidar a moins de 45 cm (repere base_link, lidar a x=+0.10) :")
m=n.scan; N=len(m.ranges)
close=[]
for i,r in enumerate(m.ranges):
    if not np.isfinite(r) or r<=0: continue
    a=m.angle_min+i*(m.angle_max-m.angle_min)/(N-1)
    bx=0.10+r*math.cos(a); by=r*math.sin(a)
    dc=math.hypot(bx,by)
    if dc<0.45: close.append((math.degrees(a)%360,r,round(bx,2),round(by,2),round(dc,2)))
if not close: print("   aucun")
for c in sorted(close)[:22]:
    print(f"   lidar {c[0]:6.1f} deg a {c[1]:.2f} m -> base_link ({c[2]:+.2f},{c[3]:+.2f}) dist centre {c[4]:.2f} m")
print(f"   ... {len(close)} echos proches au total")
