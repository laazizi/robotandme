import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
def yq(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
class T(Node):
    def __init__(s):
        super().__init__('yawref')
        s.buf=Buffer(); s.l=TransformListener(s.buf,s)
        s.p=s.create_publisher(Twist,'/cmd_vel',10)
    def get(s,a,b):
        try:
            t=s.buf.lookup_transform(a,b,rclpy.time.Time())
            return yq(t.transform.rotation)
        except Exception: return None
rclpy.init(); n=T()
t0=time.time()
while time.time()-t0<8: rclpy.spin_once(n,timeout_sec=0.2)
m0=n.get('map','base_link'); o0=n.get('odom','base_link')
if m0 is None: print("pas de TF map->base_link (SLAM inactif ?)")
if o0 is None: print("pas de TF odom->base_link"); raise SystemExit
W=0.20; DUR=6.0
tw=Twist(); tw.angular.z=W
t0=time.time()
while time.time()-t0<DUR:
    n.p.publish(tw); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.05)
for _ in range(20): n.p.publish(Twist()); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.05)
t0=time.time()
while time.time()-t0<4: rclpy.spin_once(n,timeout_sec=0.1)
m1=n.get('map','base_link'); o1=n.get('odom','base_link')
def d(a,b):
    if a is None or b is None: return None
    x=b-a
    while x>math.pi: x-=2*math.pi
    while x<-math.pi: x+=2*math.pi
    return x
att=W*DUR
dg=d(o0,o1); dl=d(m0,m1)
print(f"consigne        : {math.degrees(att):6.1f} deg")
print(f"GYRO (odom)     : {math.degrees(dg):6.1f} deg   ({100*dg/att:.0f} %)")
if dl is not None:
    print(f"LIDAR/SLAM (map): {math.degrees(dl):6.1f} deg   ({100*dl/att:.0f} %)")
    print(f"ecart gyro/lidar: {100*(dg/dl-1):+.0f} %")
