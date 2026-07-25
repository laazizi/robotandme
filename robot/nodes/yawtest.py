import rclpy, math, time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
class T(Node):
    def __init__(s):
        super().__init__('yawtest')
        s.y=None; s.y0=None
        s.create_subscription(Odometry,'/odometry/filtered',s.cb,10)
        s.p=s.create_publisher(Twist,'/cmd_vel',10)
    def cb(s,m): s.y=yaw(m.pose.pose.orientation)
rclpy.init(); n=T()
t0=time.time()
while n.y is None and time.time()-t0<10: rclpy.spin_once(n,timeout_sec=0.2)
if n.y is None: print("pas d'odometrie filtree"); raise SystemExit
y0=n.y
W=0.20; DUR=5.0            # consigne : 0.20 rad/s pendant 5 s => 1.00 rad attendu
tw=Twist(); tw.angular.z=W
t0=time.time()
while time.time()-t0<DUR:
    n.p.publish(tw); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.05)
n.p.publish(Twist())
for _ in range(20): n.p.publish(Twist()); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.05)
t0=time.time()
while time.time()-t0<2: rclpy.spin_once(n,timeout_sec=0.1)
d=n.y-y0
while d>math.pi: d-=2*math.pi
while d<-math.pi: d+=2*math.pi
att=W*DUR
print(f"consigne  = {att:.3f} rad ({math.degrees(att):.1f} deg)")
print(f"gyro reel = {d:.3f} rad ({math.degrees(d):.1f} deg)")
print(f"rendement = {100*d/att:.0f} %")
