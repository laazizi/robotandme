import math, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener

class G(Node):
    def __init__(s):
        super().__init__('trygoal')
        s.buf=Buffer(); TransformListener(s.buf,s)
        s.ac=ActionClient(s,NavigateToPose,'navigate_to_pose')
        s.cmds=[]
        s.create_subscription(Twist,'/cmd_vel',lambda m:s.cmds.append((m.linear.x,m.angular.z)),10)
    def pose(s):
        try:
            t=s.buf.lookup_transform('map','base_link',rclpy.time.Time())
            q=t.transform.rotation
            return (t.transform.translation.x,t.transform.translation.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        except Exception: return None

rclpy.init(); n=G()
t0=time.time(); p=None
while time.time()-t0<12:
    rclpy.spin_once(n,timeout_sec=0.2); p=n.pose()
    if p: break
if not p: print("pas de pose"); raise SystemExit
x0,y0,th=p
print(f"depart  x={x0:.2f} y={y0:.2f} cap={math.degrees(th):.0f}deg")

D=1.0
g=PoseStamped(); g.header.frame_id='map'
g.pose.position.x=x0+D*math.cos(th); g.pose.position.y=y0+D*math.sin(th)
g.pose.orientation.w=1.0
print(f"but     x={g.pose.position.x:.2f} y={g.pose.position.y:.2f}  (1 m droit devant)\n")

n.ac.wait_for_server(timeout_sec=15)
fut=n.ac.send_goal_async(NavigateToPose.Goal(pose=g))
rclpy.spin_until_future_complete(n,fut,timeout_sec=15)
gh=fut.result()
if gh is None or not gh.accepted: print("but REFUSE"); raise SystemExit
print("but accepte, observation 35 s...\n")
rf=gh.get_result_async()
t0=time.time(); last=0
while time.time()-t0<35:
    rclpy.spin_once(n,timeout_sec=0.2)
    e=int(time.time()-t0)
    if e>=last+7:
        last=e; q=n.pose()
        if q:
            d=math.hypot(q[0]-x0,q[1]-y0)
            nz=[c for c in n.cmds if abs(c[0])>0.001 or abs(c[1])>0.001]
            rev=[c for c in nz if c[0]<-0.001]
            print(f"  t={e:2d}s  parcouru {d*100:5.1f} cm   cmd_vel non nulles {len(nz):4d}  (dont {len(rev)} en marche arriere)")
    if rf.done(): break
q=n.pose()
print()
if q:
    print(f"deplacement total : {math.hypot(q[0]-x0,q[1]-y0)*100:.1f} cm")
nz=[c for c in n.cmds if abs(c[0])>0.001 or abs(c[1])>0.001]
if nz:
    print(f"vitesse lin. max commandee : {max(abs(c[0]) for c in nz):.3f} m/s")
    print(f"rotation max commandee     : {max(abs(c[1]) for c in nz):.3f} rad/s")
    print(f"commandes en marche arriere: {sum(1 for c in nz if c[0]<-0.001)}")
else:
    print("AUCUNE commande de mouvement emise")
