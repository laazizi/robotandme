import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

class C(Node):
    def __init__(s):
        super().__init__('pathcheck')
        s.grid=None
        # QoS TRANSIENT_LOCAL : un costmap se publie comme un topic LATCHE.
        # Avec la QoS par defaut (VOLATILE) et always_send_full_costmap: False,
        # la grille complete n'est envoyee qu'une fois au demarrage de nav2 : un
        # abonne qui arrive apres ne recoit RIEN, et cet outil affichait
        # "pas de costmap globale" alors que nav2 tournait parfaitement.
        s.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', s.cb,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        s.buf=Buffer(); TransformListener(s.buf,s)
        s.ac=ActionClient(s,ComputePathToPose,'compute_path_to_pose')
    def cb(s,m): s.grid=m

rclpy.init(); n=C()
t0=time.time()
while (n.grid is None) and time.time()-t0<20: rclpy.spin_once(n,timeout_sec=0.2)
if n.grid is None: print("pas de costmap globale"); raise SystemExit
g=n.grid
W,H,res=g.info.width,g.info.height,g.info.resolution
ox,oy=g.info.origin.position.x,g.info.origin.position.y
data=np.array(g.data,dtype=np.int16).reshape(H,W)
# nav2 republie la costmap en OccupancyGrid 0-100 : 100 = obstacle,
# 99 = gabarit du robot dessus. (Le 0-255 est sur costmap_raw.)
ys,xs=np.nonzero(data==100)   # 100 = OBSTACLE reel (99 = zone ou le robot toucherait)
print(f"costmap {W}x{H} @ {res:.3f} m   cellules OBSTACLE : {len(xs)}")
if len(xs)==0: print("aucun obstacle dans la carte -> test non significatif"); raise SystemExit
obs=np.stack([ox+(xs+0.5)*res, oy+(ys+0.5)*res],axis=1)

# position courante
t0=time.time(); pos=None
while time.time()-t0<10:
    rclpy.spin_once(n,timeout_sec=0.2)
    try:
        tr=n.buf.lookup_transform('map','base_link',rclpy.time.Time())
        pos=(tr.transform.translation.x,tr.transform.translation.y); break
    except Exception: pass
if pos is None: print("pas de TF map->base_link"); raise SystemExit
print(f"robot en x={pos[0]:.2f} y={pos[1]:.2f}\n")

n.ac.wait_for_server(timeout_sec=15)
print(f"{'but':>18} | {'long.':>6} | {'ecart min':>9} | verdict")
print("-"*58)
worst=99
for ang in range(0,360,45):
    a=math.radians(ang); D=1.6
    p=PoseStamped(); p.header.frame_id='map'
    p.pose.position.x=pos[0]+D*math.cos(a); p.pose.position.y=pos[1]+D*math.sin(a)
    p.pose.orientation.w=1.0
    goal=ComputePathToPose.Goal(); goal.goal=p; goal.use_start=False
    fut=n.ac.send_goal_async(goal); rclpy.spin_until_future_complete(n,fut,timeout_sec=15)
    gh=fut.result()
    if gh is None or not gh.accepted: print(f"{ang:>15}deg | refuse"); continue
    rf=gh.get_result_async(); rclpy.spin_until_future_complete(n,rf,timeout_sec=20)
    res_=rf.result()
    if res_ is None: print(f"{ang:>15}deg | pas de resultat"); continue
    pts=[(q.pose.position.x,q.pose.position.y) for q in res_.result.path.poses]
    if len(pts)<2: print(f"{ang:>15}deg | pas de chemin"); continue
    P=np.array(pts)
    d=np.sqrt(((P[:,None,:]-obs[None,:,:])**2).sum(-1)).min(axis=1)
    # On ignore les 40 premiers cm : le chemin part de la position ACTUELLE du
    # robot, qui peut deja etre collee a un obstacle -> ce n'est pas un choix
    # du planificateur, et ca ecraserait le minimum.
    step=np.r_[0,np.sqrt(((P[1:]-P[:-1])**2).sum(-1)).cumsum()]
    keep=step>0.40
    mn=d[keep].min() if keep.any() else d.min()
    L=np.sqrt(((P[1:]-P[:-1])**2).sum(-1)).sum()
    worst=min(worst,mn)
    verdict = "OK" if mn>=0.40 else ("juste" if mn>=0.30 else "RASE")
    print(f"{ang:>15}deg | {L:5.2f}m | {mn:8.2f}m | {verdict}")
print("-"*58)
print(f"pire distance chemin<->obstacle : {worst:.2f} m")
print(f"demi-largeur du robot : 0.27 m  -> marge libre restante : {worst-0.27:+.2f} m")
