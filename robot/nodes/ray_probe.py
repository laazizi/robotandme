import math, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
class S(Node):
    def __init__(s):
        super().__init__('ray'); s.m=[]
        s.create_subscription(LaserScan,'/scan',s.cb,qos_profile_sensor_data)
        s.create_subscription(LaserScan,'/scan_raw',s.cbr,qos_profile_sensor_data)
        s.raw=[]
    def cb(s,x): s.m.append(x)
    def cbr(s,x): s.raw.append(x)
rclpy.init(); n=S()
t0=time.time()
while len(n.m)<20 and time.time()-t0<25: rclpy.spin_once(n,timeout_sec=0.2)
if not n.m: print("pas de /scan"); raise SystemExit

def show(msgs,label):
    m=msgs[-1]; N=len(m.ranges)
    A=np.array([[x if np.isfinite(x) else np.nan for x in mm.ranges] for mm in msgs[-15:]])
    print(f"\n--- {label} ({N} pts) ---")
    print(f"{'angle':>7} | {'dist moy':>8} | {'ecart-type':>10} | stabilite")
    # le point bloquant vu depuis base_link : ~15 cm devant, 26 cm a gauche.
    # Le lidar est 10 cm en avant de base_link -> depuis le LIDAR : (0.05,0.26)
    for deg in (60,70,75,79,85,90,100):
        a=math.radians(deg)
        i=int(round((a-m.angle_min)/ (m.angle_max-m.angle_min) * (N-1))) % N
        col=A[:,i]
        mu=np.nanmean(col); sd=np.nanstd(col)
        nan=np.isnan(col).sum()
        if np.isnan(mu):
            print(f"{deg:6}deg |    (vide) |          - | rien vu")
        else:
            tag = "CONSTANTE -> partie du robot" if sd<0.02 else ("variable" if sd<0.10 else "tres variable")
            print(f"{deg:6}deg | {mu:7.3f}m | {sd:9.3f}m | {tag} ({nan}/15 vides)")
show(n.m,"/scan (apres filtrage)")
if n.raw: show(n.raw,"/scan_raw (brut du lidar)")
