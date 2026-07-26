import rclpy, math, time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
class S(Node):
    def __init__(s):
        super().__init__('scan_noise'); s.msgs=[]
        s.create_subscription(LaserScan,'/scan',s.cb,qos_profile_sensor_data)
    def cb(s,m): s.msgs.append(m)
rclpy.init(); n=S()
t0=time.time()
while len(n.msgs)<25 and time.time()-t0<25: rclpy.spin_once(n,timeout_sec=0.2)
if not n.msgs: print("pas de /scan"); raise SystemExit
m=n.msgs[-1]
r=np.array(m.ranges,dtype=float)
valid=np.isfinite(r)&(r>m.range_min)&(r<m.range_max)
print(f"points par tour : {len(r)}   valides : {valid.sum()}")
# un point REEL appartient a une surface -> ses voisins sont a une distance
# comparable. Un point aberrant est isole en profondeur.
idx=np.where(valid)[0]
iso=0; SEUIL=0.15
for i in idx:
    a,b=(i-1)%len(r),(i+1)%len(r)
    na = valid[a] and abs(r[a]-r[i])<SEUIL
    nb = valid[b] and abs(r[b]-r[i])<SEUIL
    if not na and not nb: iso+=1
print(f"points ISOLES (aucun voisin a moins de {SEUIL} m) : {iso}  soit {100*iso/max(1,valid.sum()):.1f} %")
# stabilite : un vrai obstacle est vu a la meme distance d'un tour a l'autre
A=np.array([[x if np.isfinite(x) else np.nan for x in mm.ranges] for mm in n.msgs[-15:]])
with np.errstate(all='ignore'):
    std=np.nanstd(A,axis=0)
inst=np.nansum(std>0.10)
print(f"directions INSTABLES d'un tour a l'autre (ecart-type > 10 cm) : {int(inst)}")
