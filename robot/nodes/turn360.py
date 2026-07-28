#!/usr/bin/env python3
"""Rotation sur place de 360 deg (ou N tours), asservie sur l'odometrie.

Le robot s'arrete quand SON odometrie annonce le tour complet. Tu compares
alors a la realite : il doit avoir retrouve exactement son orientation de
depart. L'ecart visible donne l'erreur d'entraxe.

  ecart > 0 (a tourne trop)   -> TRACK_WIDTH_M configure trop GRAND
  ecart < 0 (pas assez)       -> configure trop PETIT
  TRACK_WIDTH_M corrige = actuel x (360 / degres_reels)

Le GYRO est lu en parallele comme reference independante : si odometrie et gyro
concordent mais que le robot ne revient pas a sa marque, l'erreur est commune
aux deux (donc geometrique) ; s'ils divergent, c'est l'entraxe.

    python3 turn360.py [nb_tours]     defaut 1
Plusieurs tours amplifient l'erreur et la rendent mesurable a l'oeil :
1 % d'erreur = 3.6 deg sur un tour, mais 11 deg sur trois.
"""
import math, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

TOURS = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
W     = float(os.environ.get('MOWBOT_W', '0.5'))
L     = float(os.environ.get('MOWBOT_TRACK', '0.465'))

def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class T(Node):
    def __init__(s):
        super().__init__('turn360')
        s.pub = s.create_publisher(Twist, '/cmd_vel', 10)
        s.yaw = None; s.gz = 0.0; s.gyro = 0.0; s.t_prev = None; s.biais = 0.0
        s.create_subscription(Odometry, '/odom', s.cb_o, 10)
        s.create_subscription(Imu, '/imu/data_raw', s.cb_i, qos_profile_sensor_data)
    def cb_o(s, m): s.yaw = yaw_of(m.pose.pose.orientation)
    def cb_i(s, m):
        now = time.time(); s.gz = m.angular_velocity.z
        if s.t_prev is not None: s.gyro += (s.gz - s.biais) * (now - s.t_prev)
        s.t_prev = now

rclpy.init(); n = T()
t0 = time.time()
while n.yaw is None and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
if n.yaw is None:
    print("pas d'odometrie"); sys.exit(1)

ech = []
t0 = time.time()
while time.time() - t0 < 2.5:
    rclpy.spin_once(n, timeout_sec=0.1); ech.append(n.gz)
n.biais = sum(ech)/len(ech)
print(f"biais gyro : {n.biais:+.4f} rad/s")
print(f"entraxe configure : {L} m\n")

CIBLE = TOURS * 2 * math.pi
print(f">>> ROTATION de {TOURS:g} tour(s) = {math.degrees(CIBLE):.0f} deg, "
      f"a {W} rad/s (~{CIBLE/W:.0f} s)")
print("    REPERE bien l'orientation de depart\n")

n.gyro = 0.0; n.t_prev = None
cumul = 0.0; prev = n.yaw
FREIN = math.radians(25)
t0 = time.time()
while cumul < CIBLE:
    reste = CIBLE - cumul
    w = W if reste > FREIN else max(0.12, W * reste / FREIN)
    tw = Twist(); tw.angular.z = w
    n.pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.02); time.sleep(0.03)
    cur = n.yaw
    cumul += abs(wrap(cur - prev))      # increments courts : jamais ambigus
    prev = cur
    if time.time() - t0 > CIBLE/W * 4:  # garde-fou
        print("garde-fou : arret"); break
for _ in range(25):
    n.pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02); time.sleep(0.03)
t0 = time.time()
while time.time() - t0 < 1.5: rclpy.spin_once(n, timeout_sec=0.1)

print(f"odometrie : {math.degrees(cumul):7.1f} deg")
print(f"gyro      : {math.degrees(abs(n.gyro)):7.1f} deg")
ecart = math.degrees(abs(n.gyro) - cumul)
print(f"ecart odometrie/gyro : {ecart:+.1f} deg")
# Seuil a 1 % : c'est la repetabilite du gyro. A 2 % on laissait passer 21 deg
# d'ecart sur 3 tours en annoncant "d'accord", ce qui masquait le probleme.
seuil = 0.01 * math.degrees(CIBLE)
if abs(ecart) < seuil:
    print(f"  -> capteurs d'accord (moins de {seuil:.0f} deg, soit 1 %)\n")
else:
    print(f"  -> DIVERGENCE de {100*abs(ecart)/math.degrees(CIBLE):.1f} % : "
          f"entraxe ou echelle du gyro a revoir\n")
print(">>> MESURE l'ecart REEL a l'orientation de depart, puis :")
print(f"    TRACK_WIDTH_M corrige = {L} x ({math.degrees(CIBLE):.0f} / degres_reels)")
print(f"    exemple : s'il a tourne {math.degrees(CIBLE)+10:.0f} deg -> "
      f"{L * math.degrees(CIBLE)/(math.degrees(CIBLE)+10):.4f} m")
