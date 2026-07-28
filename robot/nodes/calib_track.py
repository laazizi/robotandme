#!/usr/bin/env python3
"""Calibre TRACK_WIDTH_M sans se laisser piéger par le deficit de vitesse.

PIEGE de la methode naive : comparer la rotation COMMANDEE a la rotation
mesuree melange deux causes -- un entraxe faux, et des roues qui n'atteignent
pas leur consigne. Sur ce robot les roues ne font que ~90 % de la consigne :
la rotation aussi, meme avec un entraxe parfait.

METHODE JUSTE : on compare la rotation mesuree par le GYRO a celle que les
vitesses de roue REELLES impliquent :
        w_geometrique = (v_droite - v_gauche) / L
Les vitesses de roue viennent des ENCODEURS, donc du reel, pas de la consigne.
    L_correct = (v_droite - v_gauche) / w_gyro
Le deficit de vitesse s'annule : il affecte les deux membres.

Note : reconstruire v_gauche/v_droite depuis vx et wz de /odom avec le MEME L
que le firmware redonne exactement les vitesses mesurees (la transformation
est inversible), donc aucune circularite.
"""
import math, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

L_ACTUEL = float(os.environ.get('MOWBOT_TRACK', '0.59'))
W_CMD    = float(os.environ.get('MOWBOT_W', '0.5'))
DUREE    = float(os.environ.get('MOWBOT_DUR', '5.0'))

class C(Node):
    def __init__(s):
        super().__init__('calib_track2')
        s.pub = s.create_publisher(Twist, '/cmd_vel', 10)
        s.gz = None; s.samples = []; s.collect = False
        s.create_subscription(Imu, '/imu/data_raw', s.cb_imu, qos_profile_sensor_data)
        s.create_subscription(Odometry, '/odom', s.cb_odom, 10)
        s.gyro_hist = []
    def cb_imu(s, m):
        s.gz = m.angular_velocity.z
        if s.collect: s.gyro_hist.append(s.gz)
    def cb_odom(s, m):
        if s.collect:
            vx, wz = m.twist.twist.linear.x, m.twist.twist.angular.z
            s.samples.append((vx - wz*L_ACTUEL/2, vx + wz*L_ACTUEL/2))

rclpy.init(); n = C()
t0 = time.time()
while n.gz is None and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
if n.gz is None:
    print("pas d'IMU"); sys.exit(1)

ech = []
t0 = time.time()
while time.time() - t0 < 3:
    rclpy.spin_once(n, timeout_sec=0.1)
    if n.gz is not None: ech.append(n.gz)
biais = sum(ech)/len(ech)
print(f"biais gyro : {biais:+.4f} rad/s\n")

tw = Twist(); tw.angular.z = W_CMD
n.collect = True
t0 = time.time()
while time.time() - t0 < DUREE:
    n.pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.02); time.sleep(0.03)
for _ in range(20):
    n.pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02); time.sleep(0.03)
n.collect = False

if len(n.samples) < 5 or len(n.gyro_hist) < 5:
    print("pas assez de mesures"); sys.exit(1)

# on ignore le premier tiers : montee en vitesse
k = len(n.samples)//3
vg = sum(s[0] for s in n.samples[k:]) / len(n.samples[k:])
vd = sum(s[1] for s in n.samples[k:]) / len(n.samples[k:])
kg = len(n.gyro_hist)//3
w_gyro = sum(n.gyro_hist[kg:]) / len(n.gyro_hist[kg:]) - biais

print(f"consigne de rotation      : {W_CMD:.3f} rad/s")
print(f"vitesses de roue MESUREES : gauche {vg:+.4f}  droite {vd:+.4f} m/s")
print(f"  -> consigne par roue    : {-W_CMD*L_ACTUEL/2:+.4f} / {W_CMD*L_ACTUEL/2:+.4f} m/s")
print(f"  -> les roues font {100*abs(vd)/(W_CMD*L_ACTUEL/2):.0f} % de leur consigne")
print()
print(f"rotation mesuree au GYRO  : {w_gyro:+.4f} rad/s ({math.degrees(w_gyro):+.1f} deg/s)")
w_geo = (vd - vg) / L_ACTUEL
print(f"rotation que la geometrie predit : {w_geo:+.4f} rad/s")
print()
if abs(w_gyro) < 1e-3:
    print("gyro trop faible"); sys.exit(1)
L_new = (vd - vg) / w_gyro
ratio = w_gyro / w_geo
print(f"TRACK_WIDTH_M actuel  : {L_ACTUEL:.4f} m")
print(f"TRACK_WIDTH_M corrige : {L_new:.4f} m   (ecart {100*(ratio-1):+.1f} %)")
if abs(ratio - 1) < 0.05:
    print("\n-> entraxe DEJA JUSTE (moins de 5 % d'ecart) : ne rien changer.")
    print("   Le deficit de rotation vient donc du suivi de vitesse, pas de la geometrie.")
else:
    sens = "PLUS" if ratio > 1 else "MOINS"
    print(f"\n-> le robot tourne {sens} que sa geometrie ne le predit : entraxe a corriger.")
