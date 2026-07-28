#!/usr/bin/env python3
"""Teste les moteurs : envoie une consigne et mesure ce que les ENCODEURS voient.

Mesurer l'odometrie plutot que se fier a la commande : c'est la seule facon de
distinguer "le moteur tourne" de "l'ordre a ete envoye". Chaque roue est
observee separement pour reperer un cote mort ou inverse.
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Entraxe utilise pour reconstruire les vitesses de roue depuis vx/wz.
# 0.59 = robot 12 V (bloc P4 de config.h) ; 0.38 = robot 24 V (DevKitC).
# Surchargeable : MOWBOT_TRACK=0.38 python3 motor_test.py
import os
WHEEL_R = 0.0698
TRACK   = float(os.environ.get('MOWBOT_TRACK', '0.59'))

class M(Node):
    def __init__(s):
        super().__init__('motor_test')
        s.pub = s.create_publisher(Twist, '/cmd_vel', 10)
        s.odo = []
        s.create_subscription(Odometry, '/odom', lambda m: s.odo.append(m), 10)
    def wait_odom(s, sec=10):
        t0 = time.time()
        while not s.odo and time.time() - t0 < sec:
            rclpy.spin_once(s, timeout_sec=0.2)
        return bool(s.odo)
    def drive(s, vx, wz, dur):
        """Publie la consigne a 20 Hz (le firmware coupe apres 500 ms de silence)."""
        s.odo.clear()
        tw = Twist(); tw.linear.x = vx; tw.angular.z = wz
        t0 = time.time()
        while time.time() - t0 < dur:
            s.pub.publish(tw); rclpy.spin_once(s, timeout_sec=0.02); time.sleep(0.03)
        for _ in range(15):
            s.pub.publish(Twist()); rclpy.spin_once(s, timeout_sec=0.02); time.sleep(0.03)
        t0 = time.time()
        while time.time() - t0 < 1.0:
            rclpy.spin_once(s, timeout_sec=0.1)
        return list(s.odo)

def wheels(samples):
    """Vitesses de roue moyennes, reconstruites depuis vx et wz de /odom."""
    if len(samples) < 3:
        return None
    vx = sum(m.twist.twist.linear.x for m in samples) / len(samples)
    wz = sum(m.twist.twist.angular.z for m in samples) / len(samples)
    return vx - wz * TRACK / 2, vx + wz * TRACK / 2, vx, wz

rclpy.init(); n = M()
if not n.wait_odom():
    print("AUCUNE ODOMETRIE : le firmware ne repond pas, test impossible.")
    sys.exit(1)
print("odometrie recue, debut du test\n")
print(f"{'essai':<24} {'roue G':>9} {'roue D':>9}   {'vx':>7} {'wz':>7}  verdict")
print("-" * 78)

TESTS = [
    ("avant       0.10 m/s",  0.10,  0.0),
    ("arriere    -0.10 m/s", -0.10,  0.0),
    ("rotation +  0.40 rad/s", 0.0,  0.40),
    ("rotation -  -0.40 rad/s", 0.0, -0.40),
]
res = {}
for label, vx, wz in TESTS:
    w = wheels(n.drive(vx, wz, 3.0))
    if not w:
        print(f"{label:<24} {'pas de donnees':>21}")
        continue
    gl, dr, mvx, mwz = w
    res[label] = (gl, dr)
    if abs(gl) < 0.005 and abs(dr) < 0.005:   verdict = "AUCUN MOUVEMENT"
    elif abs(gl) < 0.005:                     verdict = "roue GAUCHE morte"
    elif abs(dr) < 0.005:                     verdict = "roue DROITE morte"
    else:                                     verdict = "les deux tournent"
    print(f"{label:<24} {gl:+9.3f} {dr:+9.3f}   {mvx:+7.3f} {mwz:+7.3f}  {verdict}")

print("-" * 78)
av = res.get("avant       0.10 m/s"); ar = res.get("arriere    -0.10 m/s")
rp = res.get("rotation +  0.40 rad/s"); rm = res.get("rotation -  -0.40 rad/s")
if av and all(v > 0.005 for v in av):   print("  avant : les deux roues en marche AVANT      -> sens correct")
elif av:                                print(f"  avant : signes inattendus {av} -> cablage ou inversion a revoir")
if ar and all(v < -0.005 for v in ar):  print("  arriere : les deux roues en marche ARRIERE  -> sens correct")
if rp and rp[0] * rp[1] < 0:            print("  rotation + : roues en sens OPPOSES          -> rotation correcte")
if rm and rm[0] * rm[1] < 0:            print("  rotation - : roues en sens OPPOSES          -> rotation correcte")
