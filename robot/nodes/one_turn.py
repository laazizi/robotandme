#!/usr/bin/env python3
"""Fait tourner UNE roue d'exactement 360 deg, puis l'autre. Roues en l'air.

/cmd_vel ne pilote pas les roues separement, mais la cinematique le permet :
  v_gauche = vx - wz*L/2      v_droite = vx + wz*L/2
Pour n'animer que la DROITE : vx = V/2 et wz = +V/L  (v_gauche s'annule).
Pour la GAUCHE : wz = -V/L.

L'arret est asservi sur la distance vue par l'ENCODEUR de la roue concernee,
integree depuis /odom. Si cet encodeur ne compte pas, on ne peut que compter le
TEMPS -- et l'ecart avec la marque physique mesure alors l'erreur.
"""
import math, os, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

WHEEL_R = 0.0698
TRACK   = float(os.environ.get('MOWBOT_TRACK', '0.59'))
CIRC    = 2 * math.pi * WHEEL_R          # 0.4386 m pour un tour
V       = float(os.environ.get('MOWBOT_V', '0.05'))   # lent : ~8.8 s le tour
TIMEOUT = CIRC / V * 3                   # garde-fou

class T(Node):
    def __init__(s):
        super().__init__('one_turn')
        s.pub = s.create_publisher(Twist, '/cmd_vel', 10)
        s.g = s.d = 0.0          # distances integrees par roue
        s.t_prev = None
        s.create_subscription(Odometry, '/odom', s.cb, 10)
    def cb(s, m):
        now = time.time()
        if s.t_prev is not None:
            dt = now - s.t_prev
            vx, wz = m.twist.twist.linear.x, m.twist.twist.angular.z
            s.g += (vx - wz * TRACK / 2) * dt
            s.d += (vx + wz * TRACK / 2) * dt
        s.t_prev = now
    def reset(s):
        s.g = s.d = 0.0; s.t_prev = None
    def turn(s, side):
        """side = 'gauche' ou 'droite'"""
        sign = -1.0 if side == 'gauche' else +1.0
        tw = Twist(); tw.linear.x = V / 2; tw.angular.z = sign * V / TRACK
        s.reset()
        for _ in range(10):                    # amorce l'integration
            rclpy.spin_once(s, timeout_sec=0.05)
        s.reset()
        print(f"  roue {side} : un tour ({CIRC*100:.1f} cm de bande) a {V} m/s, "
              f"~{CIRC/V:.1f} s attendues")
        t0 = time.time()
        while True:
            s.pub.publish(tw)
            rclpy.spin_once(s, timeout_sec=0.02)
            time.sleep(0.03)
            parcouru = abs(s.g if side == 'gauche' else s.d)
            el = time.time() - t0
            if parcouru >= CIRC:
                raison = f"encodeur ({parcouru*100:.1f} cm)"
                break
            if el >= CIRC / V:                 # duree theorique atteinte
                raison = f"TEMPS ecoule (encodeur n'a vu que {parcouru*100:.1f} cm)"
                break
            if el > TIMEOUT:
                raison = "garde-fou de securite"
                break
        for _ in range(20):
            s.pub.publish(Twist()); rclpy.spin_once(s, timeout_sec=0.02); time.sleep(0.03)
        autre = abs(s.d if side == 'gauche' else s.g)
        print(f"    arret sur : {raison}   duree {el:.1f} s")
        print(f"    l'autre roue a bouge de {autre*100:.1f} cm "
              f"({'OK, elle est restee immobile' if autre < 0.05 else 'ATTENTION : elle a tourne aussi'})")
        return parcouru, el

rclpy.init(); n = T()
t0 = time.time()
while n.t_prev is None and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
if n.t_prev is None:
    print("pas d'odometrie"); raise SystemExit(1)

print(f"\nUn tour de roue = {CIRC*100:.1f} cm de bande de roulement\n")
for side in ('droite', 'gauche'):
    print(f">>> ROUE {side.upper()}")
    n.turn(side)
    print("    -- mesurez la marque, 6 s de pause --\n")
    t0 = time.time()
    while time.time() - t0 < 6:
        rclpy.spin_once(n, timeout_sec=0.1)
print("termine. Comparez chaque marque a un tour complet.")
