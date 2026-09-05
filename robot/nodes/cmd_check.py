#!/usr/bin/env python3
"""Les consignes de nav2 font-elles VRAIMENT tourner les moteurs ?

    mowbot node cmd_check.py                 30 s d'observation
    mowbot node cmd_check.py --duree 120     pendant une navigation complete
    mowbot node cmd_check.py --seuil 0.11    seuil de demarrage mesure du robot

POURQUOI CET OUTIL. Ce robot a un SEUIL DE DEMARRAGE : en dessous d'une
certaine vitesse, le moteur ne tourne pas du tout. Mesures du 5 septembre 2026 :

    mode regime : rien sous ~1200 ERPM, soit 0,386 m/s  -- MESURE
    mode duty   : bien plus bas. Une mesure du 5 septembre montre le robot
                  tournant a 0,076 m/s pour 0,077 demandes, soit 99 % de la
                  consigne. Le plancher exact reste a etablir ; 0,05 est la plus
                  basse valeur observee en fonctionnement. D'ou le defaut a 0,05
                  et non a 0,114, qui etait une extrapolation FAUSSE de ma part.

Or nav2 ralentit en approchant du but, et MPPI commande volontiers des vitesses
faibles en manoeuvre serree. Ces consignes-la partent DANS LE VIDE : le robot ne
bouge pas, le detecteur de progression conclut au blocage, et la recuperation
s'enclenche pour rien. Le journal de nav2 ne dit rien de tout cela -- il croit
avoir commande une vitesse.

Cet outil compare ce qui est DEMANDE a ce qui est OBTENU et compte le temps
passe sous le seuil. C'est le seul moyen de voir le probleme.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Verificateur(Node):
    def __init__(self, seuil, duree):
        super().__init__('cmd_check')
        self.seuil = seuil
        self.fin = time.time() + duree
        q = QoSProfile(depth=20)
        q.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Twist, 'cmd_vel', self.sur_cmd, 20)
        self.create_subscription(Odometry, 'odom', self.sur_odom, q)
        self.cmds = []          # (t, vx, wz)
        self.odos = []          # (t, vx, wz)
        self.t0 = time.time()

    def sur_cmd(self, m):
        self.cmds.append((time.time(), m.linear.x, m.angular.z))

    def sur_odom(self, m):
        self.odos.append((time.time(), m.twist.twist.linear.x,
                          m.twist.twist.angular.z))

    def termine(self):
        return time.time() >= self.fin


def rapport(v):
    d = time.time() - v.t0
    print("\n  === %.0f s d'observation ===" % d)
    if not v.cmds:
        print("  AUCUNE consigne sur /cmd_vel : nav2 n'a rien commande.")
        print("  Soit aucun but n'etait actif, soit le controleur a refuse de planifier.")
        return 1
    if not v.odos:
        print("  AUCUNE odometrie : le pilote VESC ne tourne pas.")
        return 1

    bougent = [c for c in v.cmds if abs(c[1]) > 1e-6 or abs(c[2]) > 1e-6]
    print("  consignes recues        %d  (%.1f Hz), dont %d non nulles"
          % (len(v.cmds), len(v.cmds) / d, len(bougent)))
    if not bougent:
        print("  Toutes les consignes etaient a ZERO : rien a verifier.")
        return 0

    vx = [abs(c[1]) for c in bougent]
    sous = [x for x in vx if x < v.seuil]
    print("  vitesse demandee        min %.3f | mediane %.3f | max %.3f m/s"
          % (min(vx), sorted(vx)[len(vx) // 2], max(vx)))
    print("  SOUS LE SEUIL de %.3f  %d consignes sur %d  (%.0f %%)"
          % (v.seuil, len(sous), len(vx), 100.0 * len(sous) / len(vx)))
    if len(sous) > 0.25 * len(vx):
        print("     -> PROBLEME : plus d'un quart des consignes ne peuvent PAS")
        print("        faire tourner les moteurs. Le robot paraitra bloque.")
    elif sous:
        print("     -> a surveiller, mais l'essentiel des consignes est utile.")
    else:
        print("     -> toutes les consignes sont au-dessus du seuil.")

    # obtenu contre demande, en appariant chaque consigne a l'odometrie qui suit
    paires = []
    for tc, cvx, _ in bougent:
        proches = [o for o in v.odos if tc + 0.15 <= o[0] <= tc + 0.6]
        if proches:
            paires.append((cvx, sum(o[1] for o in proches) / len(proches)))
    if paires:
        dem = sum(abs(p[0]) for p in paires) / len(paires)
        obt = sum(abs(p[1]) for p in paires) / len(paires)
        print("  demande moyen %.3f m/s -> obtenu %.3f m/s  (%.0f %% de la consigne)"
              % (dem, obt, 100.0 * obt / dem if dem else 0))
        muets = sum(1 for c, o in paires if abs(c) > 1e-6 and abs(o) < 0.02)
        print("  consignes SANS AUCUN mouvement : %d sur %d (%.0f %%)"
              % (muets, len(paires), 100.0 * muets / len(paires)))
        if muets > 0.2 * len(paires):
            print("     -> CONFIRME : le robot recoit des ordres et ne bouge pas.")
    return 0


def main():
    p = argparse.ArgumentParser(description="Verifie que les consignes de nav2 font tourner les moteurs.")
    p.add_argument('--duree', type=float, default=30.0, help="duree d'observation [s]")
    p.add_argument('--seuil', type=float, default=0.05,
                   help="vitesse minimale a laquelle le moteur demarre [m/s]")
    a = p.parse_args()
    rclpy.init()
    v = Verificateur(a.seuil, a.duree)
    print("  observation de /cmd_vel et /odom pendant %.0f s, seuil %.3f m/s..."
          % (a.duree, a.seuil))
    try:
        while rclpy.ok() and not v.termine():
            rclpy.spin_once(v, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    code = rapport(v)
    v.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
