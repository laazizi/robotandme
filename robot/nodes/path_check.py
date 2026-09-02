#!/usr/bin/env python3
"""Le robot SUIT-IL le chemin global (le vert de RViz) ? Chiffre l'ecart.

    mowbot node path_check.py           30 s
    mowbot node path_check.py 60        duree en secondes

A LANCER PENDANT QU'UN BUT EST ACTIF : /plan et /local_plan ne publient que
dans ce cas. Au repos, rien a mesurer.

CE QUE CET OUTIL REPOND, et que ni le journal de nav2 ni cmd_check ne disent :
  . a quelle distance LATERALE du chemin global le robot se trouve ;
  . si sa trajectoire locale va dans le MEME SENS que le chemin, ou a l'oppose ;
  . si le chemin passe reellement par le robot, ou s'il commence loin de lui.

Constat qui a motive l'outil : sur une capture RViz, le chemin vert finissait en
haut a droite du robot pendant que la trajectoire bleue partait vers la gauche.
Le taux de suivi des commandes etait pourtant a 100 % (cmd_check) : le robot
obeissait parfaitement... a des commandes qui l'eloignaient de son chemin. Les
deux outils sont donc complementaires, et aucun ne remplace l'autre.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener


def ecart_angulaire(a, b):
    """Difference d'angles en degres, ramenee dans [-180, 180]."""
    return (a - b + 180.0) % 360.0 - 180.0


class SuiviChemin(Node):
    def __init__(self):
        super().__init__('mowbot_path_check')
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.plan = None
        self.local = None
        self.cmd = None
        self.releves = []
        # /plan et /local_plan sont publies en RELIABLE par nav2, profondeur 1 :
        # un abonne fiable suffit, et on ne veut que le dernier.
        self.create_subscription(Path, '/plan', self._plan, 1)
        self.create_subscription(Path, '/local_plan', self._local, 1)
        self.create_subscription(Twist, '/cmd_vel', self._cmd,
                                 qos_profile_sensor_data)
        self.create_timer(0.5, self._mesure)

    def _plan(self, m):
        self.plan = m

    def _local(self, m):
        self.local = m

    def _cmd(self, m):
        self.cmd = m

    def _pose(self):
        t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        q = t.transform.rotation
        cap = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y * q.y + q.z * q.z)))
        return t.transform.translation.x, t.transform.translation.y, cap

    def _mesure(self):
        if self.plan is None or len(self.plan.poses) < 2:
            return
        try:
            rx, ry, rcap = self._pose()
        except Exception:
            return

        pts = [(p.pose.position.x, p.pose.position.y) for p in self.plan.poses]
        # ECART LATERAL : distance au point du chemin le plus proche. C'est la
        # mesure que l'oeil fait sur RViz, en metres.
        d = [math.hypot(x - rx, y - ry) for x, y in pts]
        i = min(range(len(d)), key=lambda k: d[k])
        ecart = d[i]

        # OU MENE LE CHEMIN, vu du robot : premier point du chemin a plus de
        # 30 cm devant, pour ne pas mesurer un cap sur du bruit.
        cible = None
        for k in range(i, len(pts)):
            if d[k] > 0.30:
                cible = pts[k]
                break
        cap_chemin = (math.degrees(math.atan2(cible[1] - ry, cible[0] - rx))
                      if cible else None)

        # OU VA LA TRAJECTOIRE LOCALE, c'est-a-dire ce que le robot va faire.
        cap_local = None
        if self.local is not None and len(self.local.poses) >= 2:
            a = self.local.poses[0].pose.position
            b = self.local.poses[-1].pose.position
            if math.hypot(b.x - a.x, b.y - a.y) > 0.02:
                cap_local = math.degrees(math.atan2(b.y - a.y, b.x - a.x))

        v = self.cmd.linear.x if self.cmd else 0.0
        w = self.cmd.angular.z if self.cmd else 0.0
        self.releves.append((ecart, cap_chemin, cap_local, rcap, v, w,
                             d[0], len(pts)))


def rapport(r):
    if not r:
        print("\naucun relevé : aucun but actif pendant l'observation.")
        print("Envoyer un but depuis RViz PENDANT que cet outil tourne.")
        return
    print("\n%d relevés (un toutes les 0,5 s)" % len(r))

    ec = sorted(x[0] for x in r)
    print("\n--- ECART AU CHEMIN GLOBAL (le vert de RViz) ---")
    print("  median %.2f m   maximum %.2f m" % (ec[len(ec) // 2], ec[-1]))
    print("  au-dela de 0,50 m : %d relevé(s) sur %d"
          % (sum(1 for x in ec if x > 0.50), len(ec)))

    deb = sorted(x[6] for x in r)
    print("\n--- LE CHEMIN COMMENCE-T-IL AU ROBOT ? ---")
    print("  distance du robot au PREMIER point du chemin :"
          " median %.2f m, max %.2f m" % (deb[len(deb) // 2], deb[-1]))
    if deb[len(deb) // 2] > 0.5:
        print("  >> le chemin ne part PAS du robot : le planificateur le calcule")
        print("     depuis une pose qui n'est pas la sienne, ou le chemin est")
        print("     perime. Verifier la TF map->base_link.")

    paires = [(x[2], x[1]) for x in r if x[1] is not None and x[2] is not None]
    print("\n--- SENS DE LA TRAJECTOIRE LOCALE vs CHEMIN ---")
    if not paires:
        print("  pas de /local_plan exploitable (robot immobile ?)")
    else:
        ecarts = sorted(abs(ecart_angulaire(a, b)) for a, b in paires)
        med = ecarts[len(ecarts) // 2]
        contre = sum(1 for e in ecarts if e > 120)
        print("  ecart angulaire median : %.0f deg  (sur %d relevés)"
              % (med, len(ecarts)))
        print("  trajectoire a CONTRE-SENS (>120 deg) : %d relevé(s)" % contre)
        if contre > 0.2 * len(ecarts):
            print("  >> LE ROBOT S'ELOIGNE DE SON CHEMIN une bonne part du temps.")
        elif med > 60:
            print("  >> divergence notable, mais pas un contre-sens franc.")
        else:
            print("  >> la trajectoire locale suit bien le chemin.")

    vit = sorted(abs(x[4]) for x in r)
    rot = sorted(abs(x[5]) for x in r)
    print("\n--- CE QUE NAV2 COMMANDE ---")
    print("  avance  : mediane %.3f m/s   max %.3f m/s"
          % (vit[len(vit) // 2], vit[-1]))
    print("  rotation: mediane %.3f rad/s max %.3f rad/s"
          % (rot[len(rot) // 2], rot[-1]))
    immobile = sum(1 for x in r if abs(x[4]) < 0.01 and abs(x[5]) < 0.01)
    print("  relevés a commande NULLE : %d sur %d (%.0f %%)"
          % (immobile, len(r), 100 * immobile / len(r)))
    if immobile > 0.3 * len(r):
        print("  >> nav2 ne commande RIEN une bonne part du temps : il attend")
        print("     ou il n'a pas de trajectoire. Chercher dans le journal :")
        print("     mowbot logs nav | grep -E 'No valid|Oscillation|recovery'")


def main():
    duree = 30.0
    if len(sys.argv) > 1:
        try:
            duree = float(sys.argv[1])
        except ValueError:
            print("duree invalide : %r" % sys.argv[1], file=sys.stderr)
            return 1
    print("observation de %.0f s. ENVOYER UN BUT pendant ce temps." % duree)
    rclpy.init()
    n = SuiviChemin()
    fin = time.time() + duree
    try:
        while rclpy.ok() and time.time() < fin:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    rapport(n.releves)
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
