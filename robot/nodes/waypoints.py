#!/usr/bin/env python3
"""Parcours de points de passage en boucle, via l'action FollowWaypoints.

  mowbot waypoints                       carre de 1 m, boucle infinie
  mowbot waypoints mes_points.txt        fichier de points, boucle infinie
  mowbot waypoints --tours 3             3 tours puis arret
  mowbot waypoints --points "1,0,0 1,1,90 0,1,180 0,0,-90"

POURQUOI FollowWaypoints ET NON UNE SUITE DE NavigateToPose. C'est le serveur
waypoint_follower de nav2 qui enchaine les points : il gere lui-meme le passage
au suivant, signale ceux qu'il n'a pas pu atteindre (missed_waypoints) au lieu
de s'arreter au premier echec, et un seul but est actif a la fois -- donc pas de
concurrence sur /cmd_vel.

FORMAT DU FICHIER : une ligne par point, "x y cap_en_degres", le cap etant
facultatif (0 par defaut). Les lignes vides et celles commencant par # sont
ignorees. Coordonnees dans le repere `map`, en metres.

L'ARRET PAR Ctrl+C ANNULE LE BUT EN COURS. Sans cela le robot continuerait de
rouler vers le point suivant alors que le script est mort, et il faudrait
arreter nav2 pour l'immobiliser.
"""
import argparse
import math
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints

# Carre de 1 m, cap tangent au parcours : de quoi verifier l'odometrie et les
# rotations sans dependre d'un fichier.
CARRE = [(1.0, 0.0, 0.0), (1.0, 1.0, 90.0), (0.0, 1.0, 180.0), (0.0, 0.0, -90.0)]


def lire_points(chemin):
    pts = []
    with open(chemin) as f:
        for n, ligne in enumerate(f, 1):
            ligne = ligne.split('#')[0].strip()
            if not ligne:
                continue
            ch = ligne.replace(',', ' ').split()
            if len(ch) not in (2, 3):
                raise ValueError(
                    f"{chemin}:{n} : attendu 'x y [cap]', lu {ligne!r}")
            x, y = float(ch[0]), float(ch[1])
            cap = float(ch[2]) if len(ch) == 3 else 0.0
            pts.append((x, y, cap))
    if not pts:
        raise ValueError(f"{chemin} ne contient aucun point")
    return pts


def parse_points(txt):
    pts = []
    for bloc in txt.split():
        ch = bloc.split(',')
        if len(ch) not in (2, 3):
            raise ValueError(f"point {bloc!r} : attendu x,y[,cap]")
        pts.append((float(ch[0]), float(ch[1]),
                    float(ch[2]) if len(ch) == 3 else 0.0))
    return pts


class Parcours(Node):
    def __init__(self, points, tours, repere):
        super().__init__('mowbot_waypoints')
        self.points = points
        self.tours = tours
        self.repere = repere
        self.cli = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.but = None
        self.tour = 0
        self.atteints = 0
        self.manques = 0

    def poses(self):
        out = []
        for x, y, cap in self.points:
            p = PoseStamped()
            p.header.frame_id = self.repere
            # Horodatage a ZERO et non `now()` : le serveur interprete alors la
            # pose comme "la derniere connue", ce qui evite un rejet pour
            # extrapolation si l'horloge du PC et celle du robot divergent.
            p.header.stamp.sec = 0
            p.header.stamp.nanosec = 0
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.z = math.sin(math.radians(cap) / 2.0)
            p.pose.orientation.w = math.cos(math.radians(cap) / 2.0)
            out.append(p)
        return out

    def attendre_serveur(self, secondes=30):
        self.get_logger().info(
            "attente du serveur follow_waypoints "
            "(fourni par waypoint_follower, dans nav2)...")
        if not self.cli.wait_for_server(timeout_sec=secondes):
            self.get_logger().error(
                "follow_waypoints absent apres %d s. Verifier que nav2 tourne :"
                "  mowbot status   puis   mowbot logs nav" % secondes)
            return False
        return True

    def lancer_tour(self):
        self.tour += 1
        if self.tours and self.tour > self.tours:
            self.get_logger().info(
                "TERMINE : %d tour(s), %d point(s) atteint(s), %d manque(s)"
                % (self.tours, self.atteints, self.manques))
            self.but = 'fini'
            return
        etiquette = ("tour %d/%d" % (self.tour, self.tours) if self.tours
                     else "tour %d (boucle infinie, Ctrl+C pour arreter)"
                          % self.tour)
        self.get_logger().info("%s : %d points" % (etiquette, len(self.points)))
        msg = FollowWaypoints.Goal()
        msg.poses = self.poses()
        f = self.cli.send_goal_async(msg, feedback_callback=self.retour)
        f.add_done_callback(self.accepte)

    def retour(self, fb):
        i = fb.feedback.current_waypoint
        if 0 <= i < len(self.points):
            x, y, cap = self.points[i]
            self.get_logger().info(
                "  -> point %d/%d  (%.2f, %.2f) cap %.0f deg"
                % (i + 1, len(self.points), x, y, cap))

    def accepte(self, fut):
        self.but = fut.result()
        if not self.but.accepted:
            self.get_logger().error(
                "but REFUSE par waypoint_follower. Cause frequente : la pose du"
                " robot est inconnue dans le repere '%s' (pas de TF map->odom)."
                % self.repere)
            self.but = 'fini'
            return
        self.but.get_result_async().add_done_callback(self.termine)

    def termine(self, fut):
        res = fut.result().result
        rates = list(getattr(res, 'missed_waypoints', []) or [])
        n = len(self.points)
        self.atteints += n - len(rates)
        self.manques += len(rates)
        if rates:
            # On NE S'ARRETE PAS sur un point manque : c'est justement ce qu'on
            # veut observer en boucle. Le numero affiche est en base 1.
            self.get_logger().warning(
                "tour %d : %d point(s) NON atteint(s) : %s"
                % (self.tour, len(rates), ', '.join(str(i + 1) for i in rates)))
        else:
            self.get_logger().info("tour %d : tous les points atteints"
                                   % self.tour)
        self.get_logger().info("cumul : %d atteints, %d manques"
                               % (self.atteints, self.manques))
        self.lancer_tour()

    def annuler(self):
        """Annule le but en cours pour que le robot s'arrete avec le script."""
        if self.but and self.but != 'fini' and getattr(self.but, 'accepted', False):
            self.get_logger().info("annulation du but en cours...")
            self.cli._cancel_goal_async(self.but)
            fin = self.get_clock().now().nanoseconds + 3_000_000_000
            while self.get_clock().now().nanoseconds < fin:
                rclpy.spin_once(self, timeout_sec=0.1)


def main():
    ap = argparse.ArgumentParser(
        description="Parcours de points de passage en boucle (nav2).")
    ap.add_argument('fichier', nargs='?',
                    help="fichier de points 'x y [cap]'. Sans argument : "
                         "carre de 1 m")
    ap.add_argument('--points', help='points en ligne : "x,y,cap x,y,cap ..."')
    ap.add_argument('--tours', type=int, default=0,
                    help='nombre de tours (0 = boucle infinie, defaut)')
    ap.add_argument('--repere', default='map', help="repere des points")
    a = ap.parse_args()

    try:
        if a.points:
            pts = parse_points(a.points)
        elif a.fichier:
            pts = lire_points(a.fichier)
        else:
            pts = CARRE
    except (ValueError, OSError) as e:
        print("ERREUR : %s" % e, file=sys.stderr)
        return 1

    print("%d point(s) dans le repere %s :" % (len(pts), a.repere))
    for i, (x, y, cap) in enumerate(pts, 1):
        print("  %2d  x=%6.2f  y=%6.2f  cap=%4.0f deg" % (i, x, y, cap))
    print("tours : %s" % (a.tours if a.tours else "infini (Ctrl+C pour arreter)"))

    rclpy.init()
    n = Parcours(pts, a.tours, a.repere)
    code = 0
    try:
        if not n.attendre_serveur():
            return 1
        n.lancer_tour()
        while rclpy.ok() and n.but != 'fini':
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        print()
        n.annuler()
        print("interrompu : %d atteints, %d manques" % (n.atteints, n.manques))
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
