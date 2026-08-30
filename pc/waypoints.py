#!/usr/bin/env python3
"""Parcours de points de passage EN BOUCLE, a lancer depuis le PC.

    source /opt/ros/jazzy/setup.bash
    python3 waypoints.py

Les points se modifient DANS CE FICHIER, dans le tableau POINTS ci-dessous.
Ctrl+C annule le but en cours : le robot s'arrete avec le script.

Prerequis :
  . nav2 actif sur le robot          ->  mowbot status
  . nav2_msgs sur ce PC             ->  sudo apt install ros-jazzy-nav2-msgs
"""

# ============================================================================
#  LES POINTS  --  c'est ici qu'on edite
# ============================================================================
#
#   (x, y, cap)
#     x, y : metres dans le repere `map`. L'ORIGINE EST L'ENDROIT OU LE SLAM A
#            DEMARRE, pas un coin de la piece. x vers l'avant du robot au
#            demarrage, y vers sa gauche.
#     cap  : orientation VOULUE en arrivant sur le point, en degres.
#            0 = vers +x,  90 = vers +y,  180 = vers -x,  -90 = vers -y.
#
# Le parcours BOUCLE : apres le dernier point on repart au premier. Verifier
# donc que le retour du dernier au premier est franchissable.
#
# Commencer petit : ce carre de 1 m suffit a voir si l'odometrie derive (le
# robot revient-il au meme endroit apres dix tours ?) et si les rotations sont
# justes.

POINTS = [
    (1.50464, -0.0498953, 0),
    (-1.3173, 0.629804, 0),(-2.06782, -0.442709, 0)
]

TOURS = 0          # nombre de tours ; 0 = boucle infinie
REPERE = 'map'     # repere des coordonnees ci-dessus

# ============================================================================
#  A partir d'ici, plus rien a regler
# ============================================================================
import math
import os
import sys

# AVANT d'importer rclpy : le domaine doit correspondre a celui du robot, sinon
# le PC et le robot ne se voient pas DU TOUT et le script attendrait le serveur
# indefiniment, sans le moindre message d'erreur reseau.
os.environ.setdefault('ROS_DOMAIN_ID', '0')

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
except ImportError:
    sys.exit("ROS 2 n'est pas dans l'environnement.\n"
             "  source /opt/ros/jazzy/setup.bash")
try:
    from nav2_msgs.action import FollowWaypoints
except ImportError:
    # nav2_msgs porte la DEFINITION de l'action. Ce n'est pas la pile nav2,
    # juste des messages : deux petits paquets.
    sys.exit("nav2_msgs est absent de ce PC.\n"
             "  sudo apt install ros-jazzy-nav2-msgs")


class Parcours(Node):
    """Envoie POINTS a waypoint_follower, et recommence a chaque fin de tour.

    POURQUOI FollowWaypoints ET NON UNE SUITE DE NavigateToPose : c'est le
    serveur waypoint_follower de nav2 qui enchaine les points. Il gere lui-meme
    le passage au suivant, signale ceux qu'il n'a PAS pu atteindre au lieu de
    s'arreter au premier echec, et un seul but est actif a la fois -- donc
    aucune concurrence sur /cmd_vel.
    """

    def __init__(self):
        super().__init__('mowbot_waypoints')
        self.cli = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.but = None
        self.tour = 0
        self.atteints = 0
        self.manques = 0
        self.fini = False

    def poses(self):
        out = []
        for x, y, cap in POINTS:
            p = PoseStamped()
            p.header.frame_id = REPERE
            # Horodatage a ZERO et non `now()` : le serveur comprend alors "la
            # derniere pose connue". Avec l'heure du PC, un decalage d'horloge
            # entre le PC et le robot fait rejeter le but pour extrapolation.
            p.header.stamp.sec = 0
            p.header.stamp.nanosec = 0
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.z = math.sin(math.radians(cap) / 2.0)
            p.pose.orientation.w = math.cos(math.radians(cap) / 2.0)
            out.append(p)
        return out

    def demarrer(self):
        print("attente du serveur follow_waypoints (nav2 sur le robot)...")
        if not self.cli.wait_for_server(timeout_sec=30.0):
            print("\nfollow_waypoints introuvable apres 30 s.\n"
                  "  . nav2 tourne-t-il ?        mowbot status\n"
                  "  . meme reseau et meme ROS_DOMAIN_ID des deux cotes ?\n"
                  "  . essai rapide :            ros2 topic list | grep odom",
                  file=sys.stderr)
            return False
        self.tour_suivant()
        return True

    def tour_suivant(self):
        self.tour += 1
        if TOURS and self.tour > TOURS:
            print("\nTERMINE : %d tour(s), %d point(s) atteint(s), %d manque(s)"
                  % (TOURS, self.atteints, self.manques))
            self.fini = True
            return
        etiquette = ("tour %d/%d" % (self.tour, TOURS) if TOURS
                     else "tour %d  (boucle infinie, Ctrl+C pour arreter)"
                          % self.tour)
        print("\n=== %s : %d points ===" % (etiquette, len(POINTS)))
        msg = FollowWaypoints.Goal()
        msg.poses = self.poses()
        self.cli.send_goal_async(msg, feedback_callback=self.avancement) \
            .add_done_callback(self.reponse)

    def avancement(self, fb):
        i = fb.feedback.current_waypoint
        if 0 <= i < len(POINTS):
            x, y, cap = POINTS[i]
            print("  -> point %d/%d   x=%.2f  y=%.2f  cap=%.0f deg"
                  % (i + 1, len(POINTS), x, y, cap))

    def reponse(self, fut):
        self.but = fut.result()
        if not self.but.accepted:
            print("but REFUSE par waypoint_follower.\n"
                  "Cause frequente : la pose du robot est inconnue dans le "
                  "repere '%s' (pas de TF map->odom)." % REPERE, file=sys.stderr)
            self.fini = True
            return
        self.but.get_result_async().add_done_callback(self.termine)

    def termine(self, fut):
        rates = list(getattr(fut.result().result, 'missed_waypoints', []) or [])
        self.atteints += len(POINTS) - len(rates)
        self.manques += len(rates)
        if rates:
            # ON NE S'ARRETE PAS sur un point manque : c'est justement ce qu'on
            # veut observer tour apres tour. Numeros affiches en base 1.
            print("  tour %d : %d point(s) NON atteint(s) : %s"
                  % (self.tour, len(rates),
                     ', '.join(str(i + 1) for i in rates)))
        else:
            print("  tour %d : tous les points atteints" % self.tour)
        print("  cumul : %d atteints, %d manques"
              % (self.atteints, self.manques))
        self.tour_suivant()

    def annuler(self):
        """Arrete le robot avec le script.

        Sans cela, le but reste actif cote robot apres un Ctrl+C : la tondeuse
        continue vers le point suivant alors que le script est mort, et il
        faudrait arreter nav2 pour l'immobiliser.
        """
        if self.but is not None and getattr(self.but, 'accepted', False):
            print("annulation du but en cours...")
            self.but.cancel_goal_async()
            fin = self.get_clock().now().nanoseconds + 3_000_000_000
            while self.get_clock().now().nanoseconds < fin:
                rclpy.spin_once(self, timeout_sec=0.1)


def main():
    print("%d point(s) dans le repere %s :" % (len(POINTS), REPERE))
    for i, (x, y, cap) in enumerate(POINTS, 1):
        print("  %2d   x=%6.2f   y=%6.2f   cap=%4.0f deg" % (i, x, y, cap))
    print("tours : %s" % (TOURS if TOURS else "infini"))
    print("ROS_DOMAIN_ID = %s" % os.environ['ROS_DOMAIN_ID'])

    rclpy.init()
    n = Parcours()
    try:
        if not n.demarrer():
            return 1
        while rclpy.ok() and not n.fini:
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        print()
        n.annuler()
        print("interrompu : %d atteints, %d manques" % (n.atteints, n.manques))
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
