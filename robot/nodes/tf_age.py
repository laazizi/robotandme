#!/usr/bin/env python3
"""Age de la TF map->odom, separe selon que le robot BOUGE ou non.

    mowbot node tf_age.py           40 s
    mowbot node tf_age.py 90        duree en secondes

Aucune coordination necessaire : l'outil classe lui-meme ses releves. Un seul
lancement repond, que le robot roule pendant tout ce temps, une partie, ou pas
du tout.

L'HYPOTHESE QU'IL TESTE. slam_toolbox horodate map->odom avec le dernier scan
qu'il a TRAITE, et il ne traite un nouveau scan que lorsque le robot s'est
deplace de minimum_travel_distance ou tourne de minimum_travel_heading. Entre
deux traitements il republie pourtant la transformation a 20 Hz
(transform_publish_period), avec le MEME horodatage ancien.

Consequence attendue si l'hypothese est juste : robot immobile, l'horodatage se
fige et l'age croit lineairement -- ce qui n'est PAS une panne. Robot en
mouvement, l'age reste faible.

Consequence si elle est FAUSSE : l'age reste eleve meme en mouvement, et il faut
alors chercher un vrai retard de traitement -- attente de TF, verrou interne,
scan_buffer_size. Ce ne serait pas un manque de CPU : slam_toolbox est
mono-thread et mesure 17 a 25 % d'UN coeur, soit 28 ms par scan pour 166 ms
disponibles.

POURQUOI CETTE DISTINCTION COMPTE. Un age qui ne grandit qu'a l'arret gene
seulement le PREMIER instant d'un but, quand le robot part de l'immobilite --
ce qui suffit a expliquer un but refuse en 0,1 s, mais pas un trajet entier.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

# Seuils de mouvement. Au-dela, le robot bouge assez pour que slam_toolbox
# finisse par franchir minimum_travel_distance / _heading.
BOUGE_LIN = 0.02      # m/s
BOUGE_ROT = 0.05      # rad/s


class AgeTF(Node):
    def __init__(self):
        super().__init__('mowbot_tf_age')
        self.v = 0.0
        self.w = 0.0
        self.dernier_stamp = None
        self.dernier_change = None
        self.releves = []        # (age, bouge)
        self.refresh = []        # (intervalle entre deux horodatages, bouge)
        self.create_subscription(Odometry, '/odom', self._odom,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self._tf, 100)

    def _odom(self, m):
        self.v = m.twist.twist.linear.x
        self.w = m.twist.twist.angular.z

    def _bouge(self):
        return abs(self.v) > BOUGE_LIN or abs(self.w) > BOUGE_ROT

    def _tf(self, m):
        maintenant = self.get_clock().now().nanoseconds / 1e9
        for tr in m.transforms:
            if tr.header.frame_id != 'map' or tr.child_frame_id != 'odom':
                continue
            st = tr.header.stamp.sec + tr.header.stamp.nanosec / 1e9
            self.releves.append((maintenant - st, self._bouge()))
            # Un CHANGEMENT d'horodatage signale que slam_toolbox a traite un
            # nouveau scan. C'est la mesure la plus directe de l'hypothese.
            if self.dernier_stamp is None:
                self.dernier_stamp, self.dernier_change = st, maintenant
            elif abs(st - self.dernier_stamp) > 1e-6:
                self.refresh.append((maintenant - self.dernier_change,
                                     self._bouge()))
                self.dernier_stamp, self.dernier_change = st, maintenant


def resume(nom, xs, unite='s'):
    if not xs:
        return "%-24s aucun releve" % nom
    xs = sorted(xs)
    return ("%-24s %4d  median %6.2f %s  min %6.2f  max %7.2f"
            % (nom, len(xs), xs[len(xs) // 2], unite, xs[0], xs[-1]))


def rapport(n):
    if not n.releves:
        print("\naucune TF map->odom recue. Le SLAM tourne-t-il ?")
        print("  mowbot status   puis   mowbot logs nav")
        return
    bouge = [a for a, b in n.releves if b]
    arret = [a for a, b in n.releves if not b]
    print("\n%d releves : %d en mouvement, %d a l'arret"
          % (len(n.releves), len(bouge), len(arret)))
    print("\n--- AGE DE map->odom ---")
    print("  " + resume("robot EN MOUVEMENT", bouge))
    print("  " + resume("robot A L'ARRET", arret))

    rb = [x for x, b in n.refresh if b]
    ra = [x for x, b in n.refresh if not b]
    print("\n--- INTERVALLE ENTRE DEUX HORODATAGES ---")
    print("  (delai entre deux scans reellement traites par slam_toolbox)")
    print("  " + resume("en mouvement", rb))
    print("  " + resume("a l'arret", ra))

    print("\n=== VERDICT ===")
    if not bouge:
        print("  LE ROBOT N'A PAS BOUGE pendant l'observation : impossible de")
        print("  trancher. Relancer en lui envoyant un but depuis RViz :")
        print("    mowbot node tf_age.py 90")
        return
    med_b = sorted(bouge)[len(bouge) // 2]
    if med_b < 1.0:
        print("  EN MOUVEMENT la TF est FRAICHE (%.2f s de median)." % med_b)
        if arret and sorted(arret)[len(arret) // 2] > 3.0:
            print("  A L'ARRET elle vieillit (%.2f s de median)."
                  % sorted(arret)[len(arret) // 2])
            print("  >> HYPOTHESE CONFIRMEE : l'horodatage se fige quand le")
            print("     robot ne bouge pas, parce que slam_toolbox ne traite")
            print("     plus de scan. Ce n'est PAS un retard de traitement.")
            print("     Cela gene seulement le premier instant d'un but, quand")
            print("     le robot part de l'immobilite -- ce qui peut suffire a")
            print("     faire refuser un but en 0,1 s.")
            print("     Correctif : baisser minimum_travel_distance et")
            print("     minimum_travel_heading pour rafraichir plus souvent,")
            print("     au prix d'un graphe de poses plus dense.")
        else:
            print("  >> RIEN A SIGNALER : la TF est fraiche dans les deux cas.")
    else:
        print("  EN MOUVEMENT la TF est DEJA PERIMEE (%.2f s de median)."
              % med_b)
        print("  >> HYPOTHESE REFUTEE : c'est un vrai retard de traitement.")
        print("     Ce n'est pas un manque de CPU -- slam_toolbox est")
        print("     mono-thread et n'utilise que 17 a 25 %% d'un coeur.")
        print("     Chercher : transform_timeout, scan_buffer_size, ou une")
        print("     attente sur la TF odom->base_link (mesuree a +0,45 s).")


def main():
    duree = 40.0
    if len(sys.argv) > 1:
        try:
            duree = float(sys.argv[1])
        except ValueError:
            print("duree invalide : %r" % sys.argv[1], file=sys.stderr)
            return 1
    print("observation de %.0f s." % duree)
    print("FAIRE ROULER LE ROBOT pendant une partie du temps, sinon l'outil")
    print("ne pourra pas comparer les deux situations.")
    rclpy.init()
    n = AgeTF()
    fin = time.time() + duree
    try:
        while rclpy.ok() and time.time() < fin:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    rapport(n)
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
