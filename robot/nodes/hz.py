#!/usr/bin/env python3
"""Mesure la cadence d'un topic. Affiche un nombre, ou rien s'il est muet.

    python3 hz.py /odom [duree_s]

POURQUOI NE PAS UTILISER `ros2 topic hz` : il ne produit AUCUNE sortie dans
plusieurs contextes -- constate sous Jazzy dans un conteneur, y compris sans
passer par un relais et avec PYTHONUNBUFFERED. `mowbot status` rapportait alors
TOUS les topics comme MUETS alors que la pile tournait parfaitement : /odom a
10.5 Hz, /scan a 5.6 Hz, map->odom disponible en 1.4 s. Un diagnostic faux est
pire qu'une absence de diagnostic -- on cherche une panne qui n'existe pas.

Le type du message est lu dans le graphe ROS : inutile de le connaitre a
l'avance, et ca marche donc sur n'importe quel topic.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def main():
    if len(sys.argv) < 2:
        print("usage: hz.py <topic> [duree_s]", file=sys.stderr)
        return 2
    topic = sys.argv[1]
    duree = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0

    rclpy.init()
    n = Node('mowbot_hz')

    # Attendre que le topic apparaisse dans le graphe : juste apres le
    # demarrage d'un noeud, la decouverte n'est pas instantanee.
    typ = None
    t0 = time.time()
    while time.time() - t0 < 3.0 and typ is None:
        for name, types in n.get_topic_names_and_types():
            if name == topic and types:
                typ = types[0]
                break
        if typ is None:
            time.sleep(0.2)
    if typ is None:
        n.destroy_node()
        rclpy.shutdown()
        return 1

    cls = get_message(typ)
    compte = [0]

    # UN SEUL abonnement, en BEST_EFFORT / VOLATILE : les regles de
    # compatibilite QoS font qu'il recoit aussi bien d'un editeur RELIABLE
    # (/odom, /map) que BEST_EFFORT (les capteurs). L'inverse serait faux -- un
    # abonne RELIABLE ne recoit rien d'un editeur BEST_EFFORT.
    n.create_subscription(cls, topic,
                          lambda m: compte.__setitem__(0, compte[0] + 1),
                          qos_profile_sensor_data)

    t0 = time.time()
    while time.time() - t0 < duree:
        rclpy.spin_once(n, timeout_sec=0.2)
    el = time.time() - t0

    n.destroy_node()
    rclpy.shutdown()
    if compte[0] == 0:
        return 1
    # Les deux abonnements comptent le meme message deux fois si les deux QoS
    # sont compatibles : on divise par le nombre d'abonnements effectifs.
    print(f"{compte[0] / el:.2f}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
