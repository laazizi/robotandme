#!/usr/bin/env python3
"""Publie /joint_states pour la roue directrice, afin que RViz la montre TOURNER.

POURQUOI CE NOEUD EXISTE. mowbot.urdf a un seul joint MOBILE, steer_joint.
robot_state_publisher ne l'anime que s'il recoit /joint_states -- personne ne le
publiait, donc la roue restait figee a zero dans RViz alors qu'elle braquait
vraiment.

D'OU VIENT L'ANGLE. Le firmware ne publie PAS l'angle du servo (servo RC sans
retour). Mais son odometrie en decoule exactement :

    w = v * tan(delta) / x_s        (modele bicyclette, kin_ackermann.c)
    donc  delta = atan(w * x_s / v)

On retrouve donc l'angle COMMANDE, au bit pres, depuis /odom. C'est bien l'angle
commande et non l'angle reel : sans retour de position, personne ne connait le
second. Mesure du 03/09/2026 : le braquage reel est plus faible que le commande
(gyro contre modele), donc ce que RViz montre est ce que le firmware DEMANDE.

A L'ARRET on ne peut rien deduire (v au denominateur) : le servo, lui, garde sa
position, donc on garde le dernier angle connu. C'est fidele au materiel.

La geometrie vient de config/ackerbot_geometry.env, GENERE depuis le firmware
par bin/gen_ackerbot_geometry.py : aucune valeur en dur ici.
"""
import math
import os
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

JOINT = "steer_joint"          # doit correspondre a mowbot.urdf
V_EPS = 0.02                   # m/s : sous ce seuil l'angle est indeductible
PERIODE = 0.05                 # 20 Hz : fluide a l'oeil, negligeable en charge


def lire_geometrie(chemin):
    """-> (x_s signe, delta_max). Absent ou illisible : on ne devine pas."""
    vals = {}
    try:
        with open(chemin, encoding="utf-8") as f:
            for ligne in f:
                m = re.match(r"^(ACKERBOT_STEER_X|ACKERBOT_STEER_MAX_RAD)=([-+0-9.]+)", ligne)
                if m:
                    vals[m.group(1)] = float(m.group(2))
    except OSError:
        return None
    if "ACKERBOT_STEER_X" not in vals or "ACKERBOT_STEER_MAX_RAD" not in vals:
        return None
    return vals["ACKERBOT_STEER_X"], vals["ACKERBOT_STEER_MAX_RAD"]


class EtatDirection(Node):
    def __init__(self, x_s, delta_max):
        super().__init__("steer_state")
        self.x_s = x_s
        self.delta_max = delta_max
        self.delta = 0.0
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_timer(PERIODE, self.publier)
        self.get_logger().info(
            "roue directrice animee : x_s=%+.3f m (%s), butee %.1f deg"
            % (x_s, "derriere l'essieu" if x_s < 0 else "devant", math.degrees(delta_max)))

    def on_odom(self, msg):
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        if abs(v) < V_EPS:
            return                      # a l'arret : le servo tient sa position
        d = math.atan(w * self.x_s / v)
        self.delta = max(-self.delta_max, min(self.delta_max, d))

    def publier(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = [JOINT]
        m.position = [self.delta]
        self.pub.publish(m)


def main():
    config = os.environ.get("MOWBOT_CONFIG", os.path.expanduser("~/mowbot/config"))
    geo = lire_geometrie(os.path.join(config, "ackerbot_geometry.env"))
    rclpy.init()
    if geo is None:
        # On publie quand meme le joint a ZERO : sans /joint_states,
        # robot_state_publisher n'emet pas du tout la TF de la roue et elle
        # disparait du modele. Un angle faux serait pire qu'un angle nul, donc
        # on ne devine pas la geometrie -- on le dit et on reste a zero.
        n = Node("steer_state")
        n.get_logger().warn(
            "ackerbot_geometry.env absent ou incomplet dans %s : roue directrice "
            "publiee a ZERO (elle ne tournera pas). Regenerer avec "
            "bin/gen_ackerbot_geometry.py puis redeployer." % config)
        pub = n.create_publisher(JointState, "/joint_states", 10)

        def fige():
            m = JointState()
            m.header.stamp = n.get_clock().now().to_msg()
            m.name = [JOINT]
            m.position = [0.0]
            pub.publish(m)

        n.create_timer(PERIODE, fige)
        try:
            rclpy.spin(n)
        except KeyboardInterrupt:
            pass
        rclpy.shutdown()
        return
    n = EtatDirection(*geo)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
