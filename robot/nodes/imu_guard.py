#!/usr/bin/env python3
"""Protege l'EKF d'un gyroscope GELE : /imu/data_raw -> /imu/data_checked.

POURQUOI. Le 28/08/2026 sur le robot A, le L3G4200D du GY-801 a cesse de
repondre en cours de fonctionnement. Le firmware a continue de republier sa
DERNIERE lecture : -24.283 deg/s sur l'axe de lacet, valeur figee a la sixieme
decimale sur 291 echantillons consecutifs. L'accelerometre, lui, continuait de
vivre (l'ADXL345 est une autre puce sur le meme bus) -- le probleme touchait le
seul gyroscope.

Consequence : l'EKF integrait 24 deg/s de faux lacet, soit un tour complet
toutes les 15 secondes. Le robot pivotait dans RViz alors qu'il etait pose,
roues a l'arret. Et le diagnostic est trompeur : une valeur constante ressemble
a un biais mal calibre, alors qu'un biais, lui, est BRUITE. C'est l'absence de
bruit qui trahit la panne.

CE QUE FAIT CE NOEUD. Il surveille la variation des trois axes du gyro. Si
aucune ne bouge pendant plus de FREEZE_S secondes, il considere le capteur mort
et publie des vitesses angulaires NULLES plutot que la valeur figee. L'EKF cesse
alors de faire tourner le robot. Des que le capteur redonne des valeurs
differentes, la transmission reprend telle quelle.

CE QU'IL NE FAIT PAS. Il ne repare pas le gyro et ne remplace pas le cap : avec
un gyro mort, l'EKF n'a plus de source de lacet fiable, donc les rotations
reelles ne seront plus mesurees. Le vrai correctif est dans le firmware --
detecter le gel et reinitialiser le L3G4200D. En attendant, un robot qui ignore
les rotations vaut mieux qu'un robot qui en invente.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

# Duree sans AUCUNE variation au-dela de laquelle on declare le capteur gele.
# 1.5 s a 20 Hz = 30 echantillons : largement assez pour trancher, un vrai gyro
# au repos change de valeur a chaque lecture (bruit de quantification).
FREEZE_S = 1.5

# Tolerance : deux lectures dont l'ecart est inferieur a ce seuil comptent comme
# identiques. Pas zero, pour ne pas dependre de l'arrondi des flottants.
EPS = 1e-9


class ImuGuard(Node):
    def __init__(self):
        super().__init__('imu_guard')
        self.pub = self.create_publisher(Imu, '/imu/data_checked',
                                         qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data_raw', self.cb,
                                 qos_profile_sensor_data)
        self.last = None
        self.t_change = None
        self.gele = False
        self.n_gele = 0
        self.n_total = 0
        self.create_timer(30.0, self.rapport)
        self.get_logger().info(
            f'/imu/data_raw -> /imu/data_checked : lacet annule si le gyro reste '
            f'fige plus de {FREEZE_S} s')

    def rapport(self):
        if self.n_total:
            pc = 100.0 * self.n_gele / self.n_total
            if pc > 0:
                self.get_logger().warning(
                    f'gyro gele sur {pc:.0f} % des mesures des 30 derniers s')
        self.n_gele = self.n_total = 0

    def cb(self, m):
        g = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.last is None or any(abs(a - b) > EPS for a, b in zip(g, self.last)):
            self.t_change = now
            if self.gele:
                self.gele = False
                self.get_logger().info('le gyro redonne des valeurs : reprise')
        self.last = g

        if self.t_change is not None and (now - self.t_change) > FREEZE_S:
            if not self.gele:
                self.gele = True
                self.get_logger().error(
                    f'GYRO GELE : {math.degrees(g[2]):+.3f} deg/s inchange depuis '
                    f'{now - self.t_change:.1f} s. Lacet force a zero pour ne pas '
                    f'faire tourner l\'EKF. Verifier le L3G4200D (I2C) ; '
                    f'`mowbot esp32-reset` le remet en marche.')

        out = Imu()
        out.header = m.header
        out.orientation = m.orientation
        out.orientation_covariance = m.orientation_covariance
        out.linear_acceleration = m.linear_acceleration
        out.linear_acceleration_covariance = m.linear_acceleration_covariance
        out.angular_velocity_covariance = m.angular_velocity_covariance
        if self.gele:
            out.angular_velocity.x = 0.0
            out.angular_velocity.y = 0.0
            out.angular_velocity.z = 0.0
            self.n_gele += 1
        else:
            out.angular_velocity = m.angular_velocity
        self.n_total += 1
        self.pub.publish(out)


def main():
    rclpy.init()
    n = ImuGuard()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
