#!/usr/bin/env python3
"""Arene de test virtuelle pour la navigation : carre de 1.5 m avec un
cylindre de 30 cm de diametre au centre, publie en markers RViz (frame odom).

Par defaut l'arene est centree en (0.75, 0) : le robot qui demarre en (0,0)
se trouve au milieu du bord ouest, le cylindre est a 75 cm devant lui.

ATTENTION : purement VISUEL — goto_goal ne connait pas ces obstacles.
Placez un vrai objet au sol a l'endroit du cylindre pour les tests !

Usage (Jetson ou PC) :
    python3 test_arena.py [cx] [cy]      # centre de l'arene (defaut 0.75 0)
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray

SIZE   = 2.0      # cote du carre [m]
WALL_H = 0.20     # hauteur visuelle des murs
WALL_T = 0.03     # epaisseur des murs
CYL_D  = 0.20     # diametre du cylindre central
CYL_H  = 0.35     # hauteur du cylindre

CX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.75
CY = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0


def wall(mid, x, y, lx, ly):
    m = Marker()
    m.header.frame_id = 'map'
    m.ns = 'arena'
    m.id = mid
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = WALL_H / 2
    m.pose.orientation.w = 1.0
    m.scale.x = lx
    m.scale.y = ly
    m.scale.z = WALL_H
    m.color.r, m.color.g, m.color.b, m.color.a = 0.85, 0.85, 0.9, 0.85
    return m


class Arena(Node):
    def __init__(self):
        super().__init__('test_arena')
        qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(MarkerArray, '/test_arena', qos)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(
            f'arene 1.5 m centree en ({CX}, {CY}), cylindre Ø{CYL_D*100:.0f} cm au centre')

    def publish(self):
        h = SIZE / 2
        ma = MarkerArray()
        # 4 murs
        ma.markers.append(wall(1, CX, CY + h, SIZE + WALL_T, WALL_T))   # nord
        ma.markers.append(wall(2, CX, CY - h, SIZE + WALL_T, WALL_T))   # sud
        ma.markers.append(wall(3, CX + h, CY, WALL_T, SIZE + WALL_T))   # est
        ma.markers.append(wall(4, CX - h, CY, WALL_T, SIZE + WALL_T))   # ouest
        # cylindre central (obstacle)
        c = Marker()
        c.header.frame_id = 'map'
        c.ns = 'arena'
        c.id = 10
        c.type = Marker.CYLINDER
        c.action = Marker.ADD
        c.pose.position.x = CX
        c.pose.position.y = CY
        c.pose.position.z = CYL_H / 2
        c.pose.orientation.w = 1.0
        c.scale.x = CYL_D
        c.scale.y = CYL_D
        c.scale.z = CYL_H
        c.color.r, c.color.g, c.color.b, c.color.a = 0.95, 0.55, 0.15, 0.95
        ma.markers.append(c)
        self.pub.publish(ma)


def main():
    rclpy.init()
    n = Arena()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
