#!/usr/bin/env python3
"""Republie /odom (nav_msgs/Odometry) en transformee TF odom -> base_link.

>>> OBSOLETE des que l'EKF tourne sur le robot. <<<

Ecrit avant l'integration de robot_localization, quand rien ne publiait la TF
odom -> base_link. Aujourd'hui c'est l'EKF (service mowbot-ekf) qui la publie,
a partir de la fusion roues + gyro. Lancer ce script EN PLUS donne DEUX
emetteurs pour la meme transformation : la TF oscille entre les deux valeurs,
et les consommateurs rejettent les messages ("Message Filter dropping
message ... queue is full" dans RViz, scans ignores par slam_toolbox).

Le script refuse donc de demarrer si /odometry/filtered est publie.
Il ne reste utile que pour un banc SANS EKF (firmware seul).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        # Best-effort : compatible que le firmware publie en reliable ou non.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, '/odom', self.on_odom, qos)
        self.get_logger().info('odom -> base_link TF broadcaster demarre')

    def on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id or 'odom'
        t.child_frame_id = msg.child_frame_id or 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def ekf_is_running(node, timeout=3.0):
    """Vrai si /odometry/filtered a un editeur : l'EKF publie deja la TF."""
    import time
    t0 = time.time()
    while time.time() - t0 < timeout:
        if node.count_publishers('/odometry/filtered') > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.2)
    return False


def main():
    rclpy.init()
    node = OdomTfBroadcaster()
    if ekf_is_running(node):
        node.get_logger().error(
            "l'EKF publie deja odom -> base_link (/odometry/filtered actif). "
            "Deux emetteurs feraient osciller la TF et RViz rejetterait les "
            "messages. Arret. Forcer : --force")
        import sys
        if '--force' not in sys.argv:
            node.destroy_node()
            rclpy.shutdown()
            return
        node.get_logger().warning('--force : TF dupliquee assumee')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown() peut avoir deja ete appele (Ctrl+C traite par le
        # contexte) -> sans ce garde, on terminait sur une RCLError bruyante.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
