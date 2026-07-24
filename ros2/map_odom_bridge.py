#!/usr/bin/env python3
"""Publie la TF map->odom, recalable par le bouton '2D Pose Estimate' de RViz.

Sans AMCL, map->odom est identite par defaut. Quand RViz publie /initialpose
(clic '2D Pose Estimate'), on recalcule map->odom pour que la pose ODOM
courante du robot corresponde a la pose cliquee sur la CARTE :
    T_map_odom = Pose_carte  x  Pose_odom^-1

Usage : python3 ~/map_odom_bridge.py     (lance par nav2_up.sh)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MapOdomBridge(Node):
    def __init__(self):
        super().__init__('map_odom_bridge')
        self.br = TransformBroadcaster(self)
        # transformee courante map->odom (identite au depart)
        self.tx = 0.0
        self.ty = 0.0
        self.tyaw = 0.0
        # derniere pose odom du robot
        self.ox = self.oy = self.oyaw = 0.0
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.on_initialpose, 10)
        self.create_timer(0.05, self.broadcast)   # 20 Hz
        self.get_logger().info("map->odom identite ; '2D Pose Estimate' pour recaler")

    def on_odom(self, m):
        self.ox = m.pose.pose.position.x
        self.oy = m.pose.pose.position.y
        self.oyaw = yaw_of(m.pose.pose.orientation)

    def on_initialpose(self, m):
        mx = m.pose.pose.position.x
        my = m.pose.pose.position.y
        myaw = yaw_of(m.pose.pose.orientation)
        # T_map_odom tel que T ⊗ pose_odom = pose_carte
        self.tyaw = myaw - self.oyaw
        c, s = math.cos(self.tyaw), math.sin(self.tyaw)
        self.tx = mx - (c * self.ox - s * self.oy)
        self.ty = my - (s * self.ox + c * self.oy)
        self.get_logger().info(
            f'recalage : robot place en ({mx:.2f}, {my:.2f}, {math.degrees(myaw):.0f} deg) sur la carte')

    def broadcast(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self.tx
        t.transform.translation.y = self.ty
        t.transform.rotation.z = math.sin(self.tyaw / 2)
        t.transform.rotation.w = math.cos(self.tyaw / 2)
        self.br.sendTransform(t)


def main():
    rclpy.init()
    n = MapOdomBridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
