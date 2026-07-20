#!/usr/bin/env python3
"""Integre angular_velocity.z de /imu/data_raw pendant N secondes.
Tournez le robot d'un angle CONNU pendant l'enregistrement, puis comparez
(echelle = angle_reel / valeur affichee). Sert a valider le gyro Razor.

Usage : python3 gyro_calib.py [duree_s]   (defaut 15)
"""
import math
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

DUR = int(sys.argv[1]) if len(sys.argv) > 1 else 15


class Calib(Node):
    def __init__(self):
        super().__init__("gyro_calib")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Imu, "/imu/data_raw", self.cb, qos)
        self.last = None
        self.yaw = 0.0
        self.n = 0

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last is not None:
            dt = t - self.last
            if 0 < dt < 0.5:
                self.yaw += msg.angular_velocity.z * dt
        self.last = t
        self.n += 1


def main():
    rclpy.init()
    node = Calib()
    print(f">>> Tournez le robot d'un angle CONNU (ex: 360 deg) MAINTENANT — {DUR}s")
    t0 = time.time()
    while time.time() - t0 < DUR and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f">>> Integrale gyro Z = {math.degrees(node.yaw):.1f} deg  ({node.n} msgs)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
