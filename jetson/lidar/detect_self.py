#!/usr/bin/env python3
"""Detecte les parties du ROBOT vues par le lidar.

Principe : sur ~5 s (robot immobile), un point du chassis donne toujours la
MEME distance (variance ~0) et est proche (<0.6 m). Un vrai obstacle exterieur
varie legerement (bruit) et est generalement plus loin.
Sortie : les secteurs angulaires a masquer.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

DUR = 6.0
MAX_SELF = 0.60      # m : au-dela on considere que c'est l'environnement
VAR_MAX = 0.0004     # m^2 : variance tres faible = structure fixe


class Detect(Node):
    def __init__(self):
        super().__init__('detect_self')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LaserScan, '/scan', self.cb, qos)
        self.buf = []
        self.amin = self.ainc = None

    def cb(self, m):
        self.amin, self.ainc = m.angle_min, m.angle_increment
        self.buf.append(np.array(m.ranges, dtype=float))


def main():
    import time
    rclpy.init()
    n = Detect()
    print(f">> analyse {DUR:.0f} s — NE PAS BOUGER le robot ni passer devant", flush=True)
    t0 = time.time()
    while time.time() - t0 < DUR and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
    if len(n.buf) < 5:
        print("pas assez de scans"); return
    a = np.vstack(n.buf)
    a[~np.isfinite(a)] = 99.0
    mean, var = a.mean(axis=0), a.var(axis=0)
    self_mask = (mean < MAX_SELF) & (var < VAR_MAX)
    idx = np.where(self_mask)[0]
    print(f"{len(idx)} rayons sur {a.shape[1]} voient une structure fixe proche", flush=True)
    if len(idx) == 0:
        print(">> aucune partie du robot detectee"); return
    # regrouper en secteurs contigus
    groups, start, prev = [], idx[0], idx[0]
    for i in idx[1:]:
        if i - prev > 3:
            groups.append((start, prev)); start = i
        prev = i
    groups.append((start, prev))
    print("\n== SECTEURS A MASQUER (angles en degres, 0 = avant du lidar) ==")
    for g0, g1 in groups:
        d0 = math.degrees(n.amin + g0 * n.ainc)
        d1 = math.degrees(n.amin + g1 * n.ainc)
        dist = mean[g0:g1 + 1].mean()
        print(f"  de {d0:+7.1f} a {d1:+7.1f} deg   distance {dist*100:5.1f} cm")
    n.destroy_node(); rclpy.shutdown()


main()
