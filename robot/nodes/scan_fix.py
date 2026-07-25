#!/usr/bin/env python3
"""Normalise et NETTOIE /scan_raw -> /scan.

1) Nombre de points FIXE : le LSLidar N10 sort 449/450/451 rayons selon les
   tours ; slam_toolbox enregistre la taille du 1er scan et rejette les autres
   ("contains 451 readings, expected 449"). On reechantillonne a N points.

2) Masquage des parties du ROBOT : certains montants/antennes sont dans le
   champ du lidar (detectes par detect_self.py : echos a distance constante).
   Sans filtrage, la costmap voit le robot encercle d'obstacles -> nav2 refuse
   de planifier. Les rayons concernes sont mis a l'infini (= rien vu).
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

N = 450

# Secteurs occultes : (angle_debut_deg, angle_fin_deg, distance_max_m).
# Seuls les echos PLUS PROCHES que la distance indiquee sont supprimes :
# au-dela, dans le meme secteur, ce sont de vrais obstacles -> conserves.
SELF_SECTORS = [
    (233.0, 250.0, 0.45),   # montant arriere-droit (~33 cm)
    (261.0, 280.0, 0.60),   # antenne / support (~48 cm)
    (282.0, 311.0, 0.40),   # montant arriere-gauche (~28 cm)
    (52.0,  64.0,  0.55),   # structure avant-gauche (~49 cm, detectee ensuite)
    (250.0, 262.0, 0.65),   # structure arriere (~60 cm, detectee ensuite)
]


class ScanFix(Node):
    def __init__(self):
        super().__init__('scan_fix')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub = self.create_publisher(LaserScan, '/scan', qos)
        self.create_subscription(LaserScan, '/scan_raw', self.cb, qos)
        self.mask = None
        self.get_logger().info(
            f'/scan_raw -> /scan : {N} points, {len(SELF_SECTORS)} secteurs robot masques')

    def build_mask(self, amin, amax):
        ang = np.degrees(np.linspace(amin, amax, N)) % 360.0
        lim = np.full(N, -1.0)
        for a0, a1, dmax in SELF_SECTORS:
            sel = (ang >= a0) & (ang <= a1) if a0 <= a1 else (ang >= a0) | (ang <= a1)
            lim[sel] = np.maximum(lim[sel], dmax)
        return lim

    def cb(self, m):
        n_in = len(m.ranges)
        if n_in < 2:
            return
        if self.mask is None:
            self.mask = self.build_mask(m.angle_min, m.angle_max)

        src = np.linspace(m.angle_min, m.angle_max, n_in)
        dst = np.linspace(m.angle_min, m.angle_max, N)
        r = np.array(m.ranges, dtype=float)
        r[~np.isfinite(r)] = m.range_max + 1.0
        out_r = np.interp(dst, src, r)

        # masquage : echo proche dans un secteur "robot" -> considere inexistant
        hide = (self.mask > 0) & (out_r < self.mask)
        out_r[hide] = float('inf')

        out = LaserScan()
        out.header = m.header
        out.angle_min = m.angle_min
        out.angle_max = m.angle_max
        out.angle_increment = (m.angle_max - m.angle_min) / (N - 1)
        out.time_increment = m.time_increment
        out.scan_time = m.scan_time
        out.range_min = m.range_min
        out.range_max = m.range_max
        out.ranges = out_r.astype(np.float32).tolist()
        self.pub.publish(out)


def main():
    rclpy.init()
    n = ScanFix()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
