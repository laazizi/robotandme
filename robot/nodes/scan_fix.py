#!/usr/bin/env python3
"""Normalise et NETTOIE /scan_raw -> /scan.

1) Nombre de points FIXE : le LSLidar N10 sort 449/450/451 rayons selon les
   tours ; slam_toolbox enregistre la taille du 1er scan et rejette les autres
   ("contains 451 readings, expected 449"). On reechantillonne a N points.

2) Masquage des parties du ROBOT : certains montants/antennes sont dans le
   champ du lidar (detectes par detect_self.py : echos a distance constante).
   Sans filtrage, la costmap voit le robot encercle d'obstacles -> nav2 refuse
   de planifier. Les rayons concernes sont mis a l'infini (= rien vu).

3) Suppression des points ISOLES (faux echos). Mesure sur ce robot :
   14.7 % des points n'avaient aucun voisin a moins de 15 cm, et 31 % des
   directions variaient de plus de 10 cm d'un tour a l'autre. Deux causes :
     - effet de bord : quand le faisceau frole l'arete d'un objet, une partie
       du spot touche l'objet et l'autre le fond ; la distance retournee tombe
       ENTRE les deux -> un point fantome apparait derriere l'obstacle reel ;
     - reflexions speculaires sur le parquet (lidar a triangulation).
   Une vraie surface est continue : ses points ont des voisins a distance
   comparable. Un faux echo est seul. On exige donc au moins un voisin
   coherent dans une fenetre de +/-2 rayons -- pas deux, sinon un pied de
   chaise fin (1 a 2 points a 2 m) serait efface avec le bruit.
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

# Filtre des points isoles (cf. point 3 du docstring).
ISO_ENABLE = True
ISO_WINDOW = 2      # rayons voisins examines de chaque cote
ISO_TOL_M  = 0.15   # ecart de distance en deca duquel un voisin est "coherent"
ISO_MIN_NB = 1      # nombre de voisins coherents exiges pour garder le point


class ScanFix(Node):
    def __init__(self):
        super().__init__('scan_fix')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub = self.create_publisher(LaserScan, '/scan', qos)
        self.create_subscription(LaserScan, '/scan_raw', self.cb, qos)
        self.mask = None
        self.n_dropped = 0
        self.n_seen = 0
        self.get_logger().info(
            f'/scan_raw -> /scan : {N} points, {len(SELF_SECTORS)} secteurs robot masques'
            f'{", filtre points isoles actif" if ISO_ENABLE else ""}')
        if ISO_ENABLE:
            # un point de mesure toutes les 30 s : verifie que le filtre retire
            # bien du bruit, et pas la moitie du scan (reglage trop agressif).
            self.create_timer(30.0, self.report)

    def report(self):
        if self.n_seen:
            self.get_logger().info(
                f'points isolés filtrés : {100.0 * self.n_dropped / self.n_seen:.1f} %')
        self.n_dropped = self.n_seen = 0

    def build_mask(self, amin, amax):
        ang = np.degrees(np.linspace(amin, amax, N)) % 360.0
        lim = np.full(N, -1.0)
        for a0, a1, dmax in SELF_SECTORS:
            sel = (ang >= a0) & (ang <= a1) if a0 <= a1 else (ang >= a0) | (ang <= a1)
            lim[sel] = np.maximum(lim[sel], dmax)
        return lim

    def drop_isolated(self, r, range_max):
        """Efface les echos sans voisin coherent (faux echos de bord/reflet).

        Le scan est circulaire : np.roll relie naturellement le dernier rayon
        au premier, sans cas particulier aux extremites.
        """
        valid = np.isfinite(r) & (r > 0.0) & (r <= range_max)
        neighbours = np.zeros(len(r), dtype=int)
        for k in range(1, ISO_WINDOW + 1):
            for shifted in (np.roll(r, k), np.roll(r, -k)):
                sv = np.isfinite(shifted) & (shifted > 0.0) & (shifted <= range_max)
                neighbours += (sv & (np.abs(shifted - r) < ISO_TOL_M)).astype(int)
        lonely = valid & (neighbours < ISO_MIN_NB)
        r = r.copy()
        r[lonely] = float('inf')
        self.n_dropped += int(lonely.sum())
        self.n_seen += int(valid.sum())
        return r

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

        if ISO_ENABLE:
            out_r = self.drop_isolated(out_r, m.range_max)

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
