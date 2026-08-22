#!/usr/bin/env python3
"""Mesure l'accord entre le scan courant et la carte -- diagnostic de localisation.

Principe : on projette chaque point du scan dans le repere `map` via la TF, puis
on regarde s'il tombe sur une cellule OCCUPEE. Un scan bien localise pose ses
points sur les murs : le taux d'accord est eleve. S'il est bas, on ne sait pas
encore POURQUOI -- d'ou le balayage : on rejoue le calcul en appliquant au scan
une rotation supplementaire (et eventuellement un effet miroir), et on regarde
quelle correction maximise l'accord.

    accord maximal a +0 deg, sans miroir   -> la geometrie est bonne, la
                                              localisation a simplement derive
    accord maximal a +180 deg              -> lidar monte a l'envers, ou TF
                                              laser_link fausse de 180 deg
    accord maximal AVEC miroir             -> sens de rotation du scan inverse
                                              (parametre invert_angle /
                                              laser_scan_dir du driver)

Ce dernier cas est le piege du LD14 : le capteur fait croitre son angle dans le
sens horaire, ROS attend l'inverse. Le noeud Python corrige avec
`invert_angle: true`, le driver natif avec `laser_scan_dir:=true` -- deux
mecanismes distincts qu'il faut verifier separement, d'autant que le natif
annonce ses angles sur 0..360 deg et non -180..+180.

    python3 loc_check.py
"""
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

OCC_MIN = 50        # valeur a partir de laquelle une cellule compte pour occupee
TOL_CELLS = 2       # un point compte comme "sur le mur" s'il est a <= 2 cellules
N_SCANS = 5         # on moyenne plusieurs scans : un seul peut etre atypique


class LocCheck(Node):
    def __init__(self):
        super().__init__('loc_check')
        self.grid = None
        self.scans = []
        qmap = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, qmap)
        self.create_subscription(LaserScan, '/scan', self.cb_scan,
                                 qos_profile_sensor_data)
        self.buf = Buffer()
        TransformListener(self.buf, self)

    def cb_map(self, m):
        self.grid = m

    def cb_scan(self, m):
        if len(self.scans) < N_SCANS:
            self.scans.append(m)

    # -- utilitaires ---------------------------------------------------------
    def laser_pose(self, frame):
        """Pose du lidar dans `map` : (x, y, yaw). None si la TF manque.

        Boucle de reessai indispensable : au premier appel le buffer ne contient
        que quelques millisecondes de donnees et tf2 refuse avec "would require
        extrapolation into the past" -- la chaine map->odom->base_link->laser
        n'a pas encore d'instant commun. On laisse le buffer se remplir.
        """
        t = None
        for _ in range(50):
            try:
                t = self.buf.lookup_transform('map', frame, rclpy.time.Time())
                break
            except Exception as err:
                last = err
                rclpy.spin_once(self, timeout_sec=0.2)
        if t is None:
            self.get_logger().error(f'TF map->{frame} indisponible : {last}')
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def score(self, scan, pose, dyaw, mirror):
        """Fraction des points du scan qui tombent sur une cellule occupee."""
        g = self.grid
        res, w, h = g.info.resolution, g.info.width, g.info.height
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        data = np.asarray(g.data, dtype=np.int16).reshape(h, w)
        occ = data >= OCC_MIN

        n = len(scan.ranges)
        r = np.asarray(scan.ranges, dtype=float)
        ang = scan.angle_min + np.arange(n) * scan.angle_increment
        if mirror:
            ang = -ang
        good = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        if not good.any():
            return 0.0, 0
        r, ang = r[good], ang[good]

        px, py, yaw = pose
        a = yaw + dyaw + ang
        xs = px + r * np.cos(a)
        ys = py + r * np.sin(a)
        cx = np.floor((xs - ox) / res).astype(int)
        cy = np.floor((ys - oy) / res).astype(int)
        inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        if not inside.any():
            return 0.0, 0
        cx, cy = cx[inside], cy[inside]

        # tolerance : on accepte une cellule occupee dans un petit voisinage,
        # sinon une erreur de localisation de 3 cm suffirait a tout rejeter.
        hit = np.zeros(len(cx), dtype=bool)
        for dx in range(-TOL_CELLS, TOL_CELLS + 1):
            for dy in range(-TOL_CELLS, TOL_CELLS + 1):
                nx, ny = cx + dx, cy + dy
                ok = (nx >= 0) & (nx < w) & (ny >= 0) & (ny < h)
                hit[ok] |= occ[ny[ok], nx[ok]]
        return float(hit.mean()), int(len(cx))

    def run(self):
        # attendre carte + scans
        t = 0.0
        while t < 20.0 and (self.grid is None or len(self.scans) < N_SCANS):
            rclpy.spin_once(self, timeout_sec=0.2)
            t += 0.2
        if self.grid is None:
            print('AUCUNE carte sur /map -- rien a comparer.')
            return 1
        if not self.scans:
            print('AUCUN scan sur /scan.')
            return 1

        g, s0 = self.grid, self.scans[0]
        print(f'carte  : {g.info.width}x{g.info.height} a {g.info.resolution:.3f} m/px, '
              f'origine ({g.info.origin.position.x:.2f}, {g.info.origin.position.y:.2f})')
        occ_n = int((np.asarray(g.data) >= OCC_MIN).sum())
        print(f'         {occ_n} cellules occupees')
        print(f'scan   : {len(s0.ranges)} pts, '
              f'{math.degrees(s0.angle_min):.0f}..{math.degrees(s0.angle_max):.0f} deg, '
              f'increment {math.degrees(s0.angle_increment):.3f} deg, repere {s0.header.frame_id}')

        pose = self.laser_pose(s0.header.frame_id)
        if pose is None:
            return 1
        print(f'lidar dans map : x={pose[0]:.2f} y={pose[1]:.2f} '
              f'cap={math.degrees(pose[2]):.1f} deg')

        # --- balayage rotation x miroir ------------------------------------
        print('\naccord scan/carte (moyenne sur %d scans) :' % len(self.scans))
        print('  %-8s %-9s %s' % ('miroir', 'rotation', 'accord'))
        results = []
        for mirror in (False, True):
            for dyaw_deg in range(-180, 180, 5):
                dyaw = math.radians(dyaw_deg)
                sc = [self.score(s, pose, dyaw, mirror) for s in self.scans]
                val = float(np.mean([x[0] for x in sc]))
                results.append((val, mirror, dyaw_deg))
        results.sort(reverse=True)

        # configuration actuelle, pour reference
        cur = float(np.mean([self.score(s, pose, 0.0, False)[0] for s in self.scans]))
        print('  %-8s %-9s %.1f %%   <-- configuration ACTUELLE'
              % ('non', '+0 deg', 100 * cur))
        for val, mirror, d in results[:5]:
            mark = ''
            if (mirror, d) == (False, 0):
                mark = '   (= config actuelle)'
            print('  %-8s %-9s %.1f %%%s'
                  % ('OUI' if mirror else 'non', f'{d:+d} deg', 100 * val, mark))

        best_val, best_mir, best_d = results[0]
        print('\nverdict :')
        if best_val < 0.30:
            print(f'  accord maximal seulement {100*best_val:.0f} % : AUCUNE correction de')
            print('  geometrie ne recolle le scan a la carte. La carte elle-meme est')
            print('  probablement incoherente (construite avec une TF fausse) --')
            print('  la reconstruire : mowbot new-map')
        elif best_mir or abs(best_d) > 10:
            print(f'  le scan colle a {100*best_val:.0f} % en appliquant '
                  f'{"un MIROIR et " if best_mir else ""}{best_d:+d} deg.')
            print(f'  Contre {100*cur:.0f} % dans la configuration actuelle.')
            print('  => erreur de GEOMETRIE, pas de derive : corriger le sens/l\'angle du')
            print('     lidar (run_tf.sh MOWBOT_LIDAR_YAW, ou invert_angle /')
            print('     laser_scan_dir du driver).')
        else:
            print(f'  meilleur accord a {best_d:+d} deg sans miroir ({100*best_val:.0f} %) :')
            print('  la geometrie est BONNE. Un accord faible vient alors d\'une derive de')
            print('  localisation ou d\'une carte deja polluee, pas du montage du lidar.')
        return 0


def main():
    rclpy.init()
    n = LocCheck()
    try:
        rc = n.run()
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
