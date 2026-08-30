#!/usr/bin/env python3
"""Trouve la POSITION du lidar qui colle a la carte : balayage de x et du cap.

    python3 lidar_offset.py

loc_check.py balaie les ROTATIONS et repond a la question "le boitier est-il
monte a l'envers ?". Il ne dit RIEN du decalage longitudinal : un lidar place
10 cm devant au lieu de 10 cm derriere garde le meme cap, et la rotation
optimale reste 0. Ce script balaie donc x ET le cap ensemble.

Pourquoi ca compte : les deux robots du projet n'ont pas le meme montage --
+0.10 sur l'un, -0.10 sur l'autre. Se tromper de signe decale tout le scan de
20 cm, ce qui ne se voit pas en ligne droite mais fait deriver la carte des que
le robot tourne, et fait passer les obstacles du mauvais cote du chassis.
"""
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

OCC_MIN = 50
TOL = 2          # cellules de tolerance
N_SCANS = 4


class Offset(Node):
    def __init__(self):
        super().__init__('lidar_offset')
        self.grid = None
        self.scans = []
        qm = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, qm)
        self.create_subscription(LaserScan, '/scan', self.cb_scan,
                                 qos_profile_sensor_data)
        self.buf = Buffer()
        TransformListener(self.buf, self)

    def cb_map(self, m):
        self.grid = m

    def cb_scan(self, m):
        if len(self.scans) < N_SCANS:
            self.scans.append(m)

    def pose_base(self):
        """Pose de base_link dans map : (x, y, yaw)."""
        for _ in range(60):
            try:
                t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
                q = t.transform.rotation
                return (t.transform.translation.x, t.transform.translation.y,
                        math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.2)
        return None

    def score(self, scan, pose, lx, lyaw):
        """Accord si le lidar etait a (lx, 0) avec un cap lyaw dans base_link."""
        g = self.grid
        res, w, h = g.info.resolution, g.info.width, g.info.height
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        occ = np.asarray(g.data, dtype=np.int16).reshape(h, w) >= OCC_MIN

        n = len(scan.ranges)
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + np.arange(n) * scan.angle_increment
        ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        if not ok.any():
            return 0.0
        r, a = r[ok], a[ok]

        bx, by, byaw = pose
        # point vu, dans base_link : rotation du cap du lidar, puis translation
        px = lx + r * np.cos(a + lyaw)
        py = 0.0 + r * np.sin(a + lyaw)
        # puis dans map
        c, s = math.cos(byaw), math.sin(byaw)
        xs = bx + c * px - s * py
        ys = by + s * px + c * py

        cx = np.floor((xs - ox) / res).astype(int)
        cy = np.floor((ys - oy) / res).astype(int)
        ins = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        if not ins.any():
            return 0.0
        cx, cy = cx[ins], cy[ins]
        hit = np.zeros(len(cx), dtype=bool)
        for dx in range(-TOL, TOL + 1):
            for dy in range(-TOL, TOL + 1):
                nx, ny = cx + dx, cy + dy
                m = (nx >= 0) & (nx < w) & (ny >= 0) & (ny < h)
                hit[m] |= occ[ny[m], nx[m]]
        return float(hit.mean())

    def run(self):
        t = 0.0
        while t < 20 and (self.grid is None or len(self.scans) < N_SCANS):
            rclpy.spin_once(self, timeout_sec=0.2)
            t += 0.2
        if self.grid is None or not self.scans:
            print('  pas de carte ou pas de scan')
            return 1
        pose = self.pose_base()
        if pose is None:
            print('  TF map->base_link indisponible')
            return 1
        print(f'  carte {self.grid.info.width}x{self.grid.info.height}, '
              f'{int((np.asarray(self.grid.data) >= OCC_MIN).sum())} cellules occupees')
        print(f'  base_link dans map : x={pose[0]:+.2f} y={pose[1]:+.2f} '
              f'cap={pose[2]*180/math.pi:+.0f} deg')
        print()
        res = []
        for lyaw_deg in (180, 0):
            for lx in (-0.25, -0.20, -0.15, -0.10, -0.05, 0.0,
                       0.05, 0.10, 0.15, 0.20, 0.25):
                v = float(np.mean([self.score(s, pose, lx,
                                              math.radians(lyaw_deg))
                                   for s in self.scans]))
                res.append((v, lx, lyaw_deg))
        res.sort(reverse=True)
        print('  les 8 meilleures combinaisons :')
        print('    %-8s %-8s %s' % ('x', 'cap', 'accord'))
        for v, lx, ly in res[:8]:
            marque = '   <-- CONFIGURATION ACTUELLE' if (abs(lx + 0.10) < 1e-6
                                                         and ly == 180) else ''
            print(f'    {lx:+.2f} m   {ly:3d} deg   {100*v:5.1f} %{marque}')
        act = [v for v, lx, ly in res if abs(lx + 0.10) < 1e-6 and ly == 180]
        best_v, best_x, best_y = res[0]
        print()
        print('  verdict :')
        if act and abs(best_x + 0.10) < 1e-6 and best_y == 180:
            print(f'    x=-0.10 / cap 180 est l\'optimum ({100*best_v:.1f} %) :')
            print('    la configuration actuelle est la bonne.')
        else:
            print(f'    optimum a x={best_x:+.2f} m / cap {best_y} deg '
                  f'({100*best_v:.1f} %)')
            if act:
                print(f'    contre {100*act[0]:.1f} % pour la config actuelle '
                      f'(x=-0.10 / 180 deg).')
            print('    -> corriger MOWBOT_LIDAR_X (et MOWBOT_LIDAR_YAW) dans')
            print('       l\'environnement de mowbot-tf, puis relancer le service.')
        print()
        print('  ATTENTION a l\'interpretation : ce test compare le scan a une')
        print('  carte CONSTRUITE avec la configuration actuelle. Si celle-ci est')
        print('  fausse, la carte l\'est aussi, et l\'erreur peut se confirmer')
        print('  elle-meme. Le seul juge definitif reste un objet pose au')
        print('  CONTACT de l\'avant du robot, sur une carte neuve.')
        return 0


def main():
    rclpy.init()
    n = Offset()
    try:
        return n.run()
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
