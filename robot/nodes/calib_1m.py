#!/usr/bin/env python3
"""Calibration distance sur 1 metre (boucle fermee sur /odom brut).

Le robot avance jusqu'a ce que SON odometrie dise 1.00 m, puis s'arrete.
Tu mesures alors la distance REELLE parcourue au metre-ruban :

    WHEEL_RADIUS_M nouveau = WHEEL_RADIUS_M actuel x (reel / 1.00)

  - odometrie juste  -> le robot s'arrete pile a 1 m reel
  - reel > 1 m       -> rayon configure trop petit (augmenter)
  - reel < 1 m       -> rayon configure trop grand (diminuer)

PREPARATION : robot AU SOL, 1.5 m degages devant, marque au sol au niveau
de l'axe des roues. Aucun autre pilote actif (joystick au repos = OK).

Lancement (Jetson) :  bash ~/run_calib_1m.sh   [vitesse]   (defaut 0.3 m/s)
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

TARGET  = 1.00                                     # distance odometrie visee [m]
SPEED   = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
TIMEOUT = 25.0
RATE_HZ = 20.0


class Calib1m(Node):
    def __init__(self):
        super().__init__('calib_1m')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        # /odom BRUT (roues seules) : c'est lui qu'on calibre, pas l'EKF.
        self.create_subscription(Odometry, '/odom', self.cb, qos)
        self.x = self.y = None

    def cb(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y

    def cmd(self, v):
        t = Twist()
        t.linear.x = float(v)
        self.pub.publish(t)

    def run(self):
        print('attente /odom...', flush=True)
        while rclpy.ok() and self.x is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        x0, y0 = self.x, self.y
        print(f'>> AVANCE a {SPEED} m/s jusqu\'a 1.00 m (odometrie)...', flush=True)
        t0 = time.time()
        moved = 0.0
        last = -1.0
        while rclpy.ok() and moved < TARGET and time.time() - t0 < TIMEOUT:
            self.cmd(SPEED)
            rclpy.spin_once(self, timeout_sec=1.0 / RATE_HZ)
            moved = math.hypot(self.x - x0, self.y - y0)
            if moved - last >= 0.10:                     # affichage tous les 10 cm
                last = moved
                print(f'   {moved*100:5.1f} cm', flush=True)
        # arret franc
        for _ in range(8):
            self.cmd(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
        print('=' * 56)
        print(f'  ODOMETRIE : {moved*100:.1f} cm')
        print('  Mesure la distance REELLE au metre-ruban, puis :')
        print('  rayon_nouveau = rayon_actuel x (reel_cm / '
              f'{moved*100:.1f})')
        print('=' * 56, flush=True)


def main():
    rclpy.init()
    n = Calib1m()
    try:
        n.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for _ in range(5):
                n.cmd(0.0)
                rclpy.spin_once(n, timeout_sec=0.05)
        except Exception:
            pass
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
