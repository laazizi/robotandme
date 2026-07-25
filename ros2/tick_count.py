#!/usr/bin/env python3
"""Compte les TICKS PAR TOUR DE ROUE (pour calibrer TICKS_PER_WHEEL_REV).

/odom ne publie pas les compteurs bruts : on les reconstitue depuis la pose.
  d_centre = distance parcourue,  d_theta = rotation
  d_gauche = d_centre - d_theta*TRACK/2      d_droite = d_centre + d_theta*TRACK/2
  ticks = d_roue / METERS_PER_TICK   (avec les constantes ACTUELLES du firmware)

MODE D'EMPLOI
  1. Marque un repere sur UNE roue (scotch) et un repere en face sur le chassis.
  2. Lance le script : il remet les compteurs a zero.
  3. Fais faire a cette roue EXACTEMENT UN TOUR (360 deg), repere sur repere.
     - a la main si le reducteur le permet
     - sinon pousse le robot au sol tout droit : les DEUX roues tournent, et
       n'importe laquelle des deux colonnes donne la reponse.
  4. Lis la valeur affichee pour cette roue : c'est TICKS_PER_WHEEL_REV reel.

Lancement :
    source /opt/ros/humble/setup.bash   # (ou jazzy sur le PC)
    export ROS_DOMAIN_ID=0
    python3 tick_count.py [duree_s]     # defaut 60 s
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

# --- constantes ACTUELLES du firmware (main/config.h) ---
WHEEL_RADIUS_M      = 0.0698
TICKS_PER_WHEEL_REV = 2560.0
TRACK_WIDTH_M       = 0.59

METERS_PER_TICK = 2.0 * math.pi * WHEEL_RADIUS_M / TICKS_PER_WHEEL_REV
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


class TickCount(Node):
    def __init__(self):
        super().__init__('tick_count')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, '/odom', self.cb, qos)
        self.prev = None          # (x, y, yaw)
        self.d_left = 0.0         # distances cumulees [m]
        self.d_right = 0.0
        self.last_print = 0.0

    def cb(self, m):
        x = m.pose.pose.position.x
        y = m.pose.pose.position.y
        th = yaw_of(m.pose.pose.orientation)
        if self.prev is not None:
            px, py, pth = self.prev
            dth = norm(th - pth)
            # signe du deplacement : projete sur le cap courant
            step = math.hypot(x - px, y - py)
            if step > 0 and math.cos(math.atan2(y - py, x - px) - th) < 0:
                step = -step
            self.d_left += step - dth * TRACK_WIDTH_M / 2.0
            self.d_right += step + dth * TRACK_WIDTH_M / 2.0
        self.prev = (x, y, th)

        now = time.time()
        if now - self.last_print >= 0.5:
            self.last_print = now
            tl = self.d_left / METERS_PER_TICK
            tr = self.d_right / METERS_PER_TICK
            print(f"  ticks GAUCHE = {tl:+9.0f}    ticks DROITE = {tr:+9.0f}", flush=True)


def main():
    rclpy.init()
    n = TickCount()
    print("=" * 62)
    print(" COMPTEUR DE TICKS — remis a zero")
    print(" Fais faire UN TOUR COMPLET (360 deg) a une roue,")
    print(" puis lis la valeur de cette roue = TICKS_PER_WHEEL_REV reel.")
    print("=" * 62, flush=True)
    t0 = time.time()
    try:
        while time.time() - t0 < DUR and rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    tl = n.d_left / METERS_PER_TICK
    tr = n.d_right / METERS_PER_TICK
    print("=" * 62)
    print(f" TOTAL  gauche = {tl:+.0f} ticks   droite = {tr:+.0f} ticks")
    print(" Si la roue a fait exactement 1 tour -> c'est la valeur a mettre")
    print(" dans TICKS_PER_WHEEL_REV (main/config.h).")
    print("=" * 62)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
