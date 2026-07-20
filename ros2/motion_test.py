#!/usr/bin/env python3
"""Test carre mowbot (boucle fermee sur /odom).

Parcourt un CARRE de 35 cm de cote : avancer 35 cm puis tourner 90 deg,
a chaque coin (4 cotes). Le carre complet est repete 5 fois.

Petite vitesse. Utilise /odom pour mesurer distance et angle reels.
Ctrl+C : arret d'urgence (envoie une consigne nulle avant de quitter).

Prerequis : agent micro-ROS actif sur le Jetson, memes ROS_DOMAIN_ID.
Lancement :
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0
    python3 motion_test.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ---- Parametres (modifiables) ----
LINEAR_SPEED  = 0.08                  # m/s (petite vitesse)
ANGULAR_SPEED = 0.5                   # rad/s
SIDE          = 0.35                  # cote du carre : 35 cm
CORNER        = math.radians(90.0)    # rotation a chaque coin : 90 deg
SQUARES       = 5                     # nombre de carres
PAUSE         = 0.5                   # s entre chaque mouvement
RATE_HZ       = 20.0                  # publication (> deadman firmware 500 ms)
MOVE_TIMEOUT  = 20.0                  # securite : abandon d'un mouvement trop long


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class MotionTest(Node):
    def __init__(self):
        super().__init__('motion_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos)
        self.x = self.y = self.yaw = None

    def _on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _spin(self, dt):
        rclpy.spin_once(self, timeout_sec=dt)

    def wait_odom(self):
        self.get_logger().info('attente de /odom...')
        while rclpy.ok() and self.x is None:
            self._spin(0.1)
        self.get_logger().info('/odom recu.')

    def _publish(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def stop(self):
        end = time.time() + 0.3
        while time.time() < end:
            self._publish(0.0, 0.0)
            self._spin(1.0 / RATE_HZ)

    def drive(self, dist, label=''):
        """Avance (dist>0) ou recule (dist<0) de |dist| metres."""
        x0, y0 = self.x, self.y
        v = LINEAR_SPEED if dist >= 0 else -LINEAR_SPEED
        target = abs(dist)
        t_end = time.time() + MOVE_TIMEOUT
        moved = 0.0
        while rclpy.ok() and moved < target and time.time() < t_end:
            self._publish(v, 0.0)
            self._spin(1.0 / RATE_HZ)
            moved = math.hypot(self.x - x0, self.y - y0)
        self.stop()
        self.get_logger().info(f'{label} distance={moved*100:.1f} cm')

    def turn(self, angle, label=''):
        """Tourne de |angle| rad, sens = signe de angle (+ = gauche)."""
        yaw0 = self.yaw
        w = ANGULAR_SPEED if angle >= 0 else -ANGULAR_SPEED
        target = abs(angle)
        t_end = time.time() + MOVE_TIMEOUT
        turned = 0.0
        while rclpy.ok() and turned < target and time.time() < t_end:
            self._publish(0.0, w)
            self._spin(1.0 / RATE_HZ)
            turned = abs(norm_angle(self.yaw - yaw0))
        self.stop()
        self.get_logger().info(f'{label} angle={math.degrees(turned):.1f} deg')

    def run(self):
        self.wait_odom()
        for s in range(1, SQUARES + 1):
            self.get_logger().info(f'===== CARRE {s}/{SQUARES} =====')
            for c in range(1, 5):        # 4 cotes + 4 coins
                self.drive(+SIDE, f'  cote {c}');   time.sleep(PAUSE)
                self.turn(+CORNER, f'  coin {c}');  time.sleep(PAUSE)
        self.get_logger().info('===== TEST TERMINE =====')
        self.stop()


def main():
    rclpy.init()
    node = MotionTest()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('interruption -> arret moteurs')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
