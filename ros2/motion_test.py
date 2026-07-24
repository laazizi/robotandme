#!/usr/bin/env python3
"""Test aller-retour mowbot (boucle fermee sur /odometry/filtered).

Par cycle (x5) :
  1. avance 50 cm
  2. demi-tour 180 deg HORAIRE (sens des aiguilles d'une montre)
  3. revient 50 cm
  4. demi-tour 180 deg ANTI-HORAIRE  -> cap et position d'origine

Ctrl+C : arret d'urgence (consigne nulle envoyee avant de quitter).

Prerequis : services mowbot actifs sur le Jetson (agent + IMU + EKF).
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
LINEAR_SPEED  = 0.15                  # m/s (>= 0.12 : en dessous le PID pompe sur la friction)
ANGULAR_SPEED = 0.5                   # rad/s
DIST          = 0.50                  # distance aller : 50 cm
HALF_TURN     = math.radians(180.0)   # demi-tour
CYCLES        = 5                     # nombre d'allers-retours
PAUSE         = 0.5                   # s entre chaque mouvement
RATE_HZ       = 20.0                  # publication (> deadman firmware 500 ms)
MOVE_TIMEOUT  = 25.0                  # securite : abandon d'un mouvement trop long


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class MotionTest(Node):
    def __init__(self):
        super().__init__('motion_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # RELIABLE pour matcher /odometry/filtered (l'EKF publie en reliable).
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        # Pose fusionnee EKF : cap = gyro (insensible au patinage).
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, qos)
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
        self.get_logger().info('attente de /odometry/filtered...')
        while rclpy.ok() and self.x is None:
            self._spin(0.1)
        self.get_logger().info('odometrie recue.')

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
        """Tourne de `angle` rad (signe : + = anti-horaire/gauche, - = horaire/droite).
        Accumule les increments de cap -> gere les rotations >= 180 deg."""
        w = ANGULAR_SPEED if angle >= 0 else -ANGULAR_SPEED
        target = abs(angle)
        prev = self.yaw
        turned = 0.0
        t_end = time.time() + MOVE_TIMEOUT
        while rclpy.ok() and abs(turned) < target and time.time() < t_end:
            self._publish(0.0, w)
            self._spin(1.0 / RATE_HZ)
            turned += norm_angle(self.yaw - prev)
            prev = self.yaw
        self.stop()
        self.get_logger().info(f'{label} angle={math.degrees(abs(turned)):.1f} deg')

    def run(self):
        self.wait_odom()
        x_dep, y_dep = self.x, self.y
        for c in range(1, CYCLES + 1):
            self.get_logger().info(f'===== ALLER-RETOUR {c}/{CYCLES} =====')
            self.drive(+DIST,      '  aller ');           time.sleep(PAUSE)
            self.turn(-HALF_TURN,  '  demi-tour horaire');   time.sleep(PAUSE)
            self.drive(+DIST,      '  retour');           time.sleep(PAUSE)
            self.turn(+HALF_TURN,  '  demi-tour anti-hor');  time.sleep(PAUSE)
            err = math.hypot(self.x - x_dep, self.y - y_dep)
            self.get_logger().info(f'  ecart au depart : {err*100:.1f} cm')
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
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
