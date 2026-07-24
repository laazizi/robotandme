#!/usr/bin/env python3
"""Navigation 'aller au but' : clic '2D Goal Pose' dans RViz -> le robot y va.

Machine a etats STRICTE adaptee aux moteurs a forte friction statique :
chaque commande reste dans un regime qui marche (jamais de melange
avance + grosse rotation qui mettrait une roue sous le seuil de friction).

  PIVOT  : rotation pure a W_TURN vers le cap du but (tolerance 8 deg)
  DRIVE  : ligne droite a V_DRIVE, correction de cap MINIME (roues toujours
           au-dessus du seuil). Si le cap derive > REAIM -> retour PIVOT.
  ALIGN  : rotation pure vers l'orientation finale demandee
  IDLE   : moteurs coupes

ATTENTION : navigation a l'aveugle (pas de capteur d'obstacle). Zone degagee !
Lancement (Jetson) :  python3 ~/goto_goal.py     (Ctrl+C = stop moteurs)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

V_DRIVE   = 0.16                   # m/s : avance (>= 0.13 sinon friction)
W_TURN    = 0.5                    # rad/s : pivot (regime valide par motion_test)
W_TRIM    = 0.15                   # rad/s max de correction pendant DRIVE
                                   #   -> roue la plus lente : 0.16-0.15*0.21 = 0.13 OK
TOL_POS   = 0.06                   # m : but atteint
TOL_YAW   = math.radians(8.0)      # tolerance pivot/align
REAIM     = math.radians(25.0)     # derive de cap max en DRIVE avant re-pivot
RATE_HZ   = 20.0


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GotoGoal(Node):
    def __init__(self):
        super().__init__('goto_goal')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mpub = self.create_publisher(Marker, '/goto_marker', 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.x = self.y = self.yaw = None
        self.goal = None
        self.phase = 'idle'
        self.dbg = 0
        self.create_timer(1.0 / RATE_HZ, self.step)
        self.get_logger().info("pret — cliquez '2D Goal Pose' dans RViz (frame odom)")

    def on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = yaw_of(m.pose.pose.orientation)

    def on_goal(self, m):
        self.goal = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))
        self.phase = 'pivot'
        self._goal_marker(m)
        self.get_logger().info(
            f'NOUVEAU BUT : x={self.goal[0]:.2f} y={self.goal[1]:.2f} '
            f'cap={math.degrees(self.goal[2]):.0f} deg')

    def _goal_marker(self, m=None):
        """Fleche verte sur le but courant ; None = effacer."""
        mk = Marker()
        mk.header.frame_id = 'odom'
        mk.ns = 'goto'
        mk.id = 1
        if m is None:
            mk.action = Marker.DELETE
        else:
            mk.type = Marker.ARROW
            mk.action = Marker.ADD
            mk.pose = m.pose
            mk.scale.x, mk.scale.y, mk.scale.z = 0.25, 0.05, 0.05
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.9, 0.2, 0.95
        self.mpub.publish(mk)

    def cmd(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def step(self):
        if self.goal is None or self.x is None:
            return
        gx, gy, gyaw = self.goal
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        err_bear = norm(bearing - self.yaw)

        self.dbg += 1
        if self.dbg % 20 == 0:
            self.get_logger().info(
                f'{self.phase}: dist={dist*100:.0f}cm err_cap={math.degrees(err_bear):+.0f}deg '
                f'pos=({self.x:.2f},{self.y:.2f}) yaw={math.degrees(self.yaw):.0f}deg')

        if self.phase == 'pivot':
            if dist < TOL_POS:
                self.phase = 'align'
            elif abs(err_bear) < TOL_YAW:
                self.phase = 'drive'
                self.cmd(V_DRIVE, 0.0)
            else:
                self.cmd(0.0, W_TURN if err_bear > 0 else -W_TURN)

        elif self.phase == 'drive':
            if dist < TOL_POS:
                self.phase = 'align'
                self.cmd(0.0, 0.0)
            elif abs(err_bear) > REAIM:
                self.phase = 'pivot'          # trop devie : on s'arrete et on revise
                self.cmd(0.0, 0.0)
            else:
                # correction de cap DOUCE : jamais une roue sous le seuil
                w = max(-W_TRIM, min(W_TRIM, 0.8 * err_bear))
                self.cmd(V_DRIVE, w)

        elif self.phase == 'align':
            err = norm(gyaw - self.yaw)
            if abs(err) < TOL_YAW:
                self.cmd(0.0, 0.0)
                self.get_logger().info(f'BUT ATTEINT (ecart {dist*100:.1f} cm)')
                self._goal_marker(None)
                self.goal = None
                self.phase = 'idle'
            else:
                self.cmd(0.0, W_TURN if err > 0 else -W_TURN)


def main():
    rclpy.init()
    n = GotoGoal()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.cmd(0.0, 0.0)
        except Exception:
            pass
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
