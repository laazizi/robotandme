"""nav2 envoie-t-il des commandes, et le scan arrive-t-il a l heure ?"""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
rclpy.init(); n = Node("navchk")
cv = []; sc = []
n.create_subscription(Twist, "/cmd_vel", lambda m: cv.append((time.time(), m.linear.x, m.angular.z)), 10)
n.create_subscription(LaserScan, "/scan", lambda m: sc.append(m), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 14:
    rclpy.spin_once(n, timeout_sec=0.2)
print(f"  /cmd_vel : {len(cv)} messages en 14 s")
if cv:
    vx = [c[1] for c in cv]; wz = [c[2] for c in cv]
    nz = sum(1 for c in cv if abs(c[1]) > 1e-3 or abs(c[2]) > 1e-3)
    print(f"    vx  min {min(vx):+.3f} max {max(vx):+.3f}   wz min {min(wz):+.3f} max {max(wz):+.3f}")
    print(f"    messages NON NULS : {nz} / {len(cv)}")
else:
    print("    => nav2 n'envoie AUCUNE commande")
if sc:
    st = sc[-1].header.stamp
    age = n.get_clock().now().nanoseconds*1e-9 - (st.sec + st.nanosec*1e-9)
    print(f"  /scan cote JETSON : age {age*1000:+.0f} ms   {len(sc)/14:.2f} Hz")
