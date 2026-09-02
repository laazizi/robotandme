import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node("ts2"); buf = Buffer(); TransformListener(buf, n)
raw, fix = [], []
n.create_subscription(LaserScan, "/scan_raw", lambda m: raw.append(m), qos_profile_sensor_data)
n.create_subscription(LaserScan, "/scan", lambda m: fix.append(m), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 14:
    rclpy.spin_once(n, timeout_sec=0.2)
def age(m):
    st = m.header.stamp
    now = n.get_clock().now().nanoseconds * 1e-9
    return now - (st.sec + st.nanosec * 1e-9)
for nom, lst in (("/scan_raw", raw), ("/scan", fix)):
    if not lst:
        print(f"  {nom:10s} RIEN"); continue
    a = [age(m) for m in lst[-8:]]
    print(f"  {nom:10s} n={len(lst)}  age min {min(a)*1000:+.0f} ms  max {max(a)*1000:+.0f} ms")
# la TF est-elle disponible A L'HORODATAGE du scan ?
if fix:
    m = fix[-1]
    for tgt in ("odom", "map"):
        try:
            buf.lookup_transform(tgt, "laser_link", m.header.stamp)
            print(f"  TF {tgt}<-laser_link a l horodatage du scan : OK")
        except Exception as e:
            print(f"  TF {tgt}<-laser_link a l horodatage du scan : ECHEC")
            print(f"     {str(e)[:130]}")
