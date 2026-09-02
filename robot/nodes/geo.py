import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
D = 180 / math.pi
rclpy.init(); n = Node("geo"); buf = Buffer(); TransformListener(buf, n)
sc = {}
n.create_subscription(LaserScan, "/scan", lambda m: sc.setdefault("m", m),
                      qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 8 and "m" not in sc:
    rclpy.spin_once(n, timeout_sec=0.2)

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
print("  --- TF reellement publiees ---")
for a, b in (("base_link", "laser_link"), ("odom", "base_link"), ("map", "odom")):
    for _ in range(30):
        try:
            t = buf.lookup_transform(a, b, rclpy.time.Time()); break
        except Exception:
            rclpy.spin_once(n, timeout_sec=0.2); t = None
    if t:
        tr = t.transform
        print(f"    {a}->{b:11s} x={tr.translation.x:+.3f} y={tr.translation.y:+.3f} "
              f"z={tr.translation.z:+.3f}  yaw={yaw(tr.rotation)*D:+7.1f} deg")
    else:
        print(f"    {a}->{b:11s} ABSENTE")

m = sc.get("m")
if m:
    print("\n  --- le scan lui-meme ---")
    print(f"    {len(m.ranges)} pts   {m.angle_min*D:.0f}..{m.angle_max*D:.0f} deg   "
          f"portee {m.range_min:.2f}..{m.range_max:.1f} m   repere {m.header.frame_id}")
    # direction de l'obstacle le PLUS PROCHE, dans le repere du lidar puis du robot
    best = None
    for i, r in enumerate(m.ranges):
        if m.range_min < r < m.range_max and (best is None or r < best[1]):
            best = (i, r)
    if best:
        i, r = best
        a_laser = m.angle_min + i * m.angle_increment
        # laser_link -> base_link : rotation de 180 deg et decalage de -0.10
        a_base = a_laser + math.pi
        while a_base > math.pi: a_base -= 2*math.pi
        x = -0.10 + r * math.cos(a_base)
        y = 0.0 + r * math.sin(a_base)
        print(f"    obstacle le plus proche : {r:.3f} m")
        print(f"      a {a_laser*D:+.0f} deg dans le repere LIDAR")
        print(f"      a {a_base*D:+.0f} deg dans le repere ROBOT  ->  x={x:+.2f} m  y={y:+.2f} m")
        quoi = "DEVANT" if x > 0.15 else ("DERRIERE" if x < -0.15 else "sur le cote")
        print(f"      soit {quoi} le robot")
