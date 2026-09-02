"""Que dit le modele URDF REELLEMENT PUBLIE : roues, lidar, plateau.

Utile quand RViz montre un robot qui ne ressemble pas au fichier du depot --
le modele publie vient de la machine, pas du depot."""
import re, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
rclpy.init(); n = Node("chk"); g = {}
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
n.create_subscription(String, "/robot_description", lambda m: g.setdefault("d", m.data), q)
t0 = time.time()
while time.time() - t0 < 12 and "d" not in g:
    rclpy.spin_once(n, timeout_sec=0.2)
d = g.get("d", "")
if not d:
    print("  aucune description publiee"); raise SystemExit
for nom, cle in (("roue gauche", "base_to_wheel_left"),
                 ("roue droite", "base_to_wheel_right"),
                 ("lidar", "deck_to_lidar")):
    m = re.search(cle + r'.*?<origin xyz="([^"]*)"', d, re.S)
    print(f"  {nom:12s} {m.group(1) if m else 'introuvable'}")
m = re.search(r'base_link.*?<cylinder radius="([^"]*)"', d, re.S)
print(f"  {'plateau':12s} rayon {m.group(1) if m else '?'}")
print(f"  taille de la description : {len(d)} caracteres")
