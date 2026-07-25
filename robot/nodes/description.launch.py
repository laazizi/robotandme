"""robot_state_publisher : publie le modele URDF et les TF du chassis.

Passe par un launch file et NON par `ros2 run ... -p robot_description:=$(cat ...)` :
un URDF contient des sauts de ligne et des guillemets, que l'analyseur
d'arguments de rclcpp refuse ("failed to parse arguments", arguments.c:343).
Le noeud demarrait alors en boucle sans jamais publier le modele.

Le chemin vient de l'environnement (mowbot_env.sh) pour rester portable
d'un SBC a l'autre.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.environ.get('MOWBOT_CONFIG',
                            os.path.expanduser('~/mowbot/config'))
    with open(os.path.join(config, 'mowbot.urdf')) as f:
        robot_desc = f.read()
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        )
    ])
