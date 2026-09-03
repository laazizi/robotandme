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
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    config = os.environ.get('MOWBOT_CONFIG',
                            os.path.expanduser('~/mowbot/config'))
    with open(os.path.join(config, 'mowbot.urdf')) as f:
        robot_desc = f.read()
    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        )
    ]
    # steer_state.py anime le seul joint MOBILE du modele (steer_joint, la roue
    # directrice). robot_state_publisher ne bouge un joint mobile que s'il
    # recoit /joint_states : sans ce noeud la roue reste figee a zero dans RViz
    # alors qu'elle braque vraiment. Lance ici, a cote du modele, parce que les
    # deux vont ensemble -- un URDF a joint mobile sans publieur est incomplet.
    # Absent (chassis diffdrive sans roue directrice) : on ne lance rien.
    steer = os.path.join(os.environ.get('MOWBOT_NODES',
                                        os.path.expanduser('~/mowbot/nodes')),
                         'steer_state.py')
    if os.path.exists(steer) and 'steer_joint' in robot_desc:
        nodes.append(ExecuteProcess(cmd=['python3', steer], output='screen'))
    return LaunchDescription(nodes)
