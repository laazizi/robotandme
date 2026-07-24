from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    with open('/home/nvidia/mowbot.urdf') as f:
        robot_desc = f.read()
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        )
    ])
