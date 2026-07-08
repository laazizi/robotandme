# Bringup cote SBC : EKF (odom + IMU) et TF statique de l'IMU.
# Le micro-ros-agent se lance a part (voir ros2/README.md).
#
# Lancement direct par chemin, sans package :
#   ros2 launch ./ros2/bringup.launch.py

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = os.path.join(os.path.dirname(__file__), 'ekf.yaml')

    return LaunchDescription([
        # Position de l'IMU par rapport au chassis — a ajuster quand elle
        # sera montee (x y z yaw pitch roll, en metres/radians)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
    ])
