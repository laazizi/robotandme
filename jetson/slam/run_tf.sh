#!/bin/bash
# TF statiques du robot : base_link -> laser_link (lidar) et -> imu_link (gyro).
# Regroupees dans UN service dedie : elles etaient avant lancees par les
# scripts capteurs, ou (1) `ros2 run` echoue en contexte systemd et (2) chaque
# redemarrage empilait des publishers concurrents -> TF oscillante -> le SLAM
# rejetait tous les scans. Binaire appele en CHEMIN ABSOLU.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
STP=/opt/ros/humble/lib/tf2_ros/static_transform_publisher

# lidar : 10 cm devant le centre, 28 cm de haut, oriente vers l'avant
"$STP" --x 0.10 --y 0 --z 0.28 --roll 0 --pitch 0 --yaw 0 \
       --frame-id base_link --child-frame-id laser_link &

# IMU Razor : centree (offset non critique, seul le gyro yaw est utilise)
"$STP" --x 0 --y 0 --z 0.05 --roll 0 --pitch 0 --yaw 0 \
       --frame-id base_link --child-frame-id imu_link &

wait
