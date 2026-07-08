# mowbot — côté SBC / PC

Fusion capteurs (EKF) et outillage ROS 2 qui tournent en face du firmware.

## Prérequis

ROS 2 Humble (ou docker Vulcanexus) et :

```bash
sudo apt install ros-humble-robot-localization ros-humble-teleop-twist-keyboard
```

## 1. Lancer le micro-ros-agent

```bash
# Transport série (USB) :
docker run -it --rm -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200

# Transport Ethernet/UDP (firmware compilé avec -Transport eth) :
docker run -it --rm --net=host \
    microros/micro-ros-agent:humble udp4 --port 8888
```

Vérifier que le P4 est connecté : `ros2 topic list` doit montrer
`/cmd_vel`, `/odom` et `/imu/data_raw`.

## 2. Lancer l'EKF

```bash
ros2 launch ./ros2/bringup.launch.py
```

Sortie : `/odometry/filtered` + TF `odom → base_link`. La config
[ekf.yaml](ekf.yaml) fusionne les **vitesses** de l'odométrie roues
(vx, vyaw) avec le **gyro yaw** de l'IMU — sur herbe, la pose odométrique
brute dérive trop pour être fusionnée directement.

## 3. Tester

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # piloter
ros2 topic echo /odometry/filtered                      # verifier la fusion
ros2 topic hz /imu/data_raw                             # ~100 Hz attendu
```

Contrôle rapide de cohérence : robot immobile → vitesses nulles et cap
stable ; rotation sur place à la main → le yaw de `/odometry/filtered`
doit suivre sans à-coups (gyro) et sans dérive au repos (biais calibré au boot).
