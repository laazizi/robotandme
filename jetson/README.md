# Configuration Jetson (SBC embarqué du robot)

Jetson Orin Nano, Ubuntu 22.04, ROS 2 **Humble**. Ces fichiers documentent
l'installation faite sur le Jetson (à recopier en cas de réinstallation).

## Rôle
Le Jetson fait tourner 3 services systemd qui démarrent au boot :

| Service | Rôle | Sortie |
|---------|------|--------|
| `mowbot-agent` | agent micro-ROS (pont série ESP32 ↔ DDS) | `/odom`, `/cmd_vel` |
| `mowbot-razor` | lit le gyro Razor (série FTDI) | `/imu/data_raw` |
| `mowbot-ekf`   | fusion robot_localization | `/odometry/filtered` + TF `odom→base_link` |

## Périphériques (liens udev stables par n° de série)
- **ESP32** (CH343 1a86:55d3, serial 5B7B032234) → `/dev/mowbot_esp32`
- **Razor IMU** (FTDI 0403:6015, serial FT1CBPYF) → `/dev/mowbot_imu`

## Installation (résumé)
```bash
# 1. ROS 2 Humble ros-base + robot_localization
sudo apt install ros-humble-ros-base ros-humble-robot-localization python3-serial

# 2. Agent micro-ROS : build depuis les sources eProsima (Micro-XRCE-DDS-Agent
#    v2.4.2) avec -DUAGENT_USE_SYSTEM_FASTDDS=ON  -> /usr/local/bin/MicroXRCEAgent

# 3. Copier les fichiers de ce dossier :
sudo cp udev/*.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger
cp *.sh *.py ~/                          # robot_start.sh, run_razor.sh, run_ekf.sh, gyro_calib.py
cp ../ros2/razor_imu_node.py ../ros2/ekf.yaml ~/   # nœud IMU + config EKF (partages)
sudo cp systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now mowbot-agent mowbot-razor mowbot-ekf
```

## Gestion
```bash
sudo systemctl {status|restart|stop} mowbot-{agent|razor|ekf}
journalctl -u mowbot-ekf -f
```
Ne PAS lancer les nœuds à la main quand les services tournent (conflit de port/TF).
