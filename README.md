# mowbot

Contrôleur diffdrive **micro-ROS** sur **ESP32-P4** pour une tondeuse robot.
Firmware ESP-IDF : abonnement `/cmd_vel`, PID de vitesse par roue vers un
driver **Cytron MDD10A rev 2.0**, odométrie par encodeurs quadrature
(comptage matériel PCNT), publication `/odom` à 50 Hz.

## Architecture

```
[PC / SBC embarqué]                        [ESP32-P4]
  teleop / nav2                              nœud micro-ROS "mowbot_base"
      │ /cmd_vel                               ├─ sub /cmd_vel → cinématique inverse
  micro-ros-agent ◄── série USB ou UDP ──►     ├─ PID vitesse/roue (50 Hz)
      │ /odom                                  ├─ PWM 20 kHz + DIR → MDD10A → moteurs
  robot_localization (EKF)                     └─ encodeurs PCNT → odométrie → pub /odom
```

Sur la tondeuse finale : l'odométrie roues est une source *secondaire*
(patinage sur herbe) — la fusion EKF côté SBC intégrera IMU puis GPS RTK.

## Matériel

| Élément | Référence |
|---|---|
| MCU | ESP32-P4-Function-EV-Board |
| Driver moteurs | Cytron MDD10A rev 2.0 (2×10 A, logique 3,3 V OK) |
| Encodeurs | quadrature A/B, sorties **3,3 V** (le P4 n'est pas tolérant 5 V) |
| IMU (à venir) | ICM-42688-P ou BMI270 |

### Câblage (par défaut, voir `main/config.h`)

| Signal | GPIO P4 | MDD10A / encodeur |
|---|---|---|
| PWM moteur gauche | 20 | PWM1 |
| DIR moteur gauche | 21 | DIR1 |
| PWM moteur droit | 22 | PWM2 |
| DIR moteur droit | 23 | DIR2 |
| Encodeur gauche A/B | 45 / 46 | — |
| Encodeur droit A/B | 47 / 48 | — |
| IMU SDA / SCL | 7 / 8 | ICM-42688 (I2C 400 kHz, addr 0x68) |
| GND | GND | GND (commun obligatoire) |

⚠️ Broches à valider contre le schéma de la carte EV (certaines GPIO sont
prises par le SD, le MIPI ou le PHY Ethernet).

## Compilation et flashage

Le composant micro-ROS ne se compile que sous Linux. Sous **Windows**, les
scripts fournis compilent dans Docker et flashent directement sur le port COM
(prérequis : Docker Desktop, Python + `pip install esptool pyserial`) :

```powershell
.\scripts\build.ps1                  # build dans Docker (clone le composant au 1er run)
.\scripts\build.ps1 -Transport eth   # bascule série → Ethernet/UDP (voir sdkconfig.eth)
.\scripts\build.ps1 -Menuconfig      # réglages fins (IP de l'agent, pins...)
.\scripts\flash.ps1 -Port COM5       # flash depuis Windows (auto-détection si un seul port)
.\scripts\monitor.ps1 -Port COM5     # logs du firmware (Ctrl+] pour quitter)
```

Deux profils de transport : **série** (`sdkconfig.serial`, défaut — UART0/USB
115200) et **Ethernet/UDP** (`sdkconfig.eth` — EMAC interne + PHY IP101 de la
carte EV, pins déjà corrects). Pour l'Ethernet, mettre l'IP du SBC dans
`sdkconfig.eth` (`CONFIG_MICRO_ROS_AGENT_IP`) avant de builder.

Le premier build est long (image ~2 Go + compilation complète de libmicroros) ;
les suivants sont incrémentaux.

Sous **Linux / WSL2** avec ESP-IDF v5.2+ installé nativement :

```bash
./scripts/build.sh                   # ou : build.sh clean | build.sh menuconfig
./scripts/flash.sh /dev/ttyUSB0 monitor
# Sous WSL2, attacher l'USB d'abord (côté Windows, admin) : usbipd attach --wsl --busid <id>
```

Transport par défaut : **série (UART0/USB, 115200 bauds)** — le plus simple
pour démarrer. Pour passer en Ethernet/UDP : `idf.py menuconfig` →
*micro-ROS Settings* → transport, puis adresse/port de l'agent.

## Lancer l'agent + tester

```bash
# Agent micro-ROS (docker), transport série :
docker run -it --rm -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200

# Vérifier :
ros2 topic list          # → /cmd_vel, /odom
ros2 topic echo /odom

# Piloter au clavier :
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Paramètres à calibrer (`main/config.h`)

1. `TICKS_PER_WHEEL_REV` — ticks par tour de roue (×4 quadrature, réducteur inclus).
2. `WHEEL_RADIUS_M` — pousser le robot sur 2 m, comparer à `/odom`.
3. `TRACK_WIDTH_M` — **critique pour le cap** : faire tourner le robot de
   10 tours sur lui-même, ajuster jusqu'à ce que `/odom` indique exactement 10×2π.
4. `PID_KP/KI/KD` — commencer Kp seul, ajouter Ki pour annuler l'erreur statique.

## Sécurité

- Deadman intégré : moteurs coupés si aucun `/cmd_vel` depuis 500 ms
  (agent déconnecté, câble débranché, teleop fermé).
- Pour la tondeuse : l'arrêt d'urgence et la coupure lame devront être
  **matériels**, indépendants de ce firmware.

## Feuille de route

- [x] Diffdrive : cmd_vel → PID → MDD10A, odométrie → /odom
- [x] IMU ICM-42688 → pub `/imu/data_raw` à 100 Hz (calibration biais gyro au boot ;
      le firmware démarre sans IMU si elle est absente)
- [x] Transport Ethernet (profil `sdkconfig.eth`, PHY IP101 de la carte EV)
- [x] Côté SBC : robot_localization — voir [ros2/](ros2/) (EKF vitesses odom + gyro yaw)
- [ ] GPS RTK (u-blox ZED-F9P) — source de position principale en extérieur
- [ ] Coverage planning (opennav_coverage / Fields2Cover)
- [ ] Contrôle lame + capteurs de sécurité (soulèvement, bumper)
