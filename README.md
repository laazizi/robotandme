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
| GND | GND | GND (commun obligatoire) |

⚠️ Broches à valider contre le schéma de la carte EV (certaines GPIO sont
prises par le SD, le MIPI ou le PHY Ethernet).

## Compilation

Prérequis : ESP-IDF **v5.2+** et le composant micro-ROS (branche `humble`).
Le composant micro-ROS se compile sous Linux : sous Windows, passer par
**WSL2** ou le docker `microros/esp-idf-microros`.

```bash
# 1. Cloner le composant micro-ROS (gitignoré, à faire une fois)
git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git \
    components/micro_ros_espidf_component

# 2. Cible + build + flash
idf.py set-target esp32p4
idf.py build
idf.py flash monitor
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
- [ ] IMU (ICM-42688 / BMI270) → pub `/imu/data_raw` à 100 Hz
- [ ] Transport Ethernet (PHY IP101 de la carte EV) vers le SBC
- [ ] Côté SBC : robot_localization (EKF odom + IMU)
- [ ] GPS RTK (u-blox ZED-F9P) — source de position principale en extérieur
- [ ] Coverage planning (opennav_coverage / Fields2Cover)
- [ ] Contrôle lame + capteurs de sécurité (soulèvement, bumper)
