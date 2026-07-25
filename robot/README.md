# `robot/` — logiciel embarqué (portable Jetson ↔ Raspberry Pi)

Tout ce qui tourne **sur le SBC du robot**. Rien n'est spécifique à une machine :
la distro ROS et les ports USB sont **détectés à l'installation**.

## Installer sur un nouveau SBC

```bash
# depuis le PC, dans le dépôt
scp -r robot nvidia@<ip-du-sbc>:~/mowbot_src
ssh nvidia@<ip-du-sbc> 'bash ~/mowbot_src/install.sh'
```

L'installateur :
1. copie tout dans `~/mowbot/`
2. **détecte la distro ROS** (Humble, Jazzy…) — aucun chemin en dur
3. installe les paquets manquants (EKF, SLAM, nav2, rosbridge)
4. génère les services systemd au nom de l'utilisateur courant
5. **identifie les périphériques USB** et écrit les règles udev *de cette machine*
6. ajoute la commande `mowbot` au PATH

Options : `--no-apt`, `--no-udev`, `--no-enable`.

À faire en plus sur un SBC neuf (non automatisé, dépend de l'architecture) :
- **agent micro-ROS** : compiler `MicroXRCEAgent`, ou laisser `run_agent.sh`
  basculer sur l'image Docker `microros/micro-ros-agent`
- **driver lidar** : compiler la branche `N10_V1.0` du dépôt LSLidar dans `~/lidar_ws`

## La commande `mowbot`

```
mowbot up          tout démarrer et vérifier
mowbot status      état complet (services, topics, carte, périphériques)
mowbot nav         (re)démarrer la navigation
mowbot slam        relancer le SLAM

mowbot new-map     archiver la carte et repartir en cartographie
mowbot save-map    sauvegarder la carte

mowbot detect      identifier les USB et écrire les règles udev (sudo)
mowbot devices     idem en lecture seule (diagnostic)
mowbot self-check  secteurs où le lidar se voit lui-même

mowbot node X.py   lancer un outil (motion_test.py, calib_1m.py, tick_count.py…)
mowbot logs [svc]  suivre les logs
mowbot restart A B redémarrer des services
mowbot stop-all    tout arrêter
```

## Organisation

| Dossier | Contenu |
|---|---|
| `bin/` | scripts : `mowbot`, lanceurs `run_*.sh`, exploitation, `detect_devices.sh` |
| `nodes/` | nœuds Python : IMU, filtre de scan, outils de calibration, pilote simple |
| `config/` | YAML et URDF : EKF, SLAM, nav2, lidar, modèle du robot |
| `systemd/` | modèles de services (`__USER__`/`__HOME__` substitués à l'installation) |
| `www/` | joystick navigateur |

`bin/mowbot_env.sh` est sourcé par tous les scripts : il détecte la distro ROS,
expose les chemins (`$MOWBOT_CONFIG`, `$MOWBOT_NODES`, `$MOWBOT_MAPS`…) et les
périphériques (`$DEV_ESP32`, `$DEV_LIDAR`, `$DEV_IMU`).

## ⚠️ USB et udev — le point sensible

L'**ESP32 (DevKitC)** et le **lidar N10** utilisent la même puce **CP2102 avec le
même numéro de série** (`0001`) : impossible de les distinguer par vendor/serial.
Ils sont donc identifiés par **port USB physique** (`ID_PATH`), qui **diffère
d'une machine à l'autre**.

`detect_devices.sh` les reconnaît par un **test réel** :
- ESP32 → répond au protocole bootloader (`esptool chip_id`)
- IMU Razor → puce FTDI qui émet des trames `#YPR=`
- lidar → le port restant qui débite en continu

Puis il écrit `/etc/udev/rules.d/99-mowbot.rules` adapté à la machine.

**À relancer (`mowbot detect`) après :**
- changement de SBC (Jetson → Raspberry Pi)
- déplacement d'une prise USB
- ajout d'un périphérique série

Vérification rapide : `mowbot devices` (ne modifie rien) ou `mowbot status`.

Si tu **modifies le châssis** (antennes, capteurs), relance `mowbot self-check` :
il indique les secteurs où le lidar se voit lui-même, à reporter dans
`nodes/scan_fix.py` (`SELF_SECTORS`).
