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
- **driver lidar** : seulement pour le **N10** — compiler la branche `N10_V1.0`
  du dépôt LSLidar dans `~/lidar_ws`. Le **LD14 ne demande rien** (nœud Python).

## Deux lidars supportés

| | **N10** (LSLidar) | **LD14** (LDRobot) |
|---|---|---|
| Mesuré | 10 Hz, 450 pts/tour, 12 m | 6 Hz, 391 pts/tour, 8 m |
| Driver | C++ à compiler (`~/lidar_ws`) | `nodes/ld14_node.py`, rien à compiler |
| Vitesse série | 230400 | 115200 |

Le modèle est **reconnu automatiquement** par `mowbot detect`, qui sonde le
flux série (`bin/lidar_probe.py`) et l'enregistre dans `lidar_model.env`.
Forcer un modèle : `export MOWBOT_LIDAR=ld14` (ou `n10`).

Les LD06/LD19 utilisent le même protocole que le LD14 à 230400 bauds : ils
sont détectés et pilotés par le même nœud.

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

## Profils de navigation : robot A, robot B, Ackermann

Le SBC choisit son profil dans `~/mowbot/robot_profile.env` :

```
MOWBOT_ROBOT=a          # tondeuse P4, 12 V           -> nav2_params.yaml + speeds.env (A_*)
MOWBOT_ROBOT=b          # tondeuse WROOM, 24 V        -> nav2_params.yaml + speeds.env (B_*)
MOWBOT_ROBOT=ackerbot   # tricycle a roue directrice  -> nav2_params_ackerbot.yaml + bt_ackerbot*.xml
```

Sans fichier, profil **B par sécurité** (vitesses les plus basses). `start_nav.sh`
génère une copie du YAML avec les nombres du profil ; le fichier de référence
n'est jamais modifié.

**Ackermann** (`bin/nav_profile_ackerbot.sh`) : ce robot ne pivote pas sur place,
donc DWB, NavFn et la récupération `Spin` lui sont interdits — il reçoit
Regulated Pure Pursuit (marche arrière autorisée, pas de rotation vers le cap),
SmacPlannerHybrid en Reeds-Shepp, et **deux** arbres sans `Spin` —
`bt_ackerbot.xml` (NavigateToPose) et `bt_ackerbot_through_poses.xml`
(NavigateThroughPoses) : `bt_navigator` charge les deux à l'activation, et n'en
remplacer qu'un fait échouer toute la pile. Le **rayon de
braquage minimal n'est jamais saisi à la main** : `bin/gen_ackerbot_geometry.py`
le dérive de `controllers/ackerbot_p4/main/robot.h` dans
`config/ackerbot_geometry.env` (à relancer après toute modification de la
géométrie du firmware ; le test hôte `kin_ackermann/test/run.sh` échoue si le
fichier est périmé). Sans ce fichier, le lancement est **refusé** plutôt que de
naviguer avec un rayon inventé.

## ⚠️ USB et udev — le point sensible

L'**ESP32 (DevKitC)** et les **lidars N10 / LD14** utilisent la même puce
**CP2102 avec le même numéro de série** (`0001`) : impossible de les distinguer
par vendor/serial. Ils sont donc identifiés par **port USB physique**
(`ID_PATH`), qui **diffère d'une machine à l'autre**.

`detect_devices.sh` les reconnaît par un **test réel** :
- ESP32 → répond au protocole bootloader (`esptool chip_id`)
- IMU Razor → puce FTDI qui émet des trames `#YPR=`
- lidar → signature de son flux (`lidar_probe.py`), qui donne aussi le modèle

⚠️ Piège vérifié sur la Raspberry : l'agent micro-ROS, ne trouvant pas
`/dev/mowbot_esp32`, se rabattait sur le premier CP2102 venu — **le lidar**. Il
ouvrait son port et `/scan` devenait muet. `run_agent.sh` exclut désormais le
port du lidar et **confirme** l'ESP32 par son bootloader avant de s'y attacher.

Puis il écrit `/etc/udev/rules.d/99-mowbot.rules` adapté à la machine.

**À relancer (`mowbot detect`) après :**
- changement de SBC (Jetson → Raspberry Pi)
- déplacement d'une prise USB
- ajout d'un périphérique série

Vérification rapide : `mowbot devices` (ne modifie rien) ou `mowbot status`.

Si tu **modifies le châssis** (antennes, capteurs), relance `mowbot self-check` :
il indique les secteurs où le lidar se voit lui-même, à reporter dans
`nodes/scan_fix.py` (`SELF_SECTORS`).
