# mowbot

Contrôleur diffdrive **micro-ROS** sur **ESP32-P4** pour une tondeuse robot
autonome. Le firmware ESP-IDF s'abonne à `/cmd_vel`, applique un PID de vitesse
par roue vers un driver **Cytron MDD10A rev 2.0**, mesure l'odométrie par
encodeurs quadrature (comptage matériel PCNT) et publie `/odom` à 50 Hz + l'IMU
(`/imu/data_raw`) à 100 Hz. La fusion capteurs (EKF) et la navigation tournent
côté SBC/PC, en ROS 2 Humble.

> Firmware = MCU (ESP32-P4). Agent + ROS 2 = SBC/PC (Linux). Le MCU ne parle pas
> ROS 2 directement : il parle **XRCE-DDS** au `micro-ros-agent`, qui republie
> les topics dans le graphe ROS 2.

## Sommaire

- [Architecture](#architecture)
- [Matériel & câblage](#matériel--câblage)
- [Prérequis](#prérequis)
- [Compilation & flashage](#compilation--flashage)
- [Démarrer avec ROS 2](#démarrer-avec-ros-2) ← le parcours complet
- [Première mise en route (banc)](#première-mise-en-route-banc)
- [Paramètres à calibrer](#paramètres-à-calibrer)
- [Sécurité](#sécurité)
- [Dépannage](#dépannage)
- [Structure du dépôt](#structure-du-dépôt)
- [Feuille de route](#feuille-de-route)

## Architecture

```
[PC / SBC embarqué — Linux, ROS 2 Humble]          [ESP32-P4]
  teleop / nav2                                      nœud micro-ROS "mowbot_base"
      │ /cmd_vel                                       ├─ sub /cmd_vel → cinématique inverse
  micro-ros-agent ◄── série USB (ou UDP/RJ45) ──►      ├─ PID vitesse/roue (50 Hz)
      │ /odom  /imu/data_raw                           ├─ PWM 20 kHz + DIR → MDD10A → moteurs
  robot_localization (EKF) → /odometry/filtered        └─ encodeurs PCNT → odométrie → /odom
```

Sur la tondeuse finale, l'odométrie roues est une source *secondaire* (patinage
sur herbe) : l'EKF côté SBC fusionne les **vitesses** de `/odom` avec le **gyro
yaw** de l'IMU, et le **GPS RTK** (à venir) fournira la position absolue.

Décisions d'architecture détaillées (et leur pourquoi) : voir [CLAUDE.md](CLAUDE.md).

## Matériel & câblage

| Élément | Référence |
|---|---|
| MCU | ESP32-P4-Function-EV-Board |
| Driver moteurs | Cytron MDD10A rev 2.0 (2×10 A, logique 3,3 V directe, sign-magnitude) |
| Encodeurs | quadrature A/B, sorties **3,3 V** (le P4 n'est **pas** tolérant 5 V) |
| IMU | ICM-42688-P en I2C (gyro yaw = cap court terme ; pas de magnéto) |

### Câblage (par défaut, dans [`controllers/mowbot_p4/main/robot.h`](controllers/mowbot_p4/main/robot.h))

| Signal | GPIO P4 | Vers |
|---|---|---|
| PWM moteur gauche | 20 | PWM1 (MDD10A) |
| DIR moteur gauche | 21 | DIR1 (MDD10A) |
| PWM moteur droit | 22 | PWM2 (MDD10A) |
| DIR moteur droit | 23 | DIR2 (MDD10A) |
| Encodeur gauche A/B | 45 / 46 | encodeur roue gauche |
| Encodeur droit A/B | 47 / 48 | encodeur roue droite |
| IMU SDA / SCL | 7 / 8 | ICM-42688 (I2C, addr 0x68) |
| GND | GND | **masse commune obligatoire** (ESP32 ↔ MDD10A ↔ encodeurs) |

> ⚠️ **Broches non validées** contre le schéma de la carte EV : certaines GPIO
> sont réservées (SD, MIPI, PHY Ethernet). Si un signal ne sort pas, suspecter
> un conflit de broche → voir [Dépannage](#dépannage).

## Prérequis

**Poste de développement (Windows)** — le composant micro-ROS ne compile que
sous Linux, donc le build passe par Docker :

- **Docker Desktop** (backend WSL2) démarré ;
- **Python 3** + `pip install esptool pyserial` (pour flasher via le port COM).

**SBC / PC ROS 2 (Linux)** — pour faire tourner l'agent et ROS 2 :

- ROS 2 **Humble** (ou l'image Docker Vulcanexus) ;
- `sudo apt install ros-humble-robot-localization ros-humble-teleop-twist-keyboard`.

## Compilation & flashage

Scripts PowerShell (Windows) — le 1er run clone le composant micro-ROS et
compile libmicroros (~15-20 min, ~2 Go d'image) ; les suivants sont incrémentaux.

```powershell
.\scripts\build.ps1                   # build, transport série (défaut)
.\scripts\build.ps1 -Transport eth    # bascule série → Ethernet/UDP (reconstruit libmicroros)
.\scripts\build.ps1 -Clean            # fullclean + build
.\scripts\build.ps1 -Menuconfig       # réglages fins (IP agent, pins, révision puce…)

.\scripts\flash.ps1 -Port COM20 -Monitor   # flash + ouvre le moniteur (Ctrl+] pour quitter)
.\scripts\monitor.ps1 -Port COM20          # moniteur seul
```

- **Trouver le port COM** : `flash.ps1`/`monitor.ps1` auto-détectent s'il n'y a
  qu'un seul port ; sinon passez `-Port`. La carte EV apparaît comme
  *USB-Enhanced-SERIAL CH343* (visible dans le Gestionnaire de périphériques).
- **Deux profils de transport** : `sdkconfig.serial` (défaut, UART0/USB 115200)
  et `sdkconfig.eth` (EMAC interne + PHY IP101 de la carte EV). Changer de
  transport **reconstruit libmicroros** (géré par le script, mais c'est long).
- **Révision de puce** : les cartes EV pré-série embarquent un **ESP32-P4 v1.3**.
  IDF v5.5 vise le silicium v3.1+ par défaut ; le support des révisions <3.0 est
  activé dans [`controllers/mowbot_p4/sdkconfig.defaults`](controllers/mowbot_p4/sdkconfig.defaults)
  (idem `ackerbot_p4`). ⚠️ un binaire compilé
  ainsi ne bootera **pas** sur une puce v3.x de production (retirer les 2 lignes
  `ESP32P4_REV_*` le jour venu).

⚠️ Les scripts `.ps1` ci-dessus datent de l'époque où le projet ESP-IDF était à
la racine ; ils **n'ont pas été adaptés** au découpage en contrôleurs
(`controllers/`, voir ci-dessous). Sous Linux / WSL2, c'est la voie de référence :

```bash
. ~/esp/esp-idf/export.sh
./scripts/build.sh mowbot_p4                 # <mowbot_p4|mowbot_wroom|ackerbot_p4> [build|clean|menuconfig] [serial|eth]
./scripts/flash.sh mowbot_p4 /dev/ttyACM0 monitor
```

**Un contrôleur = un dossier** dans [`controllers/`](controllers/README.md) :
`main/robot.h` (broches, géométrie, gains), `main/CMakeLists.txt` (choix de la
cinématique diffdrive ou Ackermann), `sdkconfig.defaults` (cible). Le code
commun est dans `controllers/common/`. Chaque contrôleur a son propre `build/`.

## Démarrer avec ROS 2

Le parcours complet, du firmware flashé jusqu'au robot piloté au clavier.

### 1. Flasher et vérifier que le firmware tourne

```powershell
.\scripts\flash.ps1 -Port COM20 -Monitor
```

Au boot, le moniteur affiche la calibration du gyro (~1 s, robot immobile) puis,
en boucle : `micro-ros-agent injoignable, nouvel essai dans 1 s...` **entrelacé
avec des octets binaires** (`~…XRCE…`). C'est **normal** en transport série :
UART0 porte à la fois les logs et les trames XRCE-DDS. Le firmware est vivant et
attend l'agent.

### 2. Rendre l'ESP32 visible côté Linux

L'agent et ROS 2 tournent sous Linux ; l'ESP32 doit y être accessible en série :

- **Sur le SBC/Jetson (cible)** : brancher l'ESP32 en USB → il apparaît en
  `/dev/ttyUSB0` (ou `/dev/ttyACM0`).
- **Depuis ce PC Windows (test)** : exposer le port COM à WSL2 avec
  [usbipd-win](https://github.com/dorssel/usbipd-win), en administrateur :
  ```powershell
  usbipd list                          # repérer le busid du CH343
  usbipd attach --wsl --busid <busid>  # l'ESP32 devient /dev/ttyUSB0 dans WSL
  ```

> ⚠️ **Transport série : fermer le moniteur avant de lancer l'agent.** UART0 est
> partagé (logs + transport) ; un seul process peut tenir le port. `Ctrl+]` pour
> quitter le moniteur IDF.

### 3. Lancer le micro-ros-agent

```bash
# Transport série (USB) :
docker run -it --rm -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200

# Transport Ethernet/UDP (firmware compilé avec -Transport eth) :
docker run -it --rm --net=host \
    microros/micro-ros-agent:humble udp4 --port 8888
```

Dès la connexion, le firmware cesse de logger « injoignable » et crée ses topics.

### 4. Vérifier les topics

```bash
ros2 topic list           # → /cmd_vel, /odom, /imu/data_raw
ros2 topic echo /odom
ros2 topic hz /imu/data_raw   # ~100 Hz attendu
```

### 5. Lancer la fusion EKF (optionnel mais recommandé)

```bash
ros2 launch ./ros2/bringup.launch.py     # robot_localization
ros2 topic echo /odometry/filtered       # pose fusionnée + TF odom→base_link
```

L'EKF fusionne les **vitesses** de `/odom` avec le **gyro yaw**. Détails et
config : [ros2/README.md](ros2/README.md) et [ros2/ekf.yaml](ros2/ekf.yaml).

### 6. Piloter

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Le deadman coupe les moteurs si `/cmd_vel` cesse pendant 500 ms (agent
déconnecté, teleop fermé…).

## Première mise en route (banc)

**Robot sur cales, roues en l'air**, avant tout essai au sol.

- Un **auto-test moteurs temporaire** est actuellement câblé dans
  [`app_main`](controllers/common/base/main.c) : au boot, les deux roues tournent en avant 1 s
  puis en arrière 1 s (logs `TEST MOTEURS : …`). Il sert à valider le câblage et
  le sens de rotation, **à retirer** ensuite.
- **Sens de rotation** : si une roue tourne à l'envers, inverser son flag
  `MOTOR_L_INVERT` / `MOTOR_R_INVERT` dans [`controllers/mowbot_p4/main/robot.h`](controllers/mowbot_p4/main/robot.h) ; si
  le robot recule quand il devrait avancer, inverser **les deux**. Même logique
  avec `ENC_L_INVERT` / `ENC_R_INVERT` si `/odom` décroît en marche avant.

## Paramètres à calibrer ([`controllers/mowbot_p4/main/robot.h`](controllers/mowbot_p4/main/robot.h))

1. `TICKS_PER_WHEEL_REV` — ticks par tour de roue (×4 quadrature, réducteur inclus).
2. `WHEEL_RADIUS_M` — pousser le robot sur 2 m, comparer à `/odom`.
3. `TRACK_WIDTH_M` — **critique pour le cap** : faire tourner le robot 10 tours
   sur lui-même, ajuster jusqu'à ce que `/odom` indique exactement 10×2π.
4. `PID_KP/KI/KD` — commencer par Kp seul, ajouter Ki pour annuler l'erreur statique.

## Sécurité

- **Deadman** firmware : moteurs coupés sans `/cmd_vel` depuis 500 ms.
- Pour la tondeuse, l'**arrêt d'urgence** et la **coupure lame** devront être
  **matériels**, indépendants de ce firmware.

## Dépannage

| Symptôme | Cause probable / solution |
|---|---|
| `docker info … NotSpecified: WARNING` au build | PS 5.1 transforme le stderr natif en erreur ; déjà corrigé (check via `cmd /c`). Vérifier que Docker Desktop est démarré. |
| `CONFIG_MICRO_ROS_AGENT_IP undeclared` à la compil | libmicroros compilé pour le mauvais transport. `build.ps1` gère ça via `app-colcon.meta` (série→`custom`, eth→`udp`) ; relancer le build. |
| `rcl/rcl.h: No such file` | libmicroros incomplet (build interrompu). `build.ps1` le détecte et reconstruit ; sinon `-Clean`. |
| Flash : `requires chip revision [v3.1-v3.99] (this chip is v1.3)` | Puce pré-série ; support <3.0 déjà activé dans `sdkconfig.defaults`. Supprimer `sdkconfig`, rebuilder. |
| `esptool introuvable` au flash | `pip install esptool` sur le poste Windows. |
| Un seul moteur tourne | Câblage du canal MDD10A concerné (PWM/DIR, GND commun), ou GPIO réservée. Permuter les moteurs en sortie pour isoler moteur vs voie de commande. |
| Moniteur illisible (`~…XRCE…`) | Normal en série : UART0 partagé logs+transport. |
| Agent : port occupé / pas de connexion | Fermer le moniteur avant l'agent (UART0). Vérifier le bon `/dev/tty*` (WSL : `usbipd attach`). |
| Topics absents (`ros2 topic list` vide) | L'agent n'est pas lancé, ou mauvais transport/port/baud. |

## Structure du dépôt

```
controllers/     firmware ESP-IDF, UN DOSSIER PAR CONTRÔLEUR (voir controllers/README.md)
  common/          code partagé : base/ (main.c, config.h, kin.h, pid, imu, encoders,
                   transport), kin_diffdrive/, kin_ackermann/, profils sdkconfig
  mowbot_p4/       robot A, diffdrive, Waveshare ESP32-P4-ETH, 12 V — calibré
  mowbot_wroom/    robot B, diffdrive, ESP32-WROOM DevKitC, 24 V
  ackerbot_p4/     robot Ackermann, ESP32-P4 — jamais flashé, placeholders
scripts/         build.sh / flash.sh <contrôleur> (Linux/WSL2) ; .ps1 non adaptés
robot/           côté SBC (Jetson) : bin/, config/ (nav2, slam, ekf), nodes/, launch/, systemd/
pc/              côté PC : waypoints.py, outils RViz
ros2/            ancien dossier SBC (EKF, bringup) — voir robot/ pour l'actuel
components/      composant micro-ROS (cloné ou copié au 1er build, gitignoré)
CLAUDE.md        contexte & décisions d'architecture
COMMANDES.md     mémo des commandes courantes
HANDOFF.md       note de passation (état, point de reprise)
```

## Feuille de route

- [x] Diffdrive : `/cmd_vel` → PID → MDD10A, odométrie → `/odom`
- [x] IMU ICM-42688 → `/imu/data_raw` à 100 Hz (calib biais gyro au boot ; démarre sans IMU si absente)
- [x] Transport Ethernet (profil `sdkconfig.eth`, PHY IP101 de la carte EV)
- [x] Côté SBC : robot_localization (EKF vitesses odom + gyro yaw) — voir [ros2/](ros2/)
- [ ] Bascule série → Ethernet/UDP pour le produit final (IP de l'agent dans `sdkconfig.eth`)
- [ ] GPS RTK (u-blox ZED-F9P) — position principale en extérieur
- [ ] Coverage planning (opennav_coverage / Fields2Cover)
- [ ] Contrôle lame + capteurs de sécurité (soulèvement, bumper)
- [ ] nav2
```
