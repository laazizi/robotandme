# mowbot

Tondeuse robot autonome sous **ROS 2**, et le dépôt de tous ses contrôleurs.
Deux moitiés, sur deux machines :

| moitié | où ça tourne | ce que c'est |
|---|---|---|
| **firmware** — [`controllers/`](controllers/README.md) | dans l'ESP32, flashé | `/cmd_vel` → PID par roue → moteurs ; encodeurs → `/odom` ; IMU → `/imu/data_raw` |
| **logiciel embarqué** — [`robot/`](robot/README.md) | sur le SBC du robot (Jetson) | agent micro-ROS, EKF, lidar, SLAM, **nav2**, joystick web, services systemd |
| outils poste de travail — [`pc/`](pc/README.md) | sur ton PC | RViz, pilotage clavier, points de passage en boucle |

Un quatrième dossier, [`docker/`](docker/Dockerfile.jazzy), construit une pile
ROS 2 **Jazzy en conteneur** : la Jetson Xavier NX est bloquée en Ubuntu 20.04
(JetPack 5.1.7 est la dernière version qui la supporte), et un conteneur permet
d'y faire tourner Jazzy quand même — le noyau reste celui de l'hôte.

L'ESP32 **ne parle pas ROS 2** : il parle XRCE-DDS au `micro-ros-agent`, qui
republie les topics dans le graphe ROS 2. C'est pour ça qu'un agent doit tourner
sur le SBC pour que le robot existe côté ROS.

## État réel du projet

Ce que le robot fait **aujourd'hui, vérifié sur le terrain** : il cartographie
en SLAM, se localise, planifie et navigue vers un point donné dans RViz, ou
enchaîne une boucle de points de passage. Tout démarre **automatiquement au boot
du Jetson** — il n'y a rien à taper sur le robot.

Trois contrôleurs cohabitent dans ce dépôt :

| contrôleur | robot | état |
|---|---|---|
| `mowbot_p4` | tondeuse 12 V, Waveshare ESP32-P4-ETH | **calibré et validé au sol** (carrés à ±0,4 cm, coins à ±1°) |
| `mowbot_wroom` | tondeuse 24 V, ESP32-WROOM DevKitC | en calibration ; **très lente**, ~0,055 m/s max, c'est mécanique |
| `ackerbot_p4` | robot Ackermann (direction + traction) | **compile, jamais flashé** — géométrie en placeholders |

## Par où commencer si tu arrives sur le projet

1. **[`CLAUDE.md`](CLAUDE.md)** — les décisions d'architecture *et leur pourquoi*.
   À lire avant de proposer un changement : beaucoup de choix qui semblent
   étranges sont le résultat d'une mesure.
2. **[`COMMANDES.md`](COMMANDES.md)** — le mémo opérationnel : démarrer, voir,
   piloter, cartographier, sauvegarder une carte.
3. Puis le README de la moitié qui t'intéresse :
   [`controllers/`](controllers/README.md), [`robot/`](robot/README.md) ou
   [`pc/`](pc/README.md).

### Ce qu'il ne faut pas casser

`controllers/mowbot_p4/main/robot.h` porte des constantes **calibrées au sol,
recoupées par trois méthodes indépendantes** — entraxe, rayon de roue, ticks
par tour. Les modifier fausse toutes les consignes du robot. Le fichier le dit
en tête : *ne rien modifier ici*.

Et `controllers/common/` est **partagé par les trois robots** : une modification
là touche la tondeuse validée. Recompiler `mowbot_p4` après tout changement de
`common/`, même en travaillant sur un autre robot.

## Compiler et flasher le firmware

**Toujours par les scripts.** `idf.py` lancé à la main produit un firmware qui
boote mais délire — le script gère la cible, le transport RMW et la bibliothèque
micro-ROS, trois choses qui doivent rester cohérentes.

```bash
. ~/esp/esp-idf/export.sh
./scripts/build.sh mowbot_p4                      # <contrôleur> [build|clean|menuconfig] [serial|eth]
./scripts/flash.sh mowbot_p4 /dev/ttyACM0 monitor
```

Détail de la disposition, ajout d'un contrôleur et pièges de `libmicroros` :
[`controllers/README.md`](controllers/README.md).

> Les scripts `scripts/*.ps1` (Windows + Docker) datent d'une disposition
> antérieure et **n'ont pas été adaptés** aux contrôleurs. La voie de référence
> est Linux / WSL2 avec ESP-IDF v5.5 natif.

## Déployer sur le robot

```bash
scp -r robot nvidia@<ip-du-sbc>:~/mowbot_src
ssh nvidia@<ip-du-sbc> 'bash ~/mowbot_src/install.sh'
```

L'installateur détecte la distro ROS, installe les paquets manquants, génère les
services systemd au nom de l'utilisateur courant et **écrit les règles udev de
cette machine**. Rien n'est codé en dur : voir [`robot/README.md`](robot/README.md).

⚠️ Le lidar et l'ESP32 du robot 24 V partagent la même puce USB (CP2102) et sont
distingués par **port physique**. Ne pas déplacer les prises USB du Jetson sans
relancer `mowbot detect`.

## Matériel

| élément | référence |
|---|---|
| MCU (robot A) | **Waveshare ESP32-P4-ETH** — seul le header droit est libre ; GPIO 5, 6, 15, 16 et 46 morts, 48 inaccessible |
| MCU (robot B) | ESP32-WROOM-32U DevKitC V4 |
| driver moteurs | Cytron MDD10A rev 2.0 — sign-magnitude (PWM 20 kHz + DIR), logique 3,3 V directe, 2×10 A |
| encodeurs | quadrature A/B, comptage **matériel PCNT** |
| IMU | GY-801 ou ICM-42688-P en I2C, reconnue automatiquement |
| lidar | LD14 (nœud natif) ou LSLidar N10 |

**Le câblage exact de chaque robot est dans son `robot.h`**, avec le pourquoi de
chaque broche — c'est la seule source de vérité, et elle est validée au banc :

- [`controllers/mowbot_p4/main/robot.h`](controllers/mowbot_p4/main/robot.h)
- [`controllers/mowbot_wroom/main/robot.h`](controllers/mowbot_wroom/main/robot.h)
- [`controllers/ackerbot_p4/main/robot.h`](controllers/ackerbot_p4/main/robot.h)

> ⚠️ **Aucun de ces MCU n'est tolérant 5 V.** Alimenter l'IMU en 3,3 V : ses
> résistances de tirage I2C sont reliées à son VCC et détruiraient les broches.

## Cadences

| topic | fréquence | pourquoi |
|---|---|---|
| boucle PID | 50 Hz | fluidité de l'asservissement |
| `/odom` | 10 Hz | 1 cycle sur 5 — un message Odometry pèse ~730 o, dont 576 de covariances |
| `/imu/data_raw` | 20 Hz | idem, ~330 o par message |
| lien série | 460 800 bauds | à 115 200 le lien saturait et **les deux** topics tombaient sous leur cible |

Le débit du firmware (`controllers/common/base/config.h`) et celui de l'agent
(`robot/bin/mowbot_env.sh`) **doivent être identiques**, sinon l'agent ne
dialogue pas du tout.

## Sécurité

- **Deadman firmware** : moteurs coupés si `/cmd_vel` cesse pendant 500 ms
  (agent déconnecté, teleop fermé). En Ackermann, roues remises droites en plus.
- L'**arrêt d'urgence** et la **coupure lame** devront être **matériels**,
  indépendants de ce firmware. Ce n'est pas fait.
- Le banc de test (`BOOT_BENCH_TEST` dans `config.h`) vaut **0** : le boot ne
  bouge pas les roues. Ne le passer à 1 que **roues en l'air**.

## Dépannage

| symptôme | cause probable / solution |
|---|---|
| moteurs muets, `/odom` absent | `sudo systemctl restart mowbot-agent`. Un conteneur agent resté en vie tient le port : le script le supprime avant de réveiller l'ESP32 |
| plus de `/scan` | `sudo systemctl restart mowbot-lidar` |
| nœuds visibles mais tout est muet | segment `/dev/shm` corrompu (fichier `fastrtps` de 0 octet) : `mowbot restart` |
| après un changement de WiFi, rien ne répond | les participants DDS restent liés à l'ancienne IP : `bash ~/robot_up.sh` sur le Jetson |
| `CONFIG_MICRO_ROS_AGENT_IP undeclared` à la compilation | `libmicroros` construite pour le mauvais transport RMW. `build.sh` le gère par `app-colcon.meta` ; relancer |
| `relocations in generic ELF (EM: 243)` au link | `libmicroros` d'une autre architecture. `build.sh` le détecte par `.microros_target` |
| moniteur illisible (`~…XRCE…`) | normal en série : UART0 porte les logs **et** le transport |
| port occupé au flash | fermer le moniteur (`Ctrl+]`) et arrêter l'agent avant `flash.sh` |
| `could not read Username` pendant un build | GitHub refuse le clonage anonyme sur cette machine ; `build.sh` contourne (voir `controllers/README.md`) |
| flash : `requires chip revision v3.1+` | puce P4 pré-série v1.3 ; support activé dans `controllers/mowbot_p4/sdkconfig.defaults`. ⚠️ un binaire ainsi compilé **ne bootera pas** sur une puce v3.x de production |

## Structure du dépôt

```
controllers/       firmware ESP-IDF — UN DOSSIER PAR CONTRÔLEUR
  common/base/       main.c, config.h commun, kin.h, pid, imu, encodeurs, transport
  common/kin_*/      une cinématique par dossier (diffdrive, ackermann)
  <robot>/           main/robot.h + choix de la cinématique et de la cible
scripts/           build.sh / flash.sh <contrôleur>
robot/             ce qui est déployé sur le SBC : bin/ config/ nodes/ launch/ systemd/ www/
pc/                RViz, teleop, waypoints.py
docker/            image ROS 2 Jazzy pour un SBC bloqué en Ubuntu 20.04 (Jetson NX)
components/        composant micro-ROS (cloné au 1er build, gitignoré, ~390 Mo)
CLAUDE.md          décisions d'architecture et leur pourquoi
COMMANDES.md       mémo opérationnel
```

## Feuille de route

- [x] Diffdrive : `/cmd_vel` → PID → MDD10A, odométrie → `/odom`
- [x] IMU → `/imu/data_raw`, calibration du biais gyro au boot, démarre sans IMU
- [x] EKF côté SBC (vitesses `/odom` + gyro yaw), lidar, SLAM
- [x] **nav2** : planification et suivi de trajectoire, réglés et mesurés
- [x] Points de passage en boucle avec marqueurs RViz (`pc/waypoints.py`)
- [x] Trois contrôleurs dans un seul dépôt, code commun partagé
- [ ] Ackermann : mesurer la géométrie, câbler, bring-up au banc, config nav2 dédiée
- [ ] Bascule transport série → Ethernet/UDP pour le produit final
- [ ] GPS RTK (u-blox ZED-F9P) — position absolue en extérieur
- [ ] Coverage planning (opennav_coverage / Fields2Cover)
- [ ] Contrôle lame + capteurs de sécurité (soulèvement, bumper), en matériel

## Points ouverts connus

Ce qui est mesuré mais pas résolu, pour ne pas le redécouvrir :

- **flanc gauche aveugle** du lidar, ~50–120° : mécanique, pas logiciel.
- `robot/nodes/scan_fix.py` se bloque silencieusement (nœud Python).
- le décalage `map→odom` grandit sur les longues sessions de cartographie
  (croissance du graphe de poses) : sauver la carte et passer en localisation.
- les moteurs 12 V ne suivent la consigne qu'à ~84 % à pleine vitesse
  (`FF_GAIN` vaut 0) : marge d'amélioration côté feed-forward.
