# Passation — état au 2026-07-09

Note de relais pour l'agent/développeur qui reprend. Contexte projet complet
dans [CLAUDE.md](CLAUDE.md) ; ce fichier ne couvre que l'avancement de session
et le point de reprise.

## Où on en est (vérifié)

Le firmware **compile, se flashe et démarre** sur la carte. Chaîne validée
jusqu'à : firmware vivant qui émet ses trames XRCE-DDS et attend l'agent.

- Build série OK via `.\scripts\build.ps1` (Docker ESP-IDF v5.5).
- Flash OK via `.\scripts\flash.ps1 -Port COM20 -Monitor`.
- Au moniteur : calibration gyro (~1 s), puis en boucle
  `micro-ros-agent injoignable, nouvel essai dans 1 s...` **entrelacé avec des
  octets binaires** (`~...XRCE...`). C'est NORMAL : UART0 est partagé
  logs + transport, les octets sont les trames XRCE-DDS. Le firmware tourne.

## Point de reprise (là où on s'est arrêté)

**Lancer le micro-ros-agent** pour fermer la boucle (topics `/cmd_vel`,
`/odom`, `/imu/data_raw` doivent apparaître dans `ros2 topic list`).

Décision en suspens — où lancer l'agent, car les commandes diffèrent :

1. **Sur le SBC/Jetson (cible finale)** : débrancher l'ESP32 du PC, le
   rebrancher sur la Jetson en USB, lancer l'agent Docker série
   (`microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200`,
   cf. [robot/README.md](robot/README.md)).
2. **Depuis ce PC Windows** : l'ESP32 reste sur COM20 ; exposer le port à WSL2
   avec `usbipd-win`, puis lancer l'agent dans WSL/Docker.

⚠️ **Fermer le moniteur (`Ctrl+]`) avant de lancer l'agent** : UART0 partagé,
un seul process peut tenir le port à la fois.

Ensuite : EKF (`ros2 launch ./ros2/bringup.launch.py`) et teleop, cf.
[robot/README.md](robot/README.md).

## Fait cette session

- **build.ps1** : check Docker via `cmd /c` (le `*> $null` de PS 5.1
  transformait le WARNING docker en erreur fatale). Génération de
  `app-colcon.meta` selon le transport (série→`custom`, eth→`udp`) — sans ça
  libmicroros restait figé sur UDP et `main.c` ne compilait pas en série.
  Reconstruction de libmicroros forcée si `app-colcon.meta` change OU si ses
  artefacts sont partiels (témoin : `include/rcl`), nettoyage dans le conteneur
  (MAX_PATH).
- **flash.ps1** : même correctif `cmd /c` sur le check esptool (commit `save`).
- **controllers/mowbot_p4/sdkconfig.defaults** : puce = **ESP32-P4 v1.3** (échantillon pré-série).
  IDF v5.5 vise v3.1+ et refusait le flash → `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y`
  + `CONFIG_ESP32P4_REV_MIN_100=y`.
- **esptool** installé sur le PC Windows (`pip install esptool`, v5.3.1).
- **Docker** : disque C: saturé (corrompait les métadonnées du démon →
  erreurs CMake trompeuses). Nettoyé + `vhdx` passé en sparse (84 Go rendus).

## Pièges à connaître (coûtés cher cette session)

- **Port COM = COM20** (pont CH343 wch.cn). COM7/COM8 sont du Bluetooth.
- **Binaire lié à la révision** : compilé pour v1.x, il ne bootera PAS sur un
  P4 v3.x de production. Retirer les 2 lignes de révision de
  `controllers/mowbot_p4/sdkconfig.defaults` (et `ackerbot_p4/`)
  et recompiler le jour du passage en silicium définitif.
- **Changer de transport** (série↔eth) reconstruit libmicroros (~15-20 min) ;
  géré par build.ps1 mais c'est long.
- **Ne jamais `docker image prune -a`** sans épingler `espressif/idf:release-v5.5` :
  ça l'a supprimée une fois (re-pull de 14 Go).
- Conteneurs `server-*` sur ce poste = travail de l'utilisateur, ne pas toucher.

## Reste à faire (au-delà de la reprise)

Voir la section « Reste à faire » de [CLAUDE.md](CLAUDE.md) : bascule
Ethernet/UDP pour le produit final (IP de l'agent dans
  `controllers/common/sdkconfig.eth`),
GPS RTK, coverage planning, contrôle lame + sécurités, nav2.
