# controllers/ — un dossier par contrôleur ESP32

Un **contrôleur** est le firmware micro-ROS d'un robot donné : une carte, un
câblage, une géométrie, une cinématique. Chaque contrôleur a son dossier, qui
est un projet ESP-IDF complet et **suffisant** : on y trouve tout ce qui est
propre au robot, et rien d'autre.

```
controllers/
├── common/                 code partagé, tiré par chaque contrôleur
│   ├── base/               main.c (micro-ROS, deadman, /odom, /imu), config.h commun,
│   │                       kin.h (interface), pid, imu, encoders, transport série
│   ├── kin_diffdrive/      cinématique 2 roues motrices      -> mowbot_p4, mowbot_wroom
│   ├── kin_ackermann/      cinématique direction + traction  -> ackerbot_p4
│   ├── sdkconfig.defaults  options communes (FreeRTOS)
│   ├── sdkconfig.serial    transport série  (défaut)
│   └── sdkconfig.eth       transport Ethernet/UDP
├── mowbot_p4/              robot A : diffdrive, Waveshare ESP32-P4-ETH, 12 V — CALIBRÉ
├── mowbot_wroom/           robot B : diffdrive, ESP32-WROOM DevKitC, 24 V
└── ackerbot_p4/            Ackermann, ESP32-P4 — jamais flashé, placeholders
```

Chaque dossier de contrôleur contient exactement :

| fichier | rôle |
|---|---|
| `main/robot.h` | broches, géométrie, inversions, gains, nom du nœud : **tout ce qui est propre au robot** |
| `main/CMakeLists.txt` | tire `common/base` et **une** cinématique : c'est là que se choisit diffdrive ou Ackermann |
| `sdkconfig.defaults` | la **cible** (`CONFIG_IDF_TARGET`) et les options propres à la puce |
| `CMakeLists.txt` | le projet ESP-IDF, pointe le composant micro-ROS partagé |

Le choix du robot n'est **ni** un `#if` sur la puce (deux robots différents
tournent sur ESP32-P4), **ni** une option de menuconfig : c'est le dossier.
Aucun `.c` de `common/` ne contient de condition sur le robot.

## Compiler, flasher

Toujours par les scripts — `idf.py` direct donne un firmware qui boote mais
délire (cf. mémoire projet). Le composant micro-ROS est compilé **par cible** ;
passer de P4 à WROOM le reconstruit (~15 min), passer de `mowbot_p4` à
`ackerbot_p4` non (même cible).

```bash
. ~/esp/esp-idf/export.sh
./scripts/build.sh mowbot_p4                 # [build|clean|menuconfig] [serial|eth]
./scripts/build.sh ackerbot_p4
./scripts/flash.sh  ackerbot_p4 /dev/ttyACM0 monitor
```

Chaque contrôleur a son propre `build/`, son `sdkconfig` et son
`app-colcon.meta`, dans son dossier (ignorés par git).

### La bibliothèque micro-ROS est partagée, et fragile

`components/micro_ros_espidf_component/libmicroros.a` est construite **une
fois pour toutes les contrôleurs**, et dépend de deux choses :

- la **cible** (RISC-V du P4, Xtensa du WROOM) ;
- le **transport RMW** : `custom` en série, `udp` en Ethernet.

Le marqueur `.microros_target` porte les deux ; changer l'un ou l'autre déclenche
une reconstruction d'un quart d'heure. Enchaîner P4, WROOM, P4 en coûte donc
deux : grouper les builds de même cible.

Le `colcon.meta` du composant est **figé sur UDP**. `build.sh` le surcharge en
écrivant un `app-colcon.meta` dans le dossier du contrôleur (le composant lit
`${PROJECT_DIR}/app-colcon.meta`). Sans lui, une bibliothèque reconstruite
depuis zéro sort en UDP et la compilation échoue sur
`CONFIG_MICRO_ROS_AGENT_IP undeclared` **même en série** — le piège qui a coûté
le plus de temps ici, parce que le script Linux ne connaissait pas ce mécanisme
et marchait par chance sur une bibliothèque déjà construite.

### Si tu déplaces le dépôt

Le cache CMake **fige le chemin absolu** du projet : après un déplacement, tout
`build/` devient inutilisable et `idf.py` refuse de démarrer sur un message peu
parlant (*Build directory … configured for project … in a different directory*).
`build.sh` détecte le décalage et nettoie le `build/` concerné tout seul — rien
à faire à la main.

## Ajouter un contrôleur

1. Copier le dossier du contrôleur le plus proche (`cp -r mowbot_p4 monrobot`).
2. Éditer `main/robot.h` : broches, géométrie, `ROBOT_NODE_NAME`.
3. Dans `main/CMakeLists.txt`, choisir la cinématique (`kin_diffdrive` ou `kin_ackermann`).
4. Dans `sdkconfig.defaults`, poser la cible.
5. `./scripts/build.sh monrobot`.

Pour une **nouvelle cinématique** (mecanum, skid-steer…) : un dossier
`common/kin_<nom>/` qui implémente les cinq fonctions de `kin.h`, et rien à
changer dans `base/`.

## Règle

Une modification dans `common/` touche **tous** les robots, y compris
`mowbot_p4` qui est calibré et validé. C'est le but — et c'est le risque.
Recompiler `mowbot_p4` après toute modification de `common/`, même si l'on ne
travaillait que sur un autre robot.
