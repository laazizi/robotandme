# mowbot — mémo des commandes

Deux machines :

| | Adresse | Rôle |
|---|---|---|
| **Jetson** (robot) | `ssh nvidia@192.168.1.26` (mdp `nvidia`) | tout le logiciel embarqué |
| **PC** | `~/perso/code/robot/mowbot/pc` | RViz, pilotage, développement |

> ⚠️ **Deux SBC ont servi** à ce robot : une Jetson Orin Nano et une Jetson NX
> (relevée à `192.168.1.33` en septembre 2026). L'IP ci-dessus est celle notée
> dans ce mémo ; elle **change avec le réseau WiFi** et selon la carte branchée.
> Vérifier laquelle est en service avant de s'y fier — la méthode par adresse
> MAC ci-dessous est la seule fiable.

> L'IP du Jetson change avec le réseau WiFi. Pour la retrouver :
> ```bash
> PFX=192.168.1   # préfixe du réseau courant
> for i in $(seq 1 254); do (ping -c1 -W1 $PFX.$i >/dev/null 2>&1 &); done; sleep 4
> ip neigh | grep -i 48:8f:4c        # MAC du Jetson
> ```
> Secours infaillible : câble USB-C PC↔Jetson → `ssh nvidia@192.168.55.1`

---

## 🚀 Démarrage normal

**Tout se lance automatiquement au boot du Jetson** (agent moteurs, gyro, EKF,
lidar, TF, modèle, rosbridge, joystick web, SLAM + nav2).
Il n'y a donc **rien à taper** sur le Jetson.

Sur le **PC**, pour voir et piloter :
```bash
cd ~/perso/code/robot/mowbot/pc
./robot_nav.sh        # RViz : carte, scan lidar, plan, robot
```
Dans RViz : **2D Goal Pose** = envoyer le robot quelque part.
**2D Pose Estimate** = lui dire où il est (s'il s'est perdu).

**Joystick** (navigateur PC ou téléphone) : http://192.168.1.26:8080/joystick.html

---

## 🤖 Commandes Jetson (`ssh nvidia@192.168.1.26`)

| Commande | Effet |
|---|---|
| `bash ~/robot_status.sh` | **état complet** (services, topics, carte) — à lancer en cas de doute |
| `bash ~/robot_up.sh` | **tout (re)démarrer** si quelque chose manque |
| `bash ~/save_map.sh` | **sauvegarder** la carte actuelle |
| `bash ~/new_map.sh` | **repartir sur une carte vierge** (voir plus bas) |
| `bash ~/restart_slam.sh` | relancer le SLAM seul |
| `bash ~/banc.sh` | banc de test moteurs/encodeurs (roues en l'air) |

Dépannage ponctuel :
```bash
sudo systemctl restart mowbot-agent     # moteurs ne répondent plus
sudo systemctl restart mowbot-lidar     # plus de /scan
journalctl -u mowbot-nav -f             # logs de la navigation
```

---

## 🗺️ Cartes : comment ça marche

Au démarrage, le robot regarde s'il existe une carte enregistrée :

- **Carte présente** → mode **LOCALISATION** : il charge la carte et **se retrouve
  tout seul** dedans. La carte ne s'enrichit plus.
- **Pas de carte** → mode **CARTOGRAPHIE** : il construit une carte neuve en roulant.

### Cartographier un nouvel endroit (repartir de zéro)

```bash
ssh nvidia@192.168.1.26
bash ~/new_map.sh          # archive l'ancienne carte + repart en cartographie
```
Puis :
1. **Pilote le robot** (joystick) doucement dans tout le lieu à cartographier
2. Regarde la carte se dessiner dans RViz (`./robot_nav.sh` sur le PC)
3. Fais des **boucles** (repasse aux mêmes endroits) : ça permet au SLAM de
   recaler l'ensemble ("fermeture de boucle")
4. Quand la carte te plaît :
   ```bash
   bash ~/save_map.sh
   ```
5. Au prochain démarrage, le robot rechargera cette carte automatiquement.

> Les anciennes cartes ne sont pas perdues : `new_map.sh` les archive dans
> `~/maps/archive_<date>/`. Pour en restaurer une :
> ```bash
> cp ~/maps/archive_20260725_2327/mowbot.* ~/maps/ && bash ~/restart_slam.sh
> ```

### Compléter une carte existante
Le mode localisation n'enrichit pas la carte. Pour ajouter une zone :
```bash
MOWBOT_SLAM_MODE=mapping bash ~/run_slam.sh   # force la cartographie
# ... rouler dans la nouvelle zone ...
bash ~/save_map.sh
```

---

## 🔧 Firmware ESP32 (depuis le PC)

**Un contrôleur = un dossier** dans `controllers/` ; les réglages de chaque robot
sont dans `controllers/<robot>/main/robot.h`, le code commun dans `controllers/common/`.

```bash
cd ~/perso/code/robot/mowbot
source ~/esp/esp-idf/export.sh

./scripts/build.sh mowbot_p4      # robot 12 V  (ESP32-P4)  -> controllers/mowbot_p4/build/mowbot_p4.bin
./scripts/build.sh mowbot_wroom   # robot 24 V  (DevKitC)   -> controllers/mowbot_wroom/build/mowbot_wroom.bin
./scripts/build.sh ackerbot_p4    # Ackermann   (ESP32-P4)  -> controllers/ackerbot_p4/build/ackerbot_p4.bin
```

Flasher la DevKitC branchée sur le Jetson (le script `flash32.sh` côté Jetson
attend `mowbot.bin` : on garde ce nom à destination) :
```bash
scp controllers/mowbot_wroom/build/mowbot_wroom.bin nvidia@192.168.1.26:/home/nvidia/fw32/mowbot.bin
ssh nvidia@192.168.1.26 'bash ~/flash32.sh'
```

⚠️ Changer de **cible** (P4 ↔ DevKitC) recompile la lib micro-ROS (~10-15 min) ;
passer de `mowbot_p4` à `ackerbot_p4` (même puce) ne la recompile pas.

---

## ⚠️ Pièges connus

- **Ne pas déplacer les prises USB** sur le Jetson : le lidar et la DevKitC ont
  la même puce CP2102 et sont distingués par **port physique**.
- **Un seul robot allumé à la fois** sur le réseau (sinon deux `/odom`).
- Après un **changement de WiFi**, redémarrer les services du Jetson
  (`bash ~/robot_up.sh`) : les participants DDS restent liés à l'ancienne IP.
- Si tu **modifies le châssis** (antennes, capteurs), relance
  `bash ~/run_detect_self.sh` : il indique les secteurs où le lidar se voit
  lui-même, à reporter dans `~/scan_fix.py`.
- Le robot 24 V est **lent** (~0,055 m/s max) : un trajet de 2 m prend ~40 s.
  C'est mécanique, pas un bug.
