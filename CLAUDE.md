# mowbot — contexte projet

Robots ROS 2 à firmware micro-ROS sur ESP32, parlant à un SBC embarqué (agent
micro-ROS + nav2). Le projet porte **plusieurs contrôleurs** : deux tondeuses
diffdrive (robot A sur Waveshare ESP32-P4-ETH, robot B sur ESP32-WROOM) et un
robot Ackermann (ESP32-P4). Un seul code commun, un dossier par contrôleur.

## Structure du firmware : `controllers/`

**Un contrôleur = un dossier**, projet ESP-IDF complet et suffisant. Le choix
du robot n'est ni un `#if` sur la puce (deux robots différents tournent sur
P4), ni une option Kconfig : c'est le dossier dans lequel on build.

- `controllers/common/base/` : `main.c` (micro-ROS, deadman, `/odom`, `/imu`),
  `config.h` commun, `kin.h` (interface de cinématique), pid, imu, encodeurs,
  transport série. **Aucune condition sur le robot dans ces fichiers.**
- `controllers/common/kin_diffdrive/`, `kin_ackermann/` : une cinématique par
  dossier, cinq fonctions (`kin_init/apply_twist/update/stop/bench_test`).
- `controllers/<robot>/main/robot.h` : broches, géométrie, gains, nom du nœud.
  `main/CMakeLists.txt` y choisit la cinématique ; `sdkconfig.defaults` la cible.
- Détail et procédure d'ajout : `controllers/README.md`.

**Règle** : toute modification de `common/` touche tous les robots, dont
`mowbot_p4` qui est calibré et validé. Recompiler `mowbot_p4` après chaque
changement de `common/`, même en travaillant sur un autre robot.

## Décisions d'architecture (et pourquoi)

- **Diffdrive pour les tondeuses** (2 roues motrices + roues folles) : rotation
  sur place indispensable pour la tonte en boustrophédon. L'Ackermann est un
  contrôleur à part (`ackerbot_p4`), voir sa section.
- **Driver moteurs Cytron MDD10A rev 2.0** : sign-magnitude (PWM 20 kHz + DIR),
  logique 3,3 V directe, 2×10 A. Pas de retour courant ni d'entrée encodeur.
- **Encodeurs en PCNT matériel** (jamais d'interruptions GPIO) : quadrature ×4
  décodée par le périphérique, zéro tick perdu, zéro CPU. Exigence utilisateur.
- **IMU ICM-42688-P** en I2C : gyro yaw = référence de cap court terme.
  Calibration du biais gyro au boot (~1 s, robot immobile). Le firmware
  démarre sans IMU si absente. Pas de magnétomètre (inutilisable près des moteurs).
- **Transports** : série (défaut, `sdkconfig.serial`) pour le debug ;
  Ethernet/UDP (`sdkconfig.eth`) pour le robot final — EMAC interne du P4 +
  PHY IP101 de la carte EV, les GPIO par défaut du composant micro-ROS sont
  déjà les bons (MDC 31, MDIO 52, RST 51). Wi-Fi exclu (P4 sans radio, jitter).
- **Fusion côté SBC, pas sur le MCU** : robot_localization (`robot/config/ekf.yaml`)
  fusionne les VITESSES de /odom (pas la pose : patinage sur herbe) + gyro yaw.
  Le GPS RTK (u-blox ZED-F9P) sera la source de position principale.
- **Sécurité** : deadman 500 ms sur /cmd_vel dans le firmware. L'arrêt
  d'urgence et la coupure lame devront être MATÉRIELS, hors firmware.

## Points de vigilance

- Les GPIO de `controllers/mowbot_p4/main/robot.h` sont **validés au banc** sur
  la Waveshare P4-ETH (seul le header droit est libre ; 5/6/15/16/46 morts,
  48 inaccessible). Ceux d'`ackerbot_p4` sont repris de là mais **non câblés**.
- `TRACK_WIDTH_M`, `WHEEL_RADIUS_M`, `TICKS_PER_WHEEL_REV`, gains PID :
  à calibrer (procédure dans le README).
- Le P4 n'est pas tolérant 5 V : level shifter si encodeurs 5 V.
- UART0 partagé logs/transport série : fermer le moniteur avant l'agent.
- Le composant micro-ROS (branche humble, gitignoré dans `components/`) ne se
  compile que sous Linux. Changer de transport ou de cible reconstruit
  `libmicroros` (~15 min), géré par `scripts/build.sh`.
- **Binaire lié à la révision de puce** : les P4 pré-série sont en v1.3 et le
  support des révisions <3.0 est activé dans `controllers/mowbot_p4/sdkconfig.defaults`
  (idem `ackerbot_p4`). Un binaire ainsi compilé **ne bootera pas** sur un P4
  v3.x de production : retirer les deux lignes `ESP32P4_REV_*` le jour du
  passage au silicium définitif.
- **Ne pas toucher aux conteneurs Docker `server-*`** sur le poste de travail :
  ils appartiennent à un autre travail de l'utilisateur. Et ne jamais lancer
  `docker image prune -a` sans épingler l'image ESP-IDF (14 Go à retélécharger).
- Les scripts `scripts/*.ps1` (Windows + Docker) datent de la disposition
  antérieure et n'ont pas été adaptés aux contrôleurs.

## Commandes

**Toujours par les scripts** : `idf.py` direct donne un firmware qui boote mais
délire (cf. mémoire). `libmicroros` est partagée et dépend de la **cible** ET du
**transport RMW** (série = `custom`, eth = `udp`) : changer l'un des deux la
reconstruit (~15 min), passer de `mowbot_p4` à `ackerbot_p4` non. Le
`colcon.meta` du composant est figé sur UDP, `build.sh` le surcharge par un
`app-colcon.meta` dans le dossier du contrôleur — sans quoi une reconstruction
depuis zéro échoue sur `CONFIG_MICRO_ROS_AGENT_IP undeclared` même en série.

```bash
. ~/esp/esp-idf/export.sh
./scripts/build.sh <mowbot_p4|mowbot_wroom|ackerbot_p4> [build|clean|menuconfig] [serial|eth]
./scripts/flash.sh <controleur> [/dev/ttyACM0] [monitor]
./scripts/build_esp32.sh          # = build.sh mowbot_wroom, conservé pour l'habitude
```

Les scripts Windows `scripts/*.ps1` datent de l'ancienne disposition (projet à
la racine) et **n'ont pas été adaptés** au découpage en contrôleurs.

Côté SBC : voir `robot/README.md` (agent docker, EKF, nav2, teleop).

## Ackermann (`ackerbot_p4`) — décisions et état

**État : compile, jamais flashé ni câblé.**

**Géométrie identique à robot A** (utilisateur, 2 septembre 2026) : même
châssis, mêmes roues, mêmes moteurs — seules les roues folles laissent place à
une roue directrice. `TRACK_WIDTH_M` (0,4607), `WHEEL_RADIUS_M` (0,0753) et
`TICKS_PER_WHEEL_REV` (2560) sont donc les valeurs **calibrées au sol** de
robot A, plus des placeholders. Restent deux inconnues, et elles ne peuvent pas
être héritées parce qu'un diffdrive n'en a pas besoin et ne les a jamais
mesurées : **`WHEELBASE_M`** (essieu moteur → contact de la roue directrice) et
**`STEER_MAX_RAD`** (butée mécanique).

**Mesuré et validé au banc le 3 septembre 2026** : `STEER_X_M = −0,36 m` (roue
directrice **derrière** l'essieu moteur — 20 cm derrière le lidar, lui-même à
−0,16 m). Servo sur **GPIO 14, qui FONCTIONNE** : résultat contre-intuitif, ses
voisines 5, 6, 15 et 16 sont mortes et j'en avais déduit à tort que 14 le
serait. Ne pas extrapoler la liste des broches mortes aux voisines.
Encodeurs et moteurs vérifiés roue par roue : aucune permutation de canaux,
les deux comptent + en avant, donc `MOTOR_L/R_INVERT=0`, `ENC_L_INVERT=0` et
`ENC_R_INVERT=1` sont justes. À commande identique la roue droite tourne
**5,1 %** plus vite que la gauche — d'où un PID par roue, pas un par essieu.

- **En réalité un TRICYCLE, pas un Ackermann** : **une seule** roue directrice
  (utilisateur, 2 septembre 2026), en attendant d'en avoir deux. Conséquence
  favorable : le modèle bicyclette devient **exact** et non approché — δ est
  l'angle physique de la roue, pas celui d'une roue virtuelle au milieu d'un
  train avant. Passer à deux roues plus tard ne changera **rien au firmware**
  tant qu'un servo unique pilote un trapèze de direction ; il faudrait le
  modifier seulement avec deux servos. Les vraies limites du tricycle sont
  mécaniques : renversement en virage et enfoncement dans l'herbe (tout le
  poids avant sur un seul point de contact).
- **Servo RC pour la direction** (PWM 50 Hz, LEDC TIMER_1 14 bits) : asservi en
  position par construction, donc aucune boucle à écrire ; mais **sans retour**,
  l'odométrie utilise l'angle *commandé* — sa première source d'erreur.
- **Course de direction 30 à 45°** (utilisateur) : donc **pas** de rotation sur
  place, et la config nav2 dédiée ci-dessous est bien obligatoire.
- **Deux moteurs arrière, un par roue, sur les DEUX canaux du MDD10A** (LEDC
  TIMER_0, 20 kHz, canaux 0 et 1), un PID par roue ; encodeurs PCNT sur les deux
  roues arrière, distance = moyenne des deux. Décision du 2 septembre 2026 : on
  réutilise la carte et le câblage de `mowbot_p4` **tels quels**, on n'ajoute
  qu'un servo. Les broches moteur/encodeurs/IMU et `ENC_R_INVERT=1` sont donc
  celles, validées au banc et calibrées au sol, de robot A.
- **Différentiel électronique OBLIGATOIRE**, puisqu'il n'y a pas de différentiel
  mécanique : `k = voie·tan(δ)/(2L)`, `v_int = v(1−k)`, `v_ext = v(1+k)`. Cette
  forme est équivalente à `(R∓voie/2)/R` mais **sans division par R**, donc sans
  singularité en ligne droite. Commander les deux roues à la même vitesse les
  ferait se battre.
- **`k` n'est pas petit sur ce châssis**, et j'ai d'abord écrit le contraire en
  me fiant à un placeholder de voie de 0,25 m. L'entraxe réel de robot A vaut
  **0,4607 m** : `k` atteint **1** — roue intérieure à l'arrêt — dès
  `δ = atan(2|x_s|/voie)`, soit **57,4°** pour `x_s = −0,36 m`. Au-delà, la roue
  intérieure doit **reculer** pendant que l'autre avance. C'est géométriquement
  correct et le matériel l'accepte (sign-magnitude), donc ce n'est pas bridé,
  mais c'est journalisé : y arriver signifie qu'on est au bout du châssis.
- **Recoupement de cap, gratuit et exact** : `w_roues = (v_d−v_g)/voie` contre
  `w_modèle = v·tan(δ)/L`. Sans patinage les deux sont identiques au bit près
  (vérifié sur 2 006 cas, marche arrière comprise, écart max 9e−16 rad/s), donc
  toute divergence signale du patinage ou un braquage réel différent du
  commandé. C'est le **seul** garde-fou sur δ, le servo étant sans retour.
  Journal rate-limité, seuil 0,35 rad/s à affiner au bring-up.
- **Modèle bicyclette** : `θ̇ = v·tan(δ)/L`, `δ = atan(ωL/v)` — juste aussi en
  marche arrière, sans correction de signe (validé par 12 tests numériques sur
  PC). Rotation sur place refusée proprement : v=0, pré-braquage, journal.
- **Rayon de braquage minimal dérivé** : `L/tan(δmax)`, à fournir à nav2 ; ne
  jamais le saisir à la main côté SBC.
- **nav2 doit changer pour ce robot** : `Spin`, `RotateToGoal`, `RotationShim`
  et DWB supposent une rotation sur place. Il faut un planificateur qui respecte
  le rayon (SmacPlannerHybrid) et un contrôleur RPP sans rotate-to-heading ou
  MPPI en modèle Ackermann. **Ne pas toucher la config nav2 de robot A** : un
  fichier de paramètres séparé.
- **`PIN_SERVO 45` est un CANDIDAT, pas une broche validée.** C'est le seul
  signal ajouté par ce robot, donc la seule broche non héritée de robot A.
  Raisonnement : inutilisée par tout contrôleur, même header que GPIO 47, jamais
  signalée morte — contrairement à 46 (muet), 48 (inaccessible) et 5/6/15/16.
  Le brochage Waveshare ne fait pas preuve ici : le header gauche est étiqueté
  « GPIO » et s'est révélé mort sur cette carte. À trancher au banc.
- Bring-up : `BOOT_BENCH_TEST 1` puis `kin_bench_test()`, roues en l'air et
  servo alimenté à part. Trois étapes : course et sens du servo (et donc
  validité de sa broche), puis **chaque roue séparément** — ce qui détecte aussi
  une permutation de canaux, déjà survenue sur robot A — puis l'affichage du `k`
  à la butée et du rayon de braquage, pour repérer une géométrie absurde avant
  de rouler.

## Reste à faire

- **Bascule transport série → Ethernet/UDP pour le produit final** : valider
  d'abord toute la chaîne en série sur le banc (teleop, /odom, IMU), puis
  renseigner l'IP réelle du SBC dans `sdkconfig.eth`
  (`CONFIG_MICRO_ROS_AGENT_IP`, actuellement placeholder 192.168.1.100) et
  rebuilder avec `-Transport eth`. Rappel : c'est l'IP de l'AGENT (le SBC) qui
  est figée à la compilation ; le robot prend la sienne en DHCP et initie la
  connexion. Ethernet retenu pour le jitter (Wi-Fi exclu, cf. décisions).
- GPS RTK (navsat_transform, emplacement prévu dans ekf.yaml).
- Coverage planning (opennav_coverage / Fields2Cover).
- Contrôle lame + capteurs de sécurité (soulèvement, bumper).
- nav2 (robot A : en cours, voir `robot/config/nav2_params.yaml`).
- Ackermann : mesurer la géométrie, calibrer le servo, câbler, bring-up au banc,
  puis config nav2 dédiée (voir section Ackermann).
