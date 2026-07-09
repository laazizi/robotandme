# mowbot — contexte projet

Tondeuse robot autonome. Firmware micro-ROS sur **ESP32-P4-Function-EV-Board**,
qui parle à un SBC embarqué (agent micro-ROS + ROS 2 Humble).

## Décisions d'architecture (et pourquoi)

- **Diffdrive** (2 roues motrices + roues folles), pas d'Ackermann : rotation
  sur place indispensable pour la tonte en boustrophédon.
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
- **Fusion côté SBC, pas sur le MCU** : robot_localization (`ros2/ekf.yaml`)
  fusionne les VITESSES de /odom (pas la pose : patinage sur herbe) + gyro yaw.
  Le GPS RTK (u-blox ZED-F9P) sera la source de position principale.
- **Sécurité** : deadman 500 ms sur /cmd_vel dans le firmware. L'arrêt
  d'urgence et la coupure lame devront être MATÉRIELS, hors firmware.

## Points de vigilance

- Les GPIO de `main/config.h` sont des **placeholders non validés** contre le
  schéma de la carte EV (SD/MIPI/Ethernet réservent des broches).
- `TRACK_WIDTH_M`, `WHEEL_RADIUS_M`, `TICKS_PER_WHEEL_REV`, gains PID :
  à calibrer (procédure dans le README).
- Le P4 n'est pas tolérant 5 V : level shifter si encodeurs 5 V.
- UART0 partagé logs/transport série : fermer le moniteur avant l'agent.
- Le composant micro-ROS (branche humble, gitignoré dans `components/`) ne se
  compile que sous Linux → build via Docker (`scripts/build.ps1`) ou WSL2.
  Changer de transport exige un fullclean (géré par les scripts).

## Commandes

```powershell
.\scripts\build.ps1 [-Transport serial|eth] [-Clean] [-Menuconfig]
.\scripts\flash.ps1 [-Port COM5] [-Monitor]
.\scripts\monitor.ps1
```

Côté SBC : voir `ros2/README.md` (agent docker, EKF, teleop).

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
- nav2.
