#pragma once

#include "sdkconfig.h"

// ============================================================
// Broches — selection AUTOMATIQUE selon la cible de compilation :
//   idf.py set-target esp32p4  -> Waveshare ESP32-P4-ETH   (câblage valide)
//   idf.py set-target esp32    -> ESP32-WROOM-32U DevKitC V4 (à câbler)
// ============================================================

// IMPORTANT : chaque cible = un ROBOT DIFFERENT (pins, geometrie, gains PID).
// Ne JAMAIS toucher au bloc P4 en reglant le robot 24 V, et inversement.

#if CONFIG_IDF_TARGET_ESP32P4
// ============================================================
//  ROBOT A — Waveshare ESP32-P4-ETH + moteurs 12 V
//  ETAT : CALIBRE ET VALIDE (carres a +/-0.4 cm, coins a +/-1 deg).
//  >>> NE RIEN MODIFIER ICI <<<
// ============================================================
// MDD10A rev 2.0. Seuls les pins du header DROIT sont libres (le gauche est
// reserve SD/camera/C6). GPIO46 MORT, GPIO48 inaccessible.
// M1/M2 etaient inverses vs realite physique -> L = canal M2.
#define PIN_MOTOR_L_PWM   22
#define PIN_MOTOR_L_DIR   23
#define PIN_MOTOR_R_PWM   20
#define PIN_MOTOR_R_DIR   21

#define PIN_ENC_L_A       27     // Bleu
#define PIN_ENC_L_B       47     // Orange
#define PIN_ENC_R_A       33     // Bleu
#define PIN_ENC_R_B       32     // Orange

// I2C de l'IMU sur 3/2 (et non les 7/8 par defaut) pour regrouper le cablage
// sur le meme header que moteurs et encodeurs. Choix VERIFIE :
//  - sur l'ESP32-P4 les broches de strapping sont GPIO34..38, PAS 0..3 comme
//    sur l'ESP32 classique : 2 et 3 sont donc des GPIO ordinaires, sans effet
//    au demarrage ;
//  - elles n'apparaissent dans aucune reservation de la carte (Ethernet
//    31/50/51/52, audio 9..13, ampli 53) ;
//  - aucun conflit avec le projet, contrairement a 47 qui porte l'encodeur
//    gauche et a 48 note inaccessible au bring-up.
// L'I2C est librement placable : le P4 dispose d'une matrice de commutation.
#define PIN_IMU_SDA       3
#define PIN_IMU_SCL       2

// -- geometrie robot 12 V (calibree au sol) --
#define WHEEL_RADIUS_M        0.0753f  // rayon EFFECTIF [m] — RECALIBRE au sol.
                                       // Mesure (nodes/calib_1m.py) : l'odometrie
                                       // annoncait 102.9 cm pour 111 cm reels, soit
                                       // une SOUS-estimation de 7.3 %.
                                       //   0.0698 x 111/102.9 = 0.0753
                                       // Proche du nominal 0.075 (Ø15 cm), ce qui
                                       // conforte la mesure.
                                       // Historique : 0.0698 venait d'une mesure
                                       // inverse (odom 29 cm pour 27 reels, pneu
                                       // ecrase). L'ecart de sens indique un
                                       // changement de roues, de pneus ou de charge
                                       // depuis cette premiere calibration.
#define TRACK_WIDTH_M         0.4607f  // entraxe [m] — RECALIBRE, et confirme par
                                       // TROIS methodes independantes :
                                       //   mesure au metre              : 0.46
                                       //   gyro + encodeurs             : 0.4653
                                       //   rotations reelles au repere  : 0.4607
                                       // Affinage final (nodes/turn360.py) : a 0.465
                                       // le robot depassait de 3 deg sur un tour et de
                                       // 10 deg sur trois, soit ~0.9 % de trop.
                                       //   0.465 x 1080/1090 = 0.4607
                                       // Les deux mesures concordant (0.83 % et
                                       // 0.93 %), l'ecart etait reel et non du bruit
                                       // de lecture -- un seul tour n'aurait pas
                                       // permis de trancher.
                                       // L'ancien 0.59 sur-estimait de 27 % : les
                                       // consignes angulaires etaient donc fausses
                                       // d'autant (le robot tournait trop).
                                       //
                                       // METHODE (nodes/calib_track.py) : comparer la
                                       // rotation du GYRO a celle que les vitesses de
                                       // roue MESUREES impliquent, et non a la
                                       // consigne. Sinon on melange deux causes : ici
                                       // les roues ne font que 83 % de leur consigne,
                                       // ce qui ferait conclure a tort que l'entraxe
                                       // est trop PETIT alors qu'il etait trop GRAND.
#define TICKS_PER_WHEEL_REV   2560.0f  // 64 CPR x4 quadrature x reducteur 10:1

#define MOTOR_L_INVERT    0
#define MOTOR_R_INVERT    0
#define ENC_L_INVERT      0      // suit l'echange L<->R (ex-droit, pins 27/47)
#define ENC_R_INVERT      1      // suit l'echange L<->R (ex-gauche, pins 33/32)

// -- gains PID robot 12 V (regles sur le robot) --
#define MAX_WHEEL_SPEED_MPS_ROBOT  1.0f
#define FF_GAIN  0.0f                       // pas de feed-forward (PID valide tel quel)
#define PID_KP  0.8f
#define PID_KI  2.0f
#define PID_KD  0.0f

#else
// ============================================================
//  ROBOT B — ESP32-WROOM-32U DevKitC V4 + moteurs 24 V
//  ETAT : en cours de calibration (ticks a mesurer, gains a affiner).
// ============================================================
// Pins choisis pour eviter les pieges du classique :
//  - PAS de 6..11 (flash SPI), PAS de 0/2/12/15 (strapping au boot),
//  - PAS de 34..39 pour les encodeurs (input-only SANS pull-up interne,
//    or nos encodeurs open-collector exigent les pull-ups internes),
//  - I2C sur les pins standard 21/22.
// SENS DE MARCHE INVERSE en logiciel (l'ancien arriere devient l'avant) :
// il faut ECHANGER les roles G<->D **et** inverser les deux moteurs. Inverser
// seulement les moteurs retournerait aussi le sens de rotation (gauche<->droite).
#define PIN_MOTOR_L_PWM   16     // ex-droit
#define PIN_MOTOR_L_DIR   17
#define PIN_MOTOR_R_PWM   25     // ex-gauche
#define PIN_MOTOR_R_DIR   26

// Encodeurs : chacun suit SON moteur (groupes echanges de la meme facon).
#define PIN_ENC_L_A       32     // Bleu   (roue du moteur L : pins 16/17)
#define PIN_ENC_L_B       33     // Orange
#define PIN_ENC_R_A       18     // Bleu   (roue du moteur R : pins 25/26)
#define PIN_ENC_R_B       19     // Orange

#define PIN_IMU_SDA       21
#define PIN_IMU_SCL       22

// -- geometrie robot 24 V --
#define WHEEL_RADIUS_M        0.0698f  // rayon effectif calibre (odom 1 m = 1 m reel)
#define TRACK_WIDTH_M         0.38f    // entraxe EFFECTIF [m] — CALIBRE, pas mesure.
                                       // Physique : 0.48 (centre a centre). Mais a
                                       // 0.48 le robot sur-tournait de 26 % (mesure
                                       // gyro ET recoupee au lidar/SLAM : 0 % d'ecart
                                       // entre les deux, donc mesure fiable). Le robot
                                       // pivote plus facilement que sa geometrie ne le
                                       // predit (pneus larges, roue folle, appui sol).
                                       // 0.38 = valeur qui rend les consignes justes.
                                       // Historique : 0.59 (chassis 12 V) -> +56 %.
#define TICKS_PER_WHEEL_REV   3200.0f  // MESURE (ros2/tick_count.py, 1 tour de roue) :
                                       // gauche 3292, droite 3091 -> ~3200
                                       // = 160 CPR x4 quadrature x reducteur 5:1

// Les DEUX moteurs inverses (avec l'echange G<->D ci-dessus = marche inversee).
#define MOTOR_L_INVERT    1
#define MOTOR_R_INVERT    1
// Chaque encodeur garde la coherence avec SON moteur : signe precedent inverse.
#define ENC_L_INVERT      0      // ex-R (etait 1) -> inverse -> 0
#define ENC_R_INVERT      1      // ex-L (etait 0) -> inverse -> 1

// -- dynamique robot 24 V : moteurs a fort couple TRES LENTS --
// Vitesse max MESUREE a pleine puissance : 0.055 m/s (7.5 tr/min roue).
// -> MAX_WHEEL_SPEED recale (le duty feed-forward = cible/MAX est alors juste)
// -> FEED-FORWARD indispensable : a ces echelles (erreurs ~0.05 m/s) un PID
//    seul demande des gains enormes ou met 10 s a monter. Le FF envoie
//    d'emblee le bon duty, le PID ne fait qu'affiner (charge, pente).
#define MAX_WHEEL_SPEED_MPS_ROBOT  0.06f   // legerement > max pour garder toute l'echelle
#define FF_GAIN  1.0f                       // feed-forward actif
#define PID_KP  3.0f                        // gains de CORRECTION (autour du FF)
#define PID_KI  5.0f
#define PID_KD  0.0f
#endif

// ============================================================
// IMU — deux modules supportes, reconnus automatiquement (cf. imu.c)
// ============================================================
// ICM-42688-P : 0x68 si AD0 à GND, 0x69 sinon
#define IMU_I2C_ADDR      0x68

// GY-801 (module 10 DoF) : puces separees sur le meme bus.
//   L3G4200D  gyro  : 0x69 (strap SDO à VCC) ou 0x68 selon le lot
//   ADXL345   accel : 0x53 (strap ALT à GND) ou 0x1D
// Le HMC5883L (magnetometre) et le BMP085 (barometre) du module ne sont PAS
// lus : le magnetometre est inutilisable a proximite des moteurs, et
// l'altitude ne sert a rien pour une tondeuse.
//
// >>> ALIMENTER LE MODULE EN 3.3 V <<<
// Ses resistances de tirage I2C sont reliees a son VCC. En 5 V elles
// tireraient SDA/SCL a 5 V, or le P4 n'est PAS tolerant 5 V : les broches
// seraient detruites.
#define GY801_GYRO_ADDR       0x69
#define GY801_GYRO_ADDR_ALT   0x68
#define GY801_ACCEL_ADDR      0x53

// ============================================================
// Contrôle
// ============================================================
// Debit du transport micro-ROS. DOIT correspondre a MOWBOT_ESP32_BAUD dans
// robot/bin/mowbot_env.sh (l'agent doit ouvrir le port au meme debit).
//
// 115200 bauds = 11520 octets/s, or les messages ROS sont volumineux :
// Odometry ~730 o (dont 576 rien que pour les deux covariances 6x6 en double)
// et Imu ~330 o. Publier /odom a 10 Hz et /imu a 20 Hz demande donc
// ~13900 o/s : le lien etait sature, et les DEUX topics tombaient sous leur
// cible (mesure : /odom 2.9 Hz au lieu de 10, /imu 13 Hz au lieu de 20).
// 460800 bauds donne 46080 o/s, soit 3x de marge encapsulation comprise.
#define SERIAL_BAUDRATE       460800

#define CONTROL_PERIOD_MS     20      // boucle PID à 50 Hz (fluide)
#define ODOM_PUBLISH_DIV      5       // publie /odom 1 cycle sur 5 -> 10 Hz (limite débit série 115200)
#define IMU_PERIOD_MS         50      // publication /imu/data_raw à 20 Hz
#define IMU_GYRO_CALIB_SAMPLES 200    // ~1 s de calibration biais gyro au boot
#define CMD_VEL_TIMEOUT_MS    500     // deadman : moteurs coupés sans cmd_vel
#define PWM_FREQ_HZ           20000   // 20 kHz : inaudible, max supporté MDD10A rev2.0
#define MAX_WHEEL_SPEED_MPS   MAX_WHEEL_SPEED_MPS_ROBOT   // defini PAR ROBOT plus haut

// NB : les gains PID et la geometrie sont definis PAR ROBOT plus haut
// (blocs #if CONFIG_IDF_TARGET_ESP32P4 / #else).

// Test banc au boot (fait tourner chaque roue 2 s + 30 s compteurs a la main).
// 1 = banc de diagnostic (via banc.sh). 0 = boot normal SANS bouger les roues
// (obligatoire des que le robot est au sol : 40 s de roues qui tournent sinon).
#define BOOT_BENCH_TEST 0
