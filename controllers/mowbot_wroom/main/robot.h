#pragma once
// controllers/mowbot_wroom/main/robot.h -- tout ce qui est propre a CE robot.
// Le bloc ci-dessous est l'ancien bloc "#else" (ESP32 classique) de
// main/config.h, deplace A L'IDENTIQUE quand le firmware a ete decoupe en
// controleurs. Seule addition : le nom du noeud.
#define ROBOT_NODE_NAME "mowbot_base"

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
