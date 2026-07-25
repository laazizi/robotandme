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

#define PIN_IMU_SDA       7
#define PIN_IMU_SCL       8

// -- geometrie robot 12 V (calibree au sol) --
#define WHEEL_RADIUS_M        0.0698f  // rayon EFFECTIF [m] : nominal 0.075 (Ø15) corrige
                                       // (odom 29 cm pour 27 reels, pneu ecrase sous charge).
#define TRACK_WIDTH_M         0.59f    // entraxe EFFECTIF [m] : physique 0.43 + patinage rotation
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
#define TRACK_WIDTH_M         0.48f    // entraxe MESURE (centre a centre des roues).
                                       // Etait 0.59 (herite du chassis 12 V) : la
                                       // cinematique inverse faisait alors sur-tourner
                                       // le robot de ~23 % sur chaque consigne angulaire.
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

// IMU ICM-42688-P en I2C (adresse 0x68 si AD0 à GND, 0x69 sinon)
#define IMU_I2C_ADDR      0x68

// ============================================================
// Contrôle
// ============================================================
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
