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
#define PIN_MOTOR_L_PWM   25
#define PIN_MOTOR_L_DIR   26
#define PIN_MOTOR_R_PWM   16
#define PIN_MOTOR_R_DIR   17

#define PIN_ENC_L_A       32     // Bleu   (pull-up interne OK)
#define PIN_ENC_L_B       33     // Orange
#define PIN_ENC_R_A       18     // Bleu
#define PIN_ENC_R_B       19     // Orange

#define PIN_IMU_SDA       21
#define PIN_IMU_SCL       22

// -- geometrie robot 24 V --
#define WHEEL_RADIUS_M        0.0698f  // TODO affiner : ros2/calib_distance.sh
#define TRACK_WIDTH_M         0.59f    // TODO mesurer l'entraxe reel des nouvelles roues
#define TICKS_PER_WHEEL_REV   3200.0f  // MESURE (ros2/tick_count.py, 1 tour de roue) :
                                       // gauche 3292, droite 3091 -> ~3200
                                       // = 160 CPR x4 quadrature x reducteur 5:1

#define MOTOR_L_INVERT    0
#define MOTOR_R_INVERT    0
#define ENC_L_INVERT      1      // mesure : +cmd -> gauche comptait NEGATIF (droite OK)
#define ENC_R_INVERT      0

// -- gains PID robot 24 V : ABAISSES car moteurs plus puissants que le 12 V
//    (mêmes gains = correction trop violente -> oscillation / a-coups) --
#define PID_KP  0.35f
#define PID_KI  0.7f
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
#define MAX_WHEEL_SPEED_MPS   1.0f    // saturation consigne vitesse roue

// NB : les gains PID et la geometrie sont definis PAR ROBOT plus haut
// (blocs #if CONFIG_IDF_TARGET_ESP32P4 / #else).

// Test banc au boot (fait tourner chaque roue 2 s + 30 s compteurs a la main).
// 1 = banc de diagnostic (via banc.sh). 0 = boot normal SANS bouger les roues
// (obligatoire des que le robot est au sol : 40 s de roues qui tournent sinon).
#define BOOT_BENCH_TEST 0
