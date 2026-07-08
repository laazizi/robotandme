#pragma once

// ============================================================
// Broches — A ADAPTER au câblage réel. Vérifier le schéma de
// l'ESP32-P4-Function-EV-Board : certaines GPIO sont réservées
// (SD, MIPI, Ethernet PHY...).
// ============================================================

// MDD10A rev 2.0 — canal 1 = moteur gauche, canal 2 = moteur droit
// Logique 3,3 V directe, GND commun obligatoire avec la carte.
#define PIN_MOTOR_L_PWM   20
#define PIN_MOTOR_L_DIR   21
#define PIN_MOTOR_R_PWM   22
#define PIN_MOTOR_R_DIR   23

// Encodeurs quadrature (canaux A/B). Sorties 3,3 V uniquement :
// le P4 n'est PAS tolérant 5 V — level shifter si encodeurs 5 V.
#define PIN_ENC_L_A       45
#define PIN_ENC_L_B       46
#define PIN_ENC_R_A       47
#define PIN_ENC_R_B       48

// IMU ICM-42688-P en I2C (adresse 0x68 si AD0 à GND, 0x69 sinon)
#define PIN_IMU_SDA       7
#define PIN_IMU_SCL       8
#define IMU_I2C_ADDR      0x68

// ============================================================
// Géométrie robot — à mesurer précisément.
// TRACK_WIDTH_M est LE paramètre critique pour le cap : le
// calibrer en faisant tourner le robot sur lui-même.
// ============================================================
#define WHEEL_RADIUS_M        0.10f    // rayon roue [m]
#define TRACK_WIDTH_M         0.40f    // entraxe roues [m]
#define TICKS_PER_WHEEL_REV   1920.0f  // ticks par tour de ROUE (quadrature x4, réducteur inclus)

// Inversions selon câblage (1 pour inverser)
#define MOTOR_L_INVERT    0
#define MOTOR_R_INVERT    0
#define ENC_L_INVERT      0
#define ENC_R_INVERT      0

// ============================================================
// Contrôle
// ============================================================
#define CONTROL_PERIOD_MS     20      // boucle PID + odométrie à 50 Hz
#define IMU_PERIOD_MS         10      // publication /imu/data_raw à 100 Hz
#define IMU_GYRO_CALIB_SAMPLES 200    // ~1 s de calibration biais gyro au boot
#define CMD_VEL_TIMEOUT_MS    500     // deadman : moteurs coupés sans cmd_vel
#define PWM_FREQ_HZ           20000   // 20 kHz : inaudible, max supporté MDD10A rev2.0
#define MAX_WHEEL_SPEED_MPS   1.0f    // saturation consigne vitesse roue

// Gains PID vitesse roue (sortie en duty -1..1) — à régler sur le robot
#define PID_KP  0.8f
#define PID_KI  2.0f
#define PID_KD  0.0f
