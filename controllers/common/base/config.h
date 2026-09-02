#pragma once

// ============================================================
// Configuration COMMUNE a tous les controleurs.
//
// Ce qui depend du robot -- broches, geometrie, inversions, gains, nom du
// noeud -- vient de robot.h, fourni par le dossier du controleur :
//     controllers/<nom>/main/robot.h
//
// UN CONTROLEUR = UN DOSSIER. Le choix du robot n'est ni un #if sur la puce
// (deux robots differents tournent sur ESP32-P4), ni une option Kconfig :
// c'est le dossier dans lequel on lance le build. Voir controllers/README.md.
// ============================================================
#include "robot.h"

#ifndef ROBOT_NODE_NAME
#error "robot.h doit definir ROBOT_NODE_NAME (nom du noeud micro-ROS)"
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

// Alias historique des diffdrive : la vitesse max de roue est definie PAR ROBOT.
#ifdef MAX_WHEEL_SPEED_MPS_ROBOT
#define MAX_WHEEL_SPEED_MPS   MAX_WHEEL_SPEED_MPS_ROBOT
#endif

// Test banc au boot : chaque cinematique fournit le sien (kin_bench_test).
// 1 = banc de diagnostic. 0 = boot normal SANS bouger les roues (obligatoire
// des que le robot est au sol : 40 s de roues qui tournent sinon).
#define BOOT_BENCH_TEST 0
