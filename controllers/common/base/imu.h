#pragma once

#include <stdbool.h>

// IMU en I2C (nouvelle API i2c_master). DEUX MODULES SUPPORTES, reconnus
// automatiquement sur le bus :
//   ICM-42688-P : une seule puce (accel + gyro), adresse 0x68/0x69
//   GY-801      : module 10 DoF a puces separees, dont on utilise
//                 L3G4200D (gyro, 0x69) et ADXL345 (accel, 0x53).
//                 Le magnetometre HMC5883L et le barometre BMP085 du module
//                 sont ignores : le magnetometre est inutilisable a cote des
//                 moteurs, et l'altitude ne sert pas a une tondeuse.
// Le firmware fonctionne sans IMU : imu_init() retourne false si absente.

typedef enum {
    IMU_MODEL_NONE = 0,
    IMU_MODEL_ICM42688,
    IMU_MODEL_GY801,
} imu_model_t;

typedef struct {
    float ax, ay, az;   // accélération [m/s²]
    float gx, gy, gz;   // vitesse angulaire [rad/s], biais gyro soustrait
} imu_sample_t;

// Détecte et configure l'IMU présente sur le bus.
bool imu_init(void);

// Modèle réellement détecté (pour adapter les covariances publiées).
imu_model_t imu_model(void);

// Moyenne le biais gyro pendant ~1 s — robot STRICTEMENT immobile.
void imu_calibrate_gyro(void);

bool imu_read(imu_sample_t *out);
