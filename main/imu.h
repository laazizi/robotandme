#pragma once

#include <stdbool.h>

// IMU ICM-42688-P en I2C (nouvelle API i2c_master).
// Le firmware fonctionne sans IMU : imu_init() retourne false si absente.

typedef struct {
    float ax, ay, az;   // accélération [m/s²]
    float gx, gy, gz;   // vitesse angulaire [rad/s], biais gyro soustrait
} imu_sample_t;

// Détecte et configure l'IMU (±2000 dps, ±4 g, ODR 100 Hz).
bool imu_init(void);

// Moyenne le biais gyro pendant ~1 s — robot STRICTEMENT immobile.
void imu_calibrate_gyro(void);

bool imu_read(imu_sample_t *out);
