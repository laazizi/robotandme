#include "imu.h"
#include "config.h"

#include <math.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imu";

// ---------------------------------------------------------------------------
//  ICM-42688-P (une puce : accel + gyro)
// ---------------------------------------------------------------------------
#define REG_DEVICE_CONFIG  0x11
#define REG_ACCEL_DATA_X1  0x1F  // 12 octets : accel XYZ puis gyro XYZ, big-endian
#define REG_PWR_MGMT0      0x4E
#define REG_GYRO_CONFIG0   0x4F
#define REG_ACCEL_CONFIG0  0x50
#define REG_WHO_AM_I       0x75
#define ICM42688_WHO_AM_I  0x47

// ±2000 dps → 16.4 LSB/(°/s) ; ±4 g → 8192 LSB/g
#define ICM_GYRO_SCALE  ((1.0f / 16.4f) * ((float)M_PI / 180.0f))
#define ICM_ACCEL_SCALE (9.80665f / 8192.0f)

// ---------------------------------------------------------------------------
//  GY-801 : L3G4200D (gyro) + ADXL345 (accel)
// ---------------------------------------------------------------------------
// L3G4200D
#define L3G_WHO_AM_I       0x0F
#define L3G_WHO_AM_I_VAL   0xD3
#define L3G_CTRL_REG1      0x20
#define L3G_CTRL_REG4      0x23
#define L3G_OUT_X_L        0x28
// Lecture de plusieurs octets : le bit 7 de l'adresse demande
// l'auto-incrementation. Sans lui on relit six fois le meme registre.
#define L3G_AUTO_INC       0x80
// ±500 dps a 17.5 mdps/LSB. Choix : le robot tourne a ~14 deg/s, donc ±250 dps
// suffirait et serait deux fois plus fin, mais une manipulation a la main
// saturerait la mesure -- ±500 garde de la marge sans sacrifier la resolution.
#define L3G_GYRO_SCALE  (0.0175f * ((float)M_PI / 180.0f))

// ADXL345
#define ADXL_DEVID         0x00
#define ADXL_DEVID_VAL     0xE5
#define ADXL_BW_RATE       0x2C
#define ADXL_POWER_CTL     0x2D
#define ADXL_DATA_FORMAT   0x31
#define ADXL_DATAX0        0x32
// En mode FULL_RES la sensibilite reste 256 LSB/g quelle que soit l'echelle.
#define ADXL_ACCEL_SCALE (9.80665f / 256.0f)

#define I2C_TIMEOUT_MS 50

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;        // ICM-42688, ou L3G4200D (gyro)
static i2c_master_dev_handle_t s_dev_accel;  // ADXL345 (GY-801 uniquement)
static imu_model_t s_model = IMU_MODEL_NONE;
static float s_gyro_bias[3];

// --- acces registre, sur un peripherique donne -----------------------------
static bool dev_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), I2C_TIMEOUT_MS) == ESP_OK;
}

static bool dev_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_TIMEOUT_MS) == ESP_OK;
}

static bool reg_write(uint8_t reg, uint8_t val) { return dev_write(s_dev, reg, val); }
static bool reg_read(uint8_t reg, uint8_t *d, size_t n) { return dev_read(s_dev, reg, d, n); }

static bool add_device(uint8_t addr, i2c_master_dev_handle_t *out)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    return i2c_master_bus_add_device(s_bus, &cfg, out) == ESP_OK;
}

// --- lecture brute, selon le modele ---------------------------------------
// Renvoie accel XYZ puis gyro XYZ, en unites SI.
static bool read_sample(imu_sample_t *out)
{
    if (s_model == IMU_MODEL_ICM42688) {
        uint8_t b[12];
        if (!reg_read(REG_ACCEL_DATA_X1, b, sizeof(b))) {
            return false;
        }
        int16_t r[6];
        for (int i = 0; i < 6; i++) {           // big-endian
            r[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
        }
        out->ax = r[0] * ICM_ACCEL_SCALE;
        out->ay = r[1] * ICM_ACCEL_SCALE;
        out->az = r[2] * ICM_ACCEL_SCALE;
        out->gx = r[3] * ICM_GYRO_SCALE;
        out->gy = r[4] * ICM_GYRO_SCALE;
        out->gz = r[5] * ICM_GYRO_SCALE;
        return true;
    }

    if (s_model == IMU_MODEL_GY801) {
        uint8_t g[6], a[6];
        // Les deux puces sont little-endian, contrairement a l'ICM-42688.
        if (!dev_read(s_dev, L3G_OUT_X_L | L3G_AUTO_INC, g, sizeof(g))) {
            return false;
        }
        if (!dev_read(s_dev_accel, ADXL_DATAX0, a, sizeof(a))) {
            return false;
        }
        int16_t gr[3], ar[3];
        for (int i = 0; i < 3; i++) {
            gr[i] = (int16_t)(g[2 * i] | (g[2 * i + 1] << 8));
            ar[i] = (int16_t)(a[2 * i] | (a[2 * i + 1] << 8));
        }
        out->ax = ar[0] * ADXL_ACCEL_SCALE;
        out->ay = ar[1] * ADXL_ACCEL_SCALE;
        out->az = ar[2] * ADXL_ACCEL_SCALE;
        out->gx = gr[0] * L3G_GYRO_SCALE;
        out->gy = gr[1] * L3G_GYRO_SCALE;
        out->gz = gr[2] * L3G_GYRO_SCALE;
        return true;
    }

    return false;
}

// --- initialisation par modele --------------------------------------------
static bool try_icm42688(void)
{
    if (!add_device(IMU_I2C_ADDR, &s_dev)) {
        return false;
    }
    uint8_t who = 0;
    if (!reg_read(REG_WHO_AM_I, &who, 1) || who != ICM42688_WHO_AM_I) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return false;
    }
    reg_write(REG_DEVICE_CONFIG, 0x01);          // soft reset
    vTaskDelay(pdMS_TO_TICKS(2));
    reg_write(REG_GYRO_CONFIG0, 0x08);           // ±2000 dps, ODR 100 Hz
    reg_write(REG_ACCEL_CONFIG0, 0x48);          // ±4 g, ODR 100 Hz
    reg_write(REG_PWR_MGMT0, 0x0F);              // gyro + accel en low-noise
    vTaskDelay(pdMS_TO_TICKS(50));               // démarrage gyro (~30 ms)
    ESP_LOGI(TAG, "ICM-42688 détectée et configurée");
    return true;
}

static bool try_gy801(void)
{
    // Gyroscope L3G4200D. L'adresse depend du strap SDO : 0x69 sur la plupart
    // des GY-801, 0x68 sur certains lots -> on essaie les deux.
    const uint8_t gyro_addrs[] = { GY801_GYRO_ADDR, GY801_GYRO_ADDR_ALT };
    bool gyro_ok = false;
    for (int i = 0; i < 2 && !gyro_ok; i++) {
        if (!add_device(gyro_addrs[i], &s_dev)) {
            continue;
        }
        uint8_t who = 0;
        if (dev_read(s_dev, L3G_WHO_AM_I, &who, 1) && who == L3G_WHO_AM_I_VAL) {
            gyro_ok = true;
            ESP_LOGI(TAG, "L3G4200D (gyro) sur 0x%02x", gyro_addrs[i]);
        } else {
            i2c_master_bus_rm_device(s_dev);
            s_dev = NULL;
        }
    }
    if (!gyro_ok) {
        return false;
    }

    // 100 Hz, filtre 25 Hz, les trois axes actifs, sortie de veille.
    dev_write(s_dev, L3G_CTRL_REG1, 0x1F);
    dev_write(s_dev, L3G_CTRL_REG4, 0x10);       // ±500 dps
    vTaskDelay(pdMS_TO_TICKS(20));

    // Accelerometre ADXL345 : utile mais NON indispensable. L'EKF n'exploite
    // que le gyro (le cap), donc une ADXL absente ne doit pas priver le robot
    // de son gyroscope.
    if (add_device(GY801_ACCEL_ADDR, &s_dev_accel)) {
        uint8_t id = 0;
        if (dev_read(s_dev_accel, ADXL_DEVID, &id, 1) && id == ADXL_DEVID_VAL) {
            dev_write(s_dev_accel, ADXL_DATA_FORMAT, 0x09);  // FULL_RES, ±4 g
            dev_write(s_dev_accel, ADXL_BW_RATE, 0x0A);      // 100 Hz
            dev_write(s_dev_accel, ADXL_POWER_CTL, 0x08);    // mode mesure
            ESP_LOGI(TAG, "ADXL345 (accel) sur 0x%02x", GY801_ACCEL_ADDR);
        } else {
            ESP_LOGW(TAG, "ADXL345 absente (id=0x%02x) — gyro seul", id);
            i2c_master_bus_rm_device(s_dev_accel);
            s_dev_accel = NULL;
        }
    }
    return true;
}

bool imu_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = PIN_IMU_SDA,
        .scl_io_num = PIN_IMU_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // Les modules GY-801 embarquent deja des resistances de tirage : les
        // tirages internes s'y ajoutent sans gener (ils sont bien plus faibles).
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) {
        ESP_LOGW(TAG, "bus I2C KO");
        return false;
    }

    if (try_icm42688()) {
        s_model = IMU_MODEL_ICM42688;
    } else if (try_gy801()) {
        s_model = IMU_MODEL_GY801;
        ESP_LOGI(TAG, "GY-801 détecté (magnetometre et barometre ignores)");
    } else {
        ESP_LOGW(TAG, "aucune IMU reconnue sur SDA=%d SCL=%d — firmware sans IMU",
                 PIN_IMU_SDA, PIN_IMU_SCL);
        return false;
    }
    return true;
}

imu_model_t imu_model(void)
{
    return s_model;
}

void imu_calibrate_gyro(void)
{
    if (s_model == IMU_MODEL_NONE) {
        return;
    }
    ESP_LOGI(TAG, "calibration du biais gyro (%d échantillons, robot immobile)...",
             IMU_GYRO_CALIB_SAMPLES);

    double sum[3] = { 0 };
    int valid = 0;
    for (int i = 0; i < IMU_GYRO_CALIB_SAMPLES; i++) {
        imu_sample_t s;
        // Biais mesure AVANT soustraction : read_sample() ne la fait pas,
        // c'est imu_read() qui l'applique.
        if (read_sample(&s)) {
            sum[0] += s.gx;
            sum[1] += s.gy;
            sum[2] += s.gz;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (valid > 0) {
        for (int i = 0; i < 3; i++) {
            s_gyro_bias[i] = (float)(sum[i] / valid);
        }
    }
    ESP_LOGI(TAG, "biais gyro : [%.4f %.4f %.4f] rad/s (%d/%d échantillons)",
             s_gyro_bias[0], s_gyro_bias[1], s_gyro_bias[2],
             valid, IMU_GYRO_CALIB_SAMPLES);
}

bool imu_read(imu_sample_t *out)
{
    if (s_model == IMU_MODEL_NONE || !read_sample(out)) {
        return false;
    }
    out->gx -= s_gyro_bias[0];
    out->gy -= s_gyro_bias[1];
    out->gz -= s_gyro_bias[2];
    return true;
}
