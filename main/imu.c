#include "imu.h"
#include "config.h"

#include <math.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imu";

// Registres ICM-42688-P (bank 0)
#define REG_DEVICE_CONFIG  0x11
#define REG_ACCEL_DATA_X1  0x1F  // 12 octets : accel XYZ puis gyro XYZ, big-endian
#define REG_PWR_MGMT0      0x4E
#define REG_GYRO_CONFIG0   0x4F
#define REG_ACCEL_CONFIG0  0x50
#define REG_WHO_AM_I       0x75
#define ICM42688_WHO_AM_I  0x47

// ±2000 dps → 16.4 LSB/(°/s) ; ±4 g → 8192 LSB/g
#define GYRO_SCALE  ((1.0f / 16.4f) * ((float)M_PI / 180.0f))
#define ACCEL_SCALE (9.80665f / 8192.0f)

#define I2C_TIMEOUT_MS 50

static i2c_master_dev_handle_t s_dev;
static bool s_present;
static float s_gyro_bias[3];

static bool reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_TIMEOUT_MS) == ESP_OK;
}

static bool reg_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, I2C_TIMEOUT_MS) == ESP_OK;
}

static bool read_raw(int16_t raw[6])
{
    uint8_t buf[12];
    if (!reg_read(REG_ACCEL_DATA_X1, buf, sizeof(buf))) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        raw[i] = (int16_t)((buf[2 * i] << 8) | buf[2 * i + 1]);
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
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) {
        ESP_LOGW(TAG, "bus I2C KO");
        return false;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &s_dev) != ESP_OK) {
        return false;
    }

    uint8_t who = 0;
    if (!reg_read(REG_WHO_AM_I, &who, 1) || who != ICM42688_WHO_AM_I) {
        ESP_LOGW(TAG, "ICM-42688 absente (WHO_AM_I=0x%02x) — firmware sans IMU", who);
        return false;
    }

    reg_write(REG_DEVICE_CONFIG, 0x01);          // soft reset
    vTaskDelay(pdMS_TO_TICKS(2));
    reg_write(REG_GYRO_CONFIG0, 0x08);           // ±2000 dps, ODR 100 Hz
    reg_write(REG_ACCEL_CONFIG0, 0x48);          // ±4 g, ODR 100 Hz
    reg_write(REG_PWR_MGMT0, 0x0F);              // gyro + accel en low-noise
    vTaskDelay(pdMS_TO_TICKS(50));               // démarrage gyro (~30 ms)

    s_present = true;
    ESP_LOGI(TAG, "ICM-42688 détectée et configurée");
    return true;
}

void imu_calibrate_gyro(void)
{
    if (!s_present) {
        return;
    }
    ESP_LOGI(TAG, "calibration du biais gyro (%d échantillons, robot immobile)...",
             IMU_GYRO_CALIB_SAMPLES);

    double sum[3] = { 0 };
    int valid = 0;
    for (int i = 0; i < IMU_GYRO_CALIB_SAMPLES; i++) {
        int16_t raw[6];
        if (read_raw(raw)) {
            sum[0] += raw[3] * GYRO_SCALE;
            sum[1] += raw[4] * GYRO_SCALE;
            sum[2] += raw[5] * GYRO_SCALE;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (valid > 0) {
        for (int i = 0; i < 3; i++) {
            s_gyro_bias[i] = (float)(sum[i] / valid);
        }
    }
    ESP_LOGI(TAG, "biais gyro : [%.4f %.4f %.4f] rad/s",
             s_gyro_bias[0], s_gyro_bias[1], s_gyro_bias[2]);
}

bool imu_read(imu_sample_t *out)
{
    if (!s_present) {
        return false;
    }
    int16_t raw[6];
    if (!read_raw(raw)) {
        return false;
    }
    out->ax = raw[0] * ACCEL_SCALE;
    out->ay = raw[1] * ACCEL_SCALE;
    out->az = raw[2] * ACCEL_SCALE;
    out->gx = raw[3] * GYRO_SCALE - s_gyro_bias[0];
    out->gy = raw[4] * GYRO_SCALE - s_gyro_bias[1];
    out->gz = raw[5] * GYRO_SCALE - s_gyro_bias[2];
    return true;
}
