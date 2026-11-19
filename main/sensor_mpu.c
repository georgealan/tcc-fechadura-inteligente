#include "sensor_mpu.h"
#include "i2c_bus.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MPU6050";

#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CFG     0x1B
#define MPU6050_REG_ACCEL_CFG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

#define I2C_TIMEOUT_MS           100

#define ACCEL_LSB_POR_G          16384.0f
#define GYRO_LSB_POR_DPS         131.0f
#define LIMIAR_IMPACTO_G         0.40f

static i2c_master_dev_handle_t s_mpu_dev = NULL;

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t val) {
    if (!i2c_bus_lock(200)) return ESP_ERR_TIMEOUT;
    uint8_t buf[2] = { reg, val };
    esp_err_t err = i2c_master_transmit(s_mpu_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
    i2c_bus_unlock();
    return err;
}

static esp_err_t mpu_read_regs(uint8_t reg_inicial, uint8_t *dados, size_t len) {
    if (!i2c_bus_lock(200)) return ESP_ERR_TIMEOUT;
    esp_err_t err = i2c_master_transmit_receive(s_mpu_dev, &reg_inicial, 1, dados, len, I2C_TIMEOUT_MS);
    i2c_bus_unlock();
    return err;
}

static bool mpu_tentar_conectar(i2c_master_bus_handle_t bus, uint8_t addr, uint8_t *who_am_i_out) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 50000,
    };

    if (!i2c_bus_lock(300)) return false;
    esp_err_t add_err = i2c_master_bus_add_device(bus, &dev_cfg, &s_mpu_dev);
    i2c_bus_unlock();
    if (add_err != ESP_OK) return false;

    uint8_t who = 0;
    esp_err_t err = mpu_read_regs(MPU6050_REG_WHO_AM_I, &who, 1);
    if (err == ESP_OK) {
        *who_am_i_out = who;
        return true;
    }

    if (i2c_bus_lock(300)) {
        i2c_master_bus_rm_device(s_mpu_dev);
        s_mpu_dev = NULL;
        i2c_bus_unlock();
    }
    return false;
}

static esp_err_t mpu_configurar_chip(void) {
    esp_err_t err = mpu_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(50));

    err = mpu_write_reg(MPU6050_REG_CONFIG, 0x03);
    if (err != ESP_OK) return err;

    err = mpu_write_reg(MPU6050_REG_ACCEL_CFG, 0x00);
    if (err != ESP_OK) return err;
    return mpu_write_reg(MPU6050_REG_GYRO_CFG, 0x00);
}

static void sensor_mpu_task(void *arg) {
    // Aguarda 800 ms para o VL53L1X terminar a inicializacao pesada de 91 registradores
    vTaskDelay(pdMS_TO_TICKS(800));

    i2c_master_bus_handle_t bus = i2c_bus_obter_handle();
    if (bus == NULL) {
        ESP_LOGE(TAG, "Barramento I2C nao inicializado!");
        vTaskDelete(NULL);
        return;
    }

    uint8_t who_am_i = 0;
    uint8_t addr_encontrado = 0;

    if (mpu_tentar_conectar(bus, 0x68, &who_am_i)) {
        addr_encontrado = 0x68;
    } else if (mpu_tentar_conectar(bus, 0x69, &who_am_i)) {
        addr_encontrado = 0x69;
    }

    if (addr_encontrado == 0) {
        ESP_LOGE(TAG, "MPU-6050 NAO respondeu em 0x68 nem em 0x69!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "MPU-6050 encontrado em 0x%02X! WHO_AM_I = 0x%02X", addr_encontrado, who_am_i);

    esp_err_t err = mpu_configurar_chip();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao configurar o MPU-6050: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "MPU-6050 pronto! Incline o modulo ou de leves batidas na mesa para testar.");

    int contador_log = 0;

    while (1) {
        uint8_t raw[14] = {0};
        if (mpu_read_regs(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw)) == ESP_OK) {
            int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
            int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
            int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);
            int16_t tp_raw = (int16_t)((raw[6] << 8) | raw[7]);
            int16_t gx_raw = (int16_t)((raw[8] << 8) | raw[9]);
            int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
            int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

            float ax = ax_raw / ACCEL_LSB_POR_G;
            float ay = ay_raw / ACCEL_LSB_POR_G;
            float az = az_raw / ACCEL_LSB_POR_G;

            float gx = gx_raw / GYRO_LSB_POR_DPS;
            float gy = gy_raw / GYRO_LSB_POR_DPS;
            float gz = gz_raw / GYRO_LSB_POR_DPS;

            float temp_c = (tp_raw / 340.0f) + 36.53f;

            float accel_total = sqrtf(ax * ax + ay * ay + az * az);
            float delta_g = fabsf(accel_total - 1.0f);

            if (delta_g >= LIMIAR_IMPACTO_G) {
                ESP_LOGW(TAG, ">>> IMPACTO / VIBRACAO DETECTADA! Pico: %.2fg (delta=%.2fg)", accel_total, delta_g);
                vTaskDelay(pdMS_TO_TICKS(250));
                contador_log = 0;
            } else if (++contador_log >= 12) {
                contador_log = 0;
                ESP_LOGI(TAG, "Accel[g]: X=%+.2f Y=%+.2f Z=%+.2f | Gyro[dps]: X=%+6.1f Y=%+6.1f Z=%+6.1f | Temp: %.1f C",
                         ax, ay, az, gx, gy, gz, temp_c);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void sensor_mpu_iniciar(void) {
    xTaskCreatePinnedToCore(sensor_mpu_task, "sensor_mpu_task", 4096, NULL, 5, NULL, tskNO_AFFINITY);
}
