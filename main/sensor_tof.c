#include "sensor_tof.h"
#include "i2c_bus.h"
#include "fechadura_fsm.h"

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "VL53L1X";

#define VL53L1X_I2C_ADDR                  0x29
#define VL53L1X_REG_VHV_LOOP_BOUND        0x0008
#define VL53L1X_REG_VHV_INIT              0x000B
#define VL53L1X_REG_GPIO_HV_MUX_CTRL      0x0030
#define VL53L1X_REG_GPIO_TIO_HV_STATUS    0x0031
#define VL53L1X_REG_SYSTEM_INT_CLEAR      0x0086
#define VL53L1X_REG_SYSTEM_MODE_START     0x0087
#define VL53L1X_REG_RESULT_RANGE_STATUS   0x0089
#define VL53L1X_REG_RESULT_DISTANCE_MM    0x0096
#define VL53L1X_REG_FIRMWARE_STATUS       0x00E5
#define VL53L1X_REG_MODEL_ID              0x010F
#define VL53L1X_REG_MODULE_TYPE           0x0110

#define DISTANCIA_TRIGGER_MM              600   // Ativa visitante em <= 60 cm
#define DISTANCIA_LIBERACAO_MM            620   // Libera logo acima de 62 cm
#define DISTANCIA_MIN_VALIDA_MM           20    // Descarta leituras espurias de 0 mm
#define TIMEOUT_SEM_DADO_MS               1500  // Protecao contra travamento por pico do rele
#define COOLDOWN_EVENTO_PRESENCA_MS       2500
#define I2C_TIMEOUT_MS                    100

static const uint8_t VL53L1X_DEFAULT_CONFIG[91] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08,
    0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02,
    0x0A, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01,
    0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08,
    0xB8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00
};

static i2c_master_dev_handle_t s_tof_dev = NULL;

static portMUX_TYPE s_tof_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_ultima_dist_mm = 0;
static bool s_leitura_valida = false;
static bool s_presenca_ativa = false;

bool sensor_tof_obter_leitura(uint16_t *dist_mm, bool *presenca_ativa) {
    portENTER_CRITICAL(&s_tof_mux);
    bool valida = s_leitura_valida;
    uint16_t dist = s_ultima_dist_mm;
    bool presenca = s_presenca_ativa;
    portEXIT_CRITICAL(&s_tof_mux);

    if (dist_mm != NULL) *dist_mm = dist;
    if (presenca_ativa != NULL) *presenca_ativa = presenca;
    return valida;
}

static esp_err_t tof_write_byte(uint16_t reg, uint8_t val) {
    if (!i2c_bus_lock(200)) return ESP_ERR_TIMEOUT;
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val };
    esp_err_t err = i2c_master_transmit(s_tof_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
    i2c_bus_unlock();

    if (err == ESP_ERR_TIMEOUT) {
        i2c_bus_recuperar();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return err;
}

static esp_err_t tof_read_byte(uint16_t reg, uint8_t *val) {
    if (!i2c_bus_lock(200)) return ESP_ERR_TIMEOUT;
    uint8_t reg_buf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    esp_err_t err = i2c_master_transmit(s_tof_dev, reg_buf, sizeof(reg_buf), I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        err = i2c_master_receive(s_tof_dev, val, 1, I2C_TIMEOUT_MS);
    }
    i2c_bus_unlock();

    if (err == ESP_ERR_TIMEOUT) {
        i2c_bus_recuperar();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return err;
}

static esp_err_t tof_read_word(uint16_t reg, uint16_t *val) {
    uint8_t hi = 0, lo = 0;
    esp_err_t err = tof_read_byte(reg, &hi);
    if (err != ESP_OK) return err;
    err = tof_read_byte(reg + 1, &lo);
    if (err != ESP_OK) return err;
    *val = ((uint16_t)hi << 8) | lo;
    return ESP_OK;
}

static bool tof_dado_pronto(void) {
    uint8_t mux = 0, status = 0;
    if (tof_read_byte(VL53L1X_REG_GPIO_HV_MUX_CTRL, &mux) != ESP_OK) return false;
    if (tof_read_byte(VL53L1X_REG_GPIO_TIO_HV_STATUS, &status) != ESP_OK) return false;

    uint8_t int_pol = !((mux & 0x10) >> 4);
    return ((status & 0x01) == int_pol);
}

static esp_err_t tof_inicializar_chip(void) {
    uint8_t boot_status = 0;
    for (int i = 0; i < 50; i++) {
        if (tof_read_byte(VL53L1X_REG_FIRMWARE_STATUS, &boot_status) == ESP_OK && (boot_status & 0x01)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!(boot_status & 0x01)) {
        ESP_LOGE(TAG, "Timeout aguardando boot do VL53L1X.");
        return ESP_ERR_TIMEOUT;
    }

    uint8_t model_id = 0, mod_type = 0;
    esp_err_t err = tof_read_byte(VL53L1X_REG_MODEL_ID, &model_id);
    if (err != ESP_OK) return err;
    err = tof_read_byte(VL53L1X_REG_MODULE_TYPE, &mod_type);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "VL53L1X identificado! ID = 0x%02X%02X", model_id, mod_type);

    for (uint16_t i = 0; i < sizeof(VL53L1X_DEFAULT_CONFIG); i++) {
        err = tof_write_byte(0x002D + i, VL53L1X_DEFAULT_CONFIG[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Erro ao gravar reg 0x%04X: %s", 0x002D + i, esp_err_to_name(err));
            return err;
        }
    }

    err = tof_write_byte(VL53L1X_REG_SYSTEM_MODE_START, 0x40);
    if (err != ESP_OK) return err;

    for (int i = 0; i < 100 && !tof_dado_pronto(); i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    tof_write_byte(VL53L1X_REG_SYSTEM_INT_CLEAR, 0x01);
    tof_write_byte(VL53L1X_REG_SYSTEM_MODE_START, 0x00);
    tof_write_byte(VL53L1X_REG_VHV_LOOP_BOUND, 0x09);
    tof_write_byte(VL53L1X_REG_VHV_INIT, 0x00);

    return tof_write_byte(VL53L1X_REG_SYSTEM_MODE_START, 0x40);
}

static uint16_t mediana_3(uint16_t a, uint16_t b, uint16_t c) {
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

static void sensor_tof_task(void *arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(50));

    i2c_master_bus_handle_t bus = i2c_bus_obter_handle();
    if (bus == NULL) {
        ESP_LOGE(TAG, "Barramento I2C nao inicializado!");
        vTaskDelete(NULL);
        return;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L1X_I2C_ADDR,
        .scl_speed_hz = 50000,
    };

    esp_err_t err = ESP_ERR_TIMEOUT;
    if (i2c_bus_lock(300)) {
        if (i2c_master_probe(bus, VL53L1X_I2C_ADDR, I2C_TIMEOUT_MS) == ESP_OK) {
            err = i2c_master_bus_add_device(bus, &dev_cfg, &s_tof_dev);
        } else {
            err = ESP_ERR_NOT_FOUND;
        }
        i2c_bus_unlock();
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VL53L1X nao respondeu em 0x29 (%s).", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    err = tof_inicializar_chip();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha na inicializacao do VL53L1X: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "VL53L1X medindo! (trigger <= %d mm, libera >= %d mm)",
             DISTANCIA_TRIGGER_MM, DISTANCIA_LIBERACAO_MM);

    int contador_log = 0;
    bool presenca_local = false;
    uint16_t janela[3] = { 0xFFFF, 0xFFFF, 0xFFFF };
    int idx_janela = 0;
    int64_t ultimo_evento_us = 0;
    int64_t ultimo_dado_ok_us = esp_timer_get_time();

    while (1) {
        int64_t agora_us = esp_timer_get_time();

        if (tof_dado_pronto()) {
            uint8_t status_raw = 0;
            uint16_t dist_mm = 0;

            esp_err_t e1 = tof_read_byte(VL53L1X_REG_RESULT_RANGE_STATUS, &status_raw);
            esp_err_t e2 = tof_read_word(VL53L1X_REG_RESULT_DISTANCE_MM, &dist_mm);
            tof_write_byte(VL53L1X_REG_SYSTEM_INT_CLEAR, 0x01);

            if (e1 == ESP_OK && e2 == ESP_OK) {
                ultimo_dado_ok_us = agora_us;
                uint8_t range_status = status_raw & 0x1F;

                // Se fora de alcance ou ruido perto de 0 mm, trata como infinito (0xFFFF)
                bool amostra_valida = (range_status == 9 && dist_mm >= DISTANCIA_MIN_VALIDA_MM);
                janela[idx_janela] = amostra_valida ? dist_mm : 0xFFFF;
                idx_janela = (idx_janela + 1) % 3;

                uint16_t dist_filtrada = mediana_3(janela[0], janela[1], janela[2]);
                bool filtrada_valida = (dist_filtrada != 0xFFFF);

                if (filtrada_valida && dist_filtrada <= DISTANCIA_TRIGGER_MM) {
                    if (!presenca_local || ((agora_us - ultimo_evento_us) / 1000) >= COOLDOWN_EVENTO_PRESENCA_MS) {
                        ultimo_evento_us = agora_us;
                        fechadura_fsm_enviar_evento(EVT_PRESENCA_DETECTADA, 0);
                    }
                    presenca_local = true;
                } else if (!filtrada_valida || dist_filtrada >= DISTANCIA_LIBERACAO_MM) {
                    presenca_local = false;
                }

                portENTER_CRITICAL(&s_tof_mux);
                s_leitura_valida = filtrada_valida;
                s_ultima_dist_mm = filtrada_valida ? dist_filtrada : 0;
                s_presenca_ativa = presenca_local;
                portEXIT_CRITICAL(&s_tof_mux);

                if (++contador_log >= 10) {
                    contador_log = 0;
                    if (filtrada_valida) {
                        if (presenca_local) {
                            ESP_LOGW(TAG, ">>> PRESENCA NA PORTA! Distancia: %u mm (%.1f cm)",
                                     dist_filtrada, dist_filtrada / 10.0f);
                        } else {
                            ESP_LOGI(TAG, "Distancia: %u mm (%.1f cm)",
                                     dist_filtrada, dist_filtrada / 10.0f);
                        }
                    } else {
                        ESP_LOGI(TAG, "Area livre / fora de alcance (status=%u)", range_status);
                    }
                }
            }
        } else if (((agora_us - ultimo_dado_ok_us) / 1000) >= TIMEOUT_SEM_DADO_MS) {
            ESP_LOGW(TAG, "VL53L1X sem novos dados ha >%d ms. Reinicializando...", TIMEOUT_SEM_DADO_MS);
            janela[0] = janela[1] = janela[2] = 0xFFFF;
            presenca_local = false;

            portENTER_CRITICAL(&s_tof_mux);
            s_leitura_valida = false;
            s_ultima_dist_mm = 0;
            s_presenca_ativa = false;
            portEXIT_CRITICAL(&s_tof_mux);

            tof_write_byte(VL53L1X_REG_SYSTEM_INT_CLEAR, 0x01);
            tof_inicializar_chip();
            ultimo_dado_ok_us = esp_timer_get_time();
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void sensor_tof_iniciar(void) {
    xTaskCreatePinnedToCore(sensor_tof_task, "sensor_tof_task", 4096, NULL, 5, NULL, tskNO_AFFINITY);
}
// FIM DO ARQUIVO
