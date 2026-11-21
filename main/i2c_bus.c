#include "i2c_bus.h"
#include "pinagem.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "I2C_BUS";
static i2c_master_bus_handle_t s_bus_handle = NULL;
static SemaphoreHandle_t s_i2c_mutex = NULL;

esp_err_t i2c_bus_iniciar(void) {
    if (s_bus_handle != NULL) {
        return ESP_OK;
    }

    s_i2c_mutex = xSemaphoreCreateMutex();
    if (s_i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Falha ao criar mutex do barramento I2C.");
        return ESP_ERR_NO_MEM;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar I2C: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Barramento I2C iniciado (SDA=GPIO%d, SCL=GPIO%d)", PIN_I2C_SDA, PIN_I2C_SCL);

    // Tempo de estabilizacao eletrica dos sensores antes do scanner unico de boot
    vTaskDelay(pdMS_TO_TICKS(150));

    ESP_LOGI(TAG, "Scanner I2C de boot (antes de iniciar as tarefas):");
    int encontrados = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(s_bus_handle, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  -> Dispositivo ativo em 0x%02X", addr);
            encontrados++;
        }
    }
    if (encontrados == 0) {
        ESP_LOGE(TAG, "  -> Nenhum dispositivo encontrado no I2C!");
    }

    return ESP_OK;
}

i2c_master_bus_handle_t i2c_bus_obter_handle(void) {
    return s_bus_handle;
}

bool i2c_bus_lock(uint32_t timeout_ms) {
    if (s_i2c_mutex == NULL) return false;
    return (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
}

void i2c_bus_unlock(void) {
    if (s_i2c_mutex != NULL) {
        xSemaphoreGive(s_i2c_mutex);
    }
}

esp_err_t i2c_bus_recuperar(void) {
    static int64_t ultimo_reset_us = 0;

    i2c_master_bus_handle_t bus = i2c_bus_obter_handle();
    if (bus == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!i2c_bus_lock(300)) {
        return ESP_ERR_TIMEOUT;
    }

    int64_t agora_us = esp_timer_get_time();
    if ((agora_us - ultimo_reset_us) < 500000LL) {
        // Outro sensor acabou de resetar o barramento ha menos de 500 ms
        i2c_bus_unlock();
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Timeout I2C detectado -> Executando i2c_master_bus_reset()...");
    esp_err_t err = i2c_master_bus_reset(bus);
    ultimo_reset_us = esp_timer_get_time();
    i2c_bus_unlock();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao resetar barramento I2C: %s", esp_err_to_name(err));
    }
    return err;
}
