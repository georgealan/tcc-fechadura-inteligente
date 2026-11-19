#include "i2c_bus.h"
#include "pinagem.h"
#include "esp_log.h"

static const char *TAG = "I2C_BUS";
static i2c_master_bus_handle_t s_bus_handle = NULL;

esp_err_t i2c_bus_iniciar(void) {
    if (s_bus_handle != NULL) {
        return ESP_OK; // Já inicializado
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
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Barramento I2C iniciado (SDA=GPIO%d, SCL=GPIO%d)", PIN_I2C_SDA, PIN_I2C_SCL);
    } else {
        ESP_LOGE(TAG, "Falha ao iniciar I2C: %s", esp_err_to_name(err));
    }
    return err;
}

i2c_master_bus_handle_t i2c_bus_obter_handle(void) {
    return s_bus_handle;
}
