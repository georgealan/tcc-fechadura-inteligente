#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "porta.h"
#include "teclado.h"
#include "i2c_bus.h"
#include "sensor_tof.h"
#include "sensor_mpu.h"
#include "display_tft.h"
#include "olho_magico.h"
#include "camera_uvc.h"

static const char *TAG = "MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Fechadura Inteligente (TCC)...");

    // 1. Entrada e sensores no barramento I2C0
    teclado_iniciar();
    ESP_ERROR_CHECK(porta_iniciar());
    ESP_ERROR_CHECK(i2c_bus_iniciar());
    sensor_tof_iniciar();
    sensor_mpu_iniciar();

    // 2. Video (display + decodificador + camera USB UVC)
    ESP_ERROR_CHECK(display_tft_iniciar());
    ESP_ERROR_CHECK(olho_magico_iniciar());
    ESP_ERROR_CHECK(camera_uvc_iniciar());

    ESP_LOGI(TAG, "Todos os modulos iniciados.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
