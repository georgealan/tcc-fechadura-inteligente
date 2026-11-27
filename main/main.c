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
#include "audio_i2s.h"
#include "fechadura_fsm.h"
#include "hud_tft.h"
#include "wifi_manager.h"
#include "facial_client.h"
#include "cloud_sync.h"

static const char *TAG = "MAIN";

extern const uint8_t teste_wav_start[] asm("_binary_teste_wav_start");
extern const uint8_t teste_wav_end[]   asm("_binary_teste_wav_end");

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Fechadura Inteligente (TCC)...");

    // 1. Audio e maquina de estados (base para todos os outros modulos)
    ESP_ERROR_CHECK(audio_i2s_iniciar());
    ESP_ERROR_CHECK(fechadura_fsm_iniciar());

    // 2. Entrada, atuador e sensores no barramento I2C0
    teclado_iniciar();
    ESP_ERROR_CHECK(porta_iniciar());
    ESP_ERROR_CHECK(i2c_bus_iniciar());
    sensor_tof_iniciar();
    sensor_mpu_iniciar();

    // 3. Video (display + buffer/decodificador + camera USB UVC)
    ESP_ERROR_CHECK(display_tft_iniciar());
    ESP_ERROR_CHECK(hud_tft_iniciar());
    ESP_ERROR_CHECK(olho_magico_iniciar());
    ESP_ERROR_CHECK(camera_uvc_iniciar());

    // 4. Rede e integracao com a API (Docker / VPS)
    ESP_ERROR_CHECK(wifi_manager_iniciar());
    ESP_ERROR_CHECK(facial_client_iniciar());
    ESP_ERROR_CHECK(cloud_sync_iniciar());

    ESP_LOGI(TAG, "Todos os modulos iniciados.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
