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

static const char *TAG = "MAIN";

extern const uint8_t teste_wav_start[] asm("_binary_teste_wav_start");
extern const uint8_t teste_wav_end[]   asm("_binary_teste_wav_end");

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Fechadura Inteligente (TCC)...");

    // Rotina de inicialização do módulo de áudio
    //ESP_ERROR_CHECK(audio_i2s_iniciar());
    //audio_i2s_tocar_wav(teste_wav_start, (size_t)(teste_wav_end - teste_wav_start));

    ESP_ERROR_CHECK(fechadura_fsm_iniciar());

    // 1. Entrada e sensores no barramento I2C0
    teclado_iniciar();
    ESP_ERROR_CHECK(porta_iniciar());
    ESP_ERROR_CHECK(i2c_bus_iniciar());
    sensor_tof_iniciar();
    sensor_mpu_iniciar();

    // 2. Video (display + decodificador + camera USB UVC)
    ESP_ERROR_CHECK(display_tft_iniciar());
    ESP_ERROR_CHECK(hud_tft_iniciar());
    ESP_ERROR_CHECK(olho_magico_iniciar());
    ESP_ERROR_CHECK(camera_uvc_iniciar());

    ESP_LOGI(TAG, "Todos os modulos iniciados.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
