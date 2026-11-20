#include "olho_magico.h"
#include "display_tft.h"

#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_jpeg_dec.h"

static const char *TAG = "OLHO_MAGICO";

#define OLHO_POS_X            0
#define OLHO_POS_Y            50
#define OLHO_LARGURA          320
#define OLHO_ALTURA           240

#define OLHO_JPEG_MAX_BYTES   (64 * 1024)
#define OLHO_RGB_BYTES        (OLHO_LARGURA * OLHO_ALTURA * sizeof(uint16_t))

static uint8_t *s_jpeg_buf = NULL;
static uint8_t *s_rgb_buf = NULL;
static size_t s_jpeg_len = 0;

static SemaphoreHandle_t s_frame_mutex = NULL;
static SemaphoreHandle_t s_sem_novo_frame = NULL;
static jpeg_dec_handle_t s_jpeg_dec = NULL;
static volatile bool s_iniciado = false;

static uint8_t *alocar_alinhado_16(size_t tamanho, const char *nome) {
    // Prioriza PSRAM externa (ESP32-S3 N32R16V); faz fallback para RAM interna se PSRAM estiver desativada
    uint8_t *ptr = heap_caps_aligned_alloc(16, tamanho, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ptr != NULL) {
        ESP_LOGI(TAG, "Buffer '%s' (%u bytes) alocado na PSRAM.", nome, (unsigned)tamanho);
        return ptr;
    }

    ptr = heap_caps_aligned_alloc(16, tamanho, MALLOC_CAP_8BIT);
    if (ptr != NULL) {
        ESP_LOGI(TAG, "Buffer '%s' (%u bytes) alocado na RAM interna.", nome, (unsigned)tamanho);
        return ptr;
    }

    ESP_LOGE(TAG, "Sem memoria para '%s' (%u bytes)! Ative a PSRAM Octal no menuconfig (ESP PSRAM).",
             nome, (unsigned)tamanho);
    return NULL;
}

static bool abrir_decodificador(void) {
    if (s_jpeg_dec != NULL) {
        jpeg_dec_close(s_jpeg_dec);
        s_jpeg_dec = NULL;
    }

    jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();
    cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_BE; // Ja entrega em Big Endian para o ST7796S
    cfg.scale.width = OLHO_LARGURA;                // Redimensiona 480x320 -> 320x240 na decodificacao
    cfg.scale.height = OLHO_ALTURA;

    jpeg_error_t ret = jpeg_dec_open(&cfg, &s_jpeg_dec);
    if (ret != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "Falha ao abrir esp_new_jpeg (erro=%d)", (int)ret);
        s_jpeg_dec = NULL;
        return false;
    }
    return true;
}

void olho_magico_enviar_frame(const uint8_t *jpeg_data, size_t jpeg_len) {
    if (!s_iniciado || !display_tft_pronto()) return;
    if (jpeg_data == NULL || jpeg_len < 4 || jpeg_len > OLHO_JPEG_MAX_BYTES) return;

    // Ignora pacotes USB incompletos que nao comecam com assinatura SOI de JPEG (0xFF 0xD8)
    if (jpeg_data[0] != 0xFF || jpeg_data[1] != 0xD8) return;

    // Timeout 0: se o decoder ainda estiver lendo o frame anterior, descarta este sem travar o USB
    if (xSemaphoreTake(s_frame_mutex, 0) == pdTRUE) {
        memcpy(s_jpeg_buf, jpeg_data, jpeg_len);
        s_jpeg_len = jpeg_len;
        xSemaphoreGive(s_frame_mutex);
        xSemaphoreGive(s_sem_novo_frame);
    }
}

static void olho_magico_task(void *arg) {
    int quadros_periodo = 0;
    int erros_consecutivos = 0;
    int64_t t_inicio_fps = esp_timer_get_time();

    ESP_LOGI(TAG, "Task do Olho Magico ativa no Core %d. Aguardando frames da camera...", xPortGetCoreID());

    while (1) {
        if (xSemaphoreTake(s_sem_novo_frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        int64_t t0 = esp_timer_get_time();

        jpeg_dec_io_t io = {
            .inbuf = s_jpeg_buf,
            .inbuf_len = (int)s_jpeg_len,
            .outbuf = s_rgb_buf,
        };
        jpeg_dec_header_info_t info = {0};

        bool decode_ok = false;
        jpeg_error_t ret = jpeg_dec_parse_header(s_jpeg_dec, &io, &info);
        if (ret == JPEG_ERR_OK) {
            int out_len = 0;
            jpeg_dec_get_outbuf_len(s_jpeg_dec, &out_len);
            if (out_len > 0 && out_len <= (int)OLHO_RGB_BYTES) {
                ret = jpeg_dec_process(s_jpeg_dec, &io);
                decode_ok = (ret == JPEG_ERR_OK);
            }
        }

        // Libera o buffer JPEG antes de enviar ao SPI para o USB ja capturar o proximo quadro em paralelo
        xSemaphoreGive(s_frame_mutex);

        int64_t t1 = esp_timer_get_time();

        if (decode_ok) {
            erros_consecutivos = 0;

            // inverter_bytes = false porque esp_new_jpeg ja gerou RGB565_BE
            display_tft_desenhar_rgb565(OLHO_POS_X, OLHO_POS_Y,
                                        OLHO_LARGURA, OLHO_ALTURA,
                                        (const uint16_t *)s_rgb_buf, false);

            int64_t t2 = esp_timer_get_time();
            quadros_periodo++;

            int64_t delta_fps_us = t2 - t_inicio_fps;
            if (delta_fps_us >= 3000000) { // Log de desempenho a cada 3 segundos
                float fps = (quadros_periodo * 1000000.0f) / (float)delta_fps_us;
                ESP_LOGI(TAG, "Ao vivo: %.1f FPS | Decode JPEG: %d ms | Envio SPI: %d ms | Origem: %ux%u",
                         fps,
                         (int)((t1 - t0) / 1000),
                         (int)((t2 - t1) / 1000),
                         (unsigned)info.width,
                         (unsigned)info.height);
                quadros_periodo = 0;
                t_inicio_fps = t2;
            }
        } else {
            if (++erros_consecutivos >= 5) {
                ESP_LOGW(TAG, "Reiniciando decodificador JPEG apos erros de stream (ultimo=%d)...", (int)ret);
                abrir_decodificador();
                erros_consecutivos = 0;
            }
        }
    }
}

esp_err_t olho_magico_iniciar(void) {
    s_frame_mutex = xSemaphoreCreateMutex();
    s_sem_novo_frame = xSemaphoreCreateBinary();
    if (s_frame_mutex == NULL || s_sem_novo_frame == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_jpeg_buf = alocar_alinhado_16(OLHO_JPEG_MAX_BYTES, "JPEG_IN");
    s_rgb_buf = alocar_alinhado_16(OLHO_RGB_BYTES, "RGB565_OUT");
    if (s_jpeg_buf == NULL || s_rgb_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (!abrir_decodificador()) {
        return ESP_FAIL;
    }

    s_iniciado = true;

    // Fixado no Core 1 para nao disputar ciclos com o USB Host no Core 0
    xTaskCreatePinnedToCore(olho_magico_task, "olho_magico_task", 8192, NULL, 4, NULL, 1);
    return ESP_OK;
}
// FIM DO ARQUIVO
