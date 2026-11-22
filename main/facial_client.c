#include "facial_client.h"
#include "wifi_credenciais.h"
#include "wifi_manager.h"
#include "olho_magico.h"
#include "fechadura_fsm.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "FACIAL_IA";

#define FACIAL_JPEG_MAX_BYTES       (64 * 1024)
#define FACIAL_WAV_MAX_BYTES        (1024 * 1024) // 1 MB na PSRAM (~21s de audio 24kHz 16-bit mono)
#define FACIAL_ATRASO_POSICAO_MS    1500          // Tempo para visitante olhar para a camera
#define FACIAL_COOLDOWN_MS          10000         // Intervalo minimo entre ciclos automaticos
#define FACIAL_HTTP_TIMEOUT_MS      25000         // Folga para InsightFace + Kokoro TTS em CPU
#define FACIAL_BOUNDARY             "----ESP32TCCFacialBoundary"

static uint8_t *s_jpeg_envio = NULL;
static uint8_t *s_wav_resp = NULL;

static SemaphoreHandle_t s_sem_disparo = NULL;
static portMUX_TYPE s_facial_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool s_em_andamento = false;
static facial_fase_t s_fase = FACIAL_FASE_OCIOSO;
static int64_t s_ultimo_fim_us = 0;
static char s_ultimo_nome[32] = "";

typedef struct {
    char status[24];
    char nome_raw[96];
} http_resp_headers_t;

static void definir_fase(facial_fase_t nova) {
    portENTER_CRITICAL(&s_facial_mux);
    s_fase = nova;
    portEXIT_CRITICAL(&s_facial_mux);
}

static int hex_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Decodifica %XX e converte UTF-8 acentuado comum para ASCII maiusculo (compativel com fonte 5x7 do TFT)
static void normalizar_nome_para_tft(const char *encoded, char *out, size_t out_len) {
    if (out_len == 0) return;
    uint8_t utf8[96];
    size_t ulen = 0;

    for (size_t i = 0; encoded[i] != '\0' && ulen < sizeof(utf8) - 1; i++) {
        if (encoded[i] == '%' && encoded[i + 1] && encoded[i + 2]) {
            int hi = hex_val(encoded[i + 1]);
            int lo = hex_val(encoded[i + 2]);
            if (hi >= 0 && lo >= 0) {
                utf8[ulen++] = (uint8_t)((hi << 4) | lo);
                i += 2;
                continue;
            }
        }
        utf8[ulen++] = (uint8_t)((encoded[i] == '+') ? ' ' : encoded[i]);
    }
    utf8[ulen] = '\0';

    size_t j = 0;
    for (size_t i = 0; i < ulen && j < out_len - 1; i++) {
        uint8_t b = utf8[i];
        if (b < 0x80) {
            out[j++] = (char)toupper((unsigned char)b);
        } else if (b == 0xC3 && (i + 1) < ulen) {
            uint8_t b2 = utf8[++i];
            char sub = ' ';
            if ((b2 >= 0x80 && b2 <= 0x85) || (b2 >= 0xA0 && b2 <= 0xA5)) sub = 'A';
            else if (b2 == 0x87 || b2 == 0xA7) sub = 'C';
            else if ((b2 >= 0x88 && b2 <= 0x8B) || (b2 >= 0xA8 && b2 <= 0xAB)) sub = 'E';
            else if ((b2 >= 0x8C && b2 <= 0x8F) || (b2 >= 0xAC && b2 <= 0xAF)) sub = 'I';
            else if (b2 == 0x91 || b2 == 0xB1) sub = 'N';
            else if ((b2 >= 0x92 && b2 <= 0x96) || (b2 >= 0xB2 && b2 <= 0xB6)) sub = 'O';
            else if ((b2 >= 0x99 && b2 <= 0x9C) || (b2 >= 0xB9 && b2 <= 0xBC)) sub = 'U';
            out[j++] = sub;
        }
    }
    out[j] = '\0';
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    http_resp_headers_t *hdrs = (http_resp_headers_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_HEADER && hdrs != NULL &&
        evt->header_key != NULL && evt->header_value != NULL) {
        if (strcasecmp(evt->header_key, "X-Fechadura-Status") == 0) {
            strncpy(hdrs->status, evt->header_value, sizeof(hdrs->status) - 1);
            hdrs->status[sizeof(hdrs->status) - 1] = '\0';
        } else if (strcasecmp(evt->header_key, "X-Fechadura-Nome") == 0) {
            strncpy(hdrs->nome_raw, evt->header_value, sizeof(hdrs->nome_raw) - 1);
            hdrs->nome_raw[sizeof(hdrs->nome_raw) - 1] = '\0';
        }
    }
    return ESP_OK;
}

static bool enviar_tudo_http(esp_http_client_handle_t client, const uint8_t *dados, size_t len) {
    size_t enviado = 0;
    while (enviado < len) {
        int w = esp_http_client_write(client, (const char *)(dados + enviado), (int)(len - enviado));
        if (w <= 0) return false;
        enviado += (size_t)w;
    }
    return true;
}

static void inspecionar_wav_psram(const uint8_t *wav, size_t len) {
    if (len >= 44 && memcmp(wav, "RIFF", 4) == 0 && memcmp(wav + 8, "WAVE", 4) == 0) {
        uint16_t canais = (uint16_t)(wav[22] | (wav[23] << 8));
        uint32_t sample_rate = (uint32_t)(wav[24] | (wav[25] << 8) | (wav[26] << 16) | (wav[27] << 24));
        uint16_t bits = (uint16_t)(wav[34] | (wav[35] << 8));
        uint32_t bytes_por_seg = sample_rate * canais * (bits / 8);
        float duracao_s = (bytes_por_seg > 0) ? ((float)(len - 44) / (float)bytes_por_seg) : 0.0f;

        ESP_LOGI(TAG, "WAV Kokoro recebido na PSRAM: %u bytes | %lu Hz | %u-bit | %u ch | ~%.1f s",
                 (unsigned)len, (unsigned long)sample_rate, (unsigned)bits, (unsigned)canais, duracao_s);
        // Gancho futuro para o novo MAX98357A:
        // audio_i2s_tocar_wav_psram(wav, len);
    } else {
        ESP_LOGW(TAG, "Resposta recebida (%u bytes), mas nao possui cabecalho RIFF/WAVE.", (unsigned)len);
    }
}

static void executar_reconhecimento(void) {
    if (!wifi_manager_esta_conectado()) {
        ESP_LOGW(TAG, "Wi-Fi offline. Reconhecimento facial ignorado.");
        definir_fase(FACIAL_FASE_ERRO_REDE);
        fechadura_fsm_enviar_evento(EVT_FACIAL_ERRO, 0);
        return;
    }

    // 1. Aviso de inicio + tempo para visitante olhar para a camera
    definir_fase(FACIAL_FASE_POSICIONANDO);
    ESP_LOGI(TAG, "[AUDIO LOCAL] Iniciando reconhecimento facial, olhe para a camera...");
    vTaskDelay(pdMS_TO_TICKS(FACIAL_ATRASO_POSICAO_MS));

    // Se o usuario comecou a digitar senha ou abriu a porta nesse meio tempo, aborta
    if (fechadura_fsm_obter_estado() != ESTADO_RECONHECIMENTO_FACIAL) {
        ESP_LOGI(TAG, "Ciclo facial cancelado por outra acao na fechadura.");
        return;
    }

    // 2. Captura snapshot JPEG direto da PSRAM do Olho Magico
    size_t jpeg_len = 0;
    if (!olho_magico_capturar_jpeg(s_jpeg_envio, FACIAL_JPEG_MAX_BYTES, &jpeg_len)) {
        ESP_LOGW(TAG, "Sem frame JPEG recente da camera UVC.");
        definir_fase(FACIAL_FASE_ERRO_REDE);
        fechadura_fsm_enviar_evento(EVT_FACIAL_ERRO, 0);
        return;
    }

    definir_fase(FACIAL_FASE_ENVIANDO);
    ESP_LOGI(TAG, "Frame capturado (%u bytes). Enviando para %s ...", (unsigned)jpeg_len, API_FACIAL_URL);
    ESP_LOGI(TAG, "[AUDIO LOCAL] Imagem enviada com sucesso, aguarde...");

    // 3. Monta envelope multipart/form-data com campo "imagem" (igual ao FastAPI)
    const char *part_head =
        "--" FACIAL_BOUNDARY "\r\n"
        "Content-Disposition: form-data; name=\"imagem\"; filename=\"rosto.jpg\"\r\n"
        "Content-Type: image/jpeg\r\n\r\n";
    const char *part_tail = "\r\n--" FACIAL_BOUNDARY "--\r\n";

    size_t head_len = strlen(part_head);
    size_t tail_len = strlen(part_tail);
    int total_post_len = (int)(head_len + jpeg_len + tail_len);

    http_resp_headers_t resp_hdrs = {0};

    esp_http_client_config_t cfg = {
        .url = API_FACIAL_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = FACIAL_HTTP_TIMEOUT_MS,
        .event_handler = http_event_handler,
        .user_data = &resp_hdrs,
        .buffer_size = 4096,
        .buffer_size_tx = 4096,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Falha ao inicializar cliente HTTP.");
        definir_fase(FACIAL_FASE_ERRO_REDE);
        fechadura_fsm_enviar_evento(EVT_FACIAL_ERRO, 0);
        return;
    }

    esp_http_client_set_header(client, "Content-Type", "multipart/form-data; boundary=" FACIAL_BOUNDARY);

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = esp_http_client_open(client, total_post_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao conectar no servidor (%s): %s", API_FACIAL_URL, esp_err_to_name(err));
        esp_http_client_cleanup(client);
        definir_fase(FACIAL_FASE_ERRO_REDE);
        fechadura_fsm_enviar_evento(EVT_FACIAL_ERRO, 0);
        return;
    }

    bool write_ok = enviar_tudo_http(client, (const uint8_t *)part_head, head_len) &&
                    enviar_tudo_http(client, s_jpeg_envio, jpeg_len) &&
                    enviar_tudo_http(client, (const uint8_t *)part_tail, tail_len);

    if (!write_ok) {
        ESP_LOGE(TAG, "Falha durante upload do frame JPEG.");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        definir_fase(FACIAL_FASE_ERRO_REDE);
        fechadura_fsm_enviar_evento(EVT_FACIAL_ERRO, 0);
        return;
    }

    int content_len = esp_http_client_fetch_headers(client);
    int http_status = esp_http_client_get_status_code(client);

    // 4. Le o corpo da resposta (WAV do Kokoro ou JSON de erro) direto para a PSRAM
    size_t total_lido = 0;
    while (1) {
        if (total_lido < FACIAL_WAV_MAX_BYTES) {
            int espaco = (int)(FACIAL_WAV_MAX_BYTES - total_lido);
            int bloco = (espaco > 4096) ? 4096 : espaco;
            int r = esp_http_client_read(client, (char *)(s_wav_resp + total_lido), bloco);
            if (r <= 0) break;
            total_lido += (size_t)r;
        } else {
            char descarte[256];
            int r = esp_http_client_read(client, descarte, sizeof(descarte));
            if (r <= 0) break;
        }
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    int tempo_ms = (int)((esp_timer_get_time() - t0) / 1000);
    ESP_LOGI(TAG, "Resposta HTTP %d em %d ms | Content-Length=%d | Lido=%u bytes | Status='%s'",
             http_status, tempo_ms, content_len, (unsigned)total_lido, resp_hdrs.status);

    // 5. Interpreta resultado da IA
    if (http_status == 200 && strcasecmp(resp_hdrs.status, "autorizado") == 0) {
        char nome_limpo[32] = "MORADOR";
        if (resp_hdrs.nome_raw[0] != '\0') {
            normalizar_nome_para_tft(resp_hdrs.nome_raw, nome_limpo, sizeof(nome_limpo));
        }

        portENTER_CRITICAL(&s_facial_mux);
        strncpy(s_ultimo_nome, nome_limpo, sizeof(s_ultimo_nome) - 1);
        s_ultimo_nome[sizeof(s_ultimo_nome) - 1] = '\0';
        portEXIT_CRITICAL(&s_facial_mux);

        ESP_LOGI(TAG, ">>> FACIAL AUTORIZADO! Bem-vindo(a), %s!", nome_limpo);
        inspecionar_wav_psram(s_wav_resp, total_lido);

        definir_fase(FACIAL_FASE_OCIOSO);
        fechadura_fsm_enviar_evento(EVT_ACESSO_REMOTO_OK, 0);

    } else if (http_status == 401 || strcasecmp(resp_hdrs.status, "negado") == 0) {
        ESP_LOGW(TAG, ">>> FACIAL NEGADO! Rosto nao reconhecido.");
        inspecionar_wav_psram(s_wav_resp, total_lido);

        definir_fase(FACIAL_FASE_NEGADO);
        fechadura_fsm_enviar_evento(EVT_FACIAL_NEGADO, 0);

    } else if (http_status == 400) {
        size_t lim = (total_lido < 200) ? total_lido : 200;
        s_wav_resp[lim] = '\0';
        ESP_LOGW(TAG, "Servidor retornou 400: %s", (const char *)s_wav_resp);

        definir_fase(FACIAL_FASE_SEM_ROSTO);
        fechadura_fsm_enviar_evento(EVT_FACIAL_ERRO, 0);

    } else {
        ESP_LOGE(TAG, "Erro inesperado do servidor (HTTP %d).", http_status);
        definir_fase(FACIAL_FASE_ERRO_REDE);
        fechadura_fsm_enviar_evento(EVT_FACIAL_ERRO, 0);
    }
}

static void facial_task(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Task de Reconhecimento Facial pronta (API: %s).", API_FACIAL_URL);

    while (1) {
        if (xSemaphoreTake(s_sem_disparo, portMAX_DELAY) == pdTRUE) {
            executar_reconhecimento();

            // Mantem o feedback visual de erro/negado visivel no HUD por 2,5s antes de limpar
            if (facial_client_obter_fase() != FACIAL_FASE_OCIOSO) {
                vTaskDelay(pdMS_TO_TICKS(2500));
                definir_fase(FACIAL_FASE_OCIOSO);
            }

            portENTER_CRITICAL(&s_facial_mux);
            s_ultimo_fim_us = esp_timer_get_time();
            s_em_andamento = false;
            portEXIT_CRITICAL(&s_facial_mux);
        }
    }
}

bool facial_client_solicitar_ciclo(void) {
    if (s_sem_disparo == NULL || !wifi_manager_esta_conectado()) {
        return false;
    }

    int64_t agora_us = esp_timer_get_time();
    bool pode_disparar = false;

    portENTER_CRITICAL(&s_facial_mux);
    int64_t desde_ultimo_ms = (s_ultimo_fim_us > 0) ? ((agora_us - s_ultimo_fim_us) / 1000) : FACIAL_COOLDOWN_MS;
    if (!s_em_andamento && desde_ultimo_ms >= FACIAL_COOLDOWN_MS) {
        s_em_andamento = true;
        s_fase = FACIAL_FASE_POSICIONANDO;
        pode_disparar = true;
    }
    portEXIT_CRITICAL(&s_facial_mux);

    if (pode_disparar) {
        xSemaphoreGive(s_sem_disparo);
    }
    return pode_disparar;
}

facial_fase_t facial_client_obter_fase(void) {
    portENTER_CRITICAL(&s_facial_mux);
    facial_fase_t f = s_fase;
    portEXIT_CRITICAL(&s_facial_mux);
    return f;
}

void facial_client_obter_ultimo_nome(char *buf, size_t len) {
    if (buf == NULL || len == 0) return;
    portENTER_CRITICAL(&s_facial_mux);
    strncpy(buf, s_ultimo_nome, len - 1);
    buf[len - 1] = '\0';
    portEXIT_CRITICAL(&s_facial_mux);
}

esp_err_t facial_client_iniciar(void) {
    s_jpeg_envio = heap_caps_malloc(FACIAL_JPEG_MAX_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_wav_resp = heap_caps_malloc(FACIAL_WAV_MAX_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_jpeg_envio == NULL || s_wav_resp == NULL) {
        ESP_LOGE(TAG, "Falha ao alocar buffers faciais na PSRAM!");
        return ESP_ERR_NO_MEM;
    }

    s_sem_disparo = xSemaphoreCreateBinary();
    if (s_sem_disparo == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Stack de 8 KB no Core 0 (mesmo core da pilha Wi-Fi/TCP-IP)
    BaseType_t ok = xTaskCreatePinnedToCore(facial_task, "facial_task", 8192, NULL, 4, NULL, 0);
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}
// FIM DO ARQUIVO
