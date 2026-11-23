#include "cloud_sync.h"
#include "wifi_credenciais.h"
#include "wifi_manager.h"
#include "olho_magico.h"
#include "fechadura_fsm.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"

static const char *TAG = "CLOUD_SYNC";

#define CLOUD_POLL_INTERVALO_MS   1500
#define CLOUD_HTTP_TIMEOUT_MS     3000
#define CLOUD_EVENTO_TIMEOUT_MS   8000
#define CLOUD_JPEG_MAX_BYTES      (64 * 1024)
#define CLOUD_BOUNDARY            "----ESP32TCCAlertaBoundary"

static uint8_t *s_jpeg_alerta = NULL;

static SemaphoreHandle_t s_sem_acordar = NULL;
static SemaphoreHandle_t s_sem_pin_pronto = NULL;
static SemaphoreHandle_t s_mutex_pin_api = NULL;
static portMUX_TYPE s_sync_mux = portMUX_INITIALIZER_UNLOCKED;

// Estado da fila de alerta de arrombamento
static volatile bool s_alarme_pendente = false;
static char s_alarme_detalhes[160] = "";

// Estado da requisicao de validacao de PIN
static volatile bool s_pin_pendente = false;
static char s_pin_req[16] = "";
static bool s_pin_req_desarmar = false;
static pin_online_resultado_t s_pin_resp_status = PIN_ONLINE_OFFLINE;
static char s_pin_resp_nome[32] = "";

// ------------------------------------------
// HELPERS LEVES DE JSON E UTF-8 (SEM cJSON)
// ------------------------------------------
static const char *json_encontrar_valor(const char *json, const char *chave) {
    if (!json || !chave) return NULL;
    char padrao[48];
    snprintf(padrao, sizeof(padrao), "\"%s\"", chave);
    size_t padrao_len = strlen(padrao);

    const char *p = json;
    while ((p = strstr(p, padrao)) != NULL) {
        p += padrao_len;
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        if (*p == ':') {
            p++;
            while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
            return p;
        }
    }
    return NULL;
}

static bool json_extrair_string(const char *json, const char *chave, char *out, size_t out_len) {
    if (!out || out_len == 0) return false;
    out[0] = '\0';
    const char *v = json_encontrar_valor(json, chave);
    if (!v || *v != '"') return false;
    v++; // pula aspa inicial

    size_t i = 0;
    while (*v && *v != '"' && i < out_len - 1) {
        if (*v == '\\' && *(v + 1)) {
            v++;
        }
        out[i++] = *v++;
    }
    out[i] = '\0';
    return true;
}

static bool json_extrair_bool(const char *json, const char *chave) {
    const char *v = json_encontrar_valor(json, chave);
    return (v != NULL && strncmp(v, "true", 4) == 0);
}

// Converte UTF-8 acentuado para ASCII maiusculo (compativel com fonte 5x7 do TFT)
static void normalizar_utf8_para_tft(const char *utf8, char *out, size_t out_len) {
    if (!utf8 || !out || out_len == 0) return;
    size_t ulen = strlen(utf8);
    size_t j = 0;

    for (size_t i = 0; i < ulen && j < out_len - 1; i++) {
        uint8_t b = (uint8_t)utf8[i];
        if (b < 0x80) {
            out[j++] = (char)toupper((unsigned char)b);
        } else if (b == 0xC3 && (i + 1) < ulen) {
            uint8_t b2 = (uint8_t)utf8[++i];
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

static bool enviar_tudo_http(esp_http_client_handle_t client, const uint8_t *dados, size_t len) {
    size_t enviado = 0;
    while (enviado < len) {
        int w = esp_http_client_write(client, (const char *)(dados + enviado), (int)(len - enviado));
        if (w <= 0) return false;
        enviado += (size_t)w;
    }
    return true;
}

static size_t ler_corpo_http(esp_http_client_handle_t client, char *buf, size_t max_len) {
    size_t total = 0;
    while (total < max_len - 1) {
        int r = esp_http_client_read(client, buf + total, (int)(max_len - 1 - total));
        if (r <= 0) break;
        total += (size_t)r;
    }
    buf[total] = '\0';
    return total;
}

// ------------------------------------------
// 1. ENVIO DE ALERTA DE ARROMBAMENTO + FOTO
// ------------------------------------------
static void processar_envio_arrombamento(const char *detalhes) {
    if (!wifi_manager_esta_conectado()) {
        ESP_LOGW(TAG, "Wi-Fi offline: nao foi possivel enviar alerta de arrombamento.");
        return;
    }

    size_t jpeg_len = 0;
    bool tem_foto = (s_jpeg_alerta != NULL) &&
                    olho_magico_capturar_jpeg(s_jpeg_alerta, CLOUD_JPEG_MAX_BYTES, &jpeg_len);

    char parte_campos[512];
    int campos_len = snprintf(
        parte_campos, sizeof(parte_campos),
        "--" CLOUD_BOUNDARY "\r\n"
        "Content-Disposition: form-data; name=\"tipo_evento\"\r\n\r\n"
        "ALARME_ARROMBAMENTO\r\n"
        "--" CLOUD_BOUNDARY "\r\n"
        "Content-Disposition: form-data; name=\"detalhes\"\r\n\r\n"
        "%s\r\n",
        (detalhes && detalhes[0] != '\0') ? detalhes : "Arrombamento detectado pelos sensores da fechadura."
    );

    const char *parte_img_head =
        "--" CLOUD_BOUNDARY "\r\n"
        "Content-Disposition: form-data; name=\"imagem\"; filename=\"alerta.jpg\"\r\n"
        "Content-Type: image/jpeg\r\n\r\n";
    const char *parte_tail_com_img = "\r\n--" CLOUD_BOUNDARY "--\r\n";
    const char *parte_tail_sem_img = "--" CLOUD_BOUNDARY "--\r\n";

    size_t head_len = strlen(parte_img_head);
    size_t tail_len = tem_foto ? strlen(parte_tail_com_img) : strlen(parte_tail_sem_img);
    int total_len = campos_len + (tem_foto ? (int)(head_len + jpeg_len) : 0) + (int)tail_len;

    esp_http_client_config_t cfg = {
        .url = API_BASE_URL "/esp32/evento",
        .method = HTTP_METHOD_POST,
        .timeout_ms = CLOUD_EVENTO_TIMEOUT_MS,
        .buffer_size = 2048,
        .buffer_size_tx = 4096,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return;

    esp_http_client_set_header(client, "Content-Type", "multipart/form-data; boundary=" CLOUD_BOUNDARY);

    if (esp_http_client_open(client, total_len) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao conectar em /esp32/evento.");
        esp_http_client_cleanup(client);
        return;
    }

    bool ok = enviar_tudo_http(client, (const uint8_t *)parte_campos, (size_t)campos_len);
    if (ok && tem_foto) {
        ok = enviar_tudo_http(client, (const uint8_t *)parte_img_head, head_len) &&
             enviar_tudo_http(client, s_jpeg_alerta, jpeg_len) &&
             enviar_tudo_http(client, (const uint8_t *)parte_tail_com_img, tail_len);
    } else if (ok) {
        ok = enviar_tudo_http(client, (const uint8_t *)parte_tail_sem_img, tail_len);
    }

    if (ok) {
        esp_http_client_fetch_headers(client);
        int status = esp_http_client_get_status_code(client);
        char resp[256];
        ler_corpo_http(client, resp, sizeof(resp));
        ESP_LOGW(TAG, "Alerta de arrombamento enviado (HTTP %d, foto=%s): %s",
                 status, tem_foto ? "SIM" : "NAO", resp);
    } else {
        ESP_LOGE(TAG, "Erro ao transmitir alerta de arrombamento.");
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

// ------------------------------------------
// 2. VALIDACAO DE PIN ONLINE (TECLADO 4x4)
// ------------------------------------------
static pin_online_resultado_t executar_validacao_pin_http(const char *pin,
                                                          bool desarmar_alarme,
                                                          char *nome_out,
                                                          size_t nome_len) {
    if (nome_out && nome_len > 0) nome_out[0] = '\0';

    if (!wifi_manager_esta_conectado()) {
        return PIN_ONLINE_OFFLINE;
    }

    char body[96];
    int body_len = snprintf(
        body, sizeof(body),
        "{\"pin\":\"%s\",\"desarmar_alarme\":%s}",
        pin,
        desarmar_alarme ? "true" : "false"
    );
    if (body_len <= 0 || body_len >= (int)sizeof(body)) {
        return PIN_ONLINE_OFFLINE;
    }

    esp_http_client_config_t cfg = {
        .url = API_BASE_URL "/esp32/validar-pin",
        .method = HTTP_METHOD_POST,
        .timeout_ms = CLOUD_HTTP_TIMEOUT_MS,
        .buffer_size = 1024,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return PIN_ONLINE_OFFLINE;

    esp_http_client_set_header(client, "Content-Type", "application/json");

    if (esp_http_client_open(client, body_len) != ESP_OK ||
        !enviar_tudo_http(client, (const uint8_t *)body, (size_t)body_len)) {
        ESP_LOGW(TAG, "Servidor inacessivel em /esp32/validar-pin -> acionando fallback offline.");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return PIN_ONLINE_OFFLINE;
    }

    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    char resp[512];
    ler_corpo_http(client, resp, sizeof(resp));

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (status == 200) {
        char nome_raw[64] = "";
        if (json_extrair_string(resp, "nome", nome_raw, sizeof(nome_raw)) &&
            nome_out && nome_len > 0) {
            normalizar_utf8_para_tft(nome_raw, nome_out, nome_len);
        }
        ESP_LOGI(TAG, "PIN aprovado online para: %s", (nome_out && nome_out[0]) ? nome_out : "MORADOR");
        return PIN_ONLINE_AUTORIZADO;
    }

    if (status == 401 || status == 403) {
        ESP_LOGW(TAG, "PIN negado pelo servidor (HTTP %d): %s", status, resp);
        return PIN_ONLINE_NEGADO;
    }

    ESP_LOGW(TAG, "Erro inesperado em /esp32/validar-pin (HTTP %d) -> fallback offline.", status);
    return PIN_ONLINE_OFFLINE;
}

// ------------------------------------------
// 3. POLLING DE ABERTURA REMOTA PELO APP
// ------------------------------------------
static void consultar_comando_remoto(void) {
    if (!wifi_manager_esta_conectado()) return;

    // Nao abre remotamente enquanto o alarme de arrombamento estiver disparado
    if (fechadura_fsm_obter_estado() == ESTADO_ALARME_ARROMBAMENTO) return;

    esp_http_client_config_t cfg = {
        .url = API_BASE_URL "/esp32/comando",
        .method = HTTP_METHOD_GET,
        .timeout_ms = CLOUD_HTTP_TIMEOUT_MS,
        .buffer_size = 1024,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return;

    if (esp_http_client_open(client, 0) != ESP_OK) {
        esp_http_client_cleanup(client);
        return;
    }

    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    char resp[512];
    ler_corpo_http(client, resp, sizeof(resp));

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (status != 200 || resp[0] == '\0') return;

    if (json_extrair_bool(resp, "tem_comando")) {
        char cmd_str[32] = "";
        if (json_extrair_string(resp, "comando", cmd_str, sizeof(cmd_str)) &&
            strcmp(cmd_str, "ABRIR_PORTA") == 0) {
            char nome_raw[64] = "";
            char nome_tft[32] = "APP REMOTO";
            if (json_extrair_string(resp, "nome", nome_raw, sizeof(nome_raw)) && nome_raw[0] != '\0') {
                normalizar_utf8_para_tft(nome_raw, nome_tft, sizeof(nome_tft));
            }

            ESP_LOGI(TAG, ">>> COMANDO REMOTO RECEBIDO DO APP! Solicitado por: %s", nome_tft);
            fechadura_fsm_definir_usuario_autorizado(nome_tft);
            fechadura_fsm_enviar_evento(EVT_ACESSO_REMOTO_OK, 0);
        }
    }
}

// ------------------------------------------
// TASK PRINCIPAL DE SINCRONIZACAO (CORE 0)
// ------------------------------------------
static void cloud_sync_task(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Task Cloud Sync iniciada (polling: %d ms | API: %s).", CLOUD_POLL_INTERVALO_MS, API_BASE_URL);

    while (1) {
        // Espera ate 1,5s ou acorda imediatamente se chegar PIN ou alarme
        xSemaphoreTake(s_sem_acordar, pdMS_TO_TICKS(CLOUD_POLL_INTERVALO_MS));

        // Prioridade 1: Alerta de arrombamento
        bool enviar_alarme = false;
        char detalhes_local[160] = "";
        portENTER_CRITICAL(&s_sync_mux);
        if (s_alarme_pendente) {
            s_alarme_pendente = false;
            strncpy(detalhes_local, s_alarme_detalhes, sizeof(detalhes_local) - 1);
            detalhes_local[sizeof(detalhes_local) - 1] = '\0';
            enviar_alarme = true;
        }
        portEXIT_CRITICAL(&s_sync_mux);

        if (enviar_alarme) {
            processar_envio_arrombamento(detalhes_local);
        }

        // Prioridade 2: Validacao de PIN digitado no teclado
        bool validar_pin = false;
        char pin_local[16] = "";
        bool desarmar_local = false;
        portENTER_CRITICAL(&s_sync_mux);
        if (s_pin_pendente) {
            s_pin_pendente = false;
            strncpy(pin_local, s_pin_req, sizeof(pin_local) - 1);
            pin_local[sizeof(pin_local) - 1] = '\0';
            desarmar_local = s_pin_req_desarmar;
            validar_pin = true;
        }
        portEXIT_CRITICAL(&s_sync_mux);

        if (validar_pin) {
            char nome_res[32] = "";
            pin_online_resultado_t res = executar_validacao_pin_http(
                pin_local, desarmar_local, nome_res, sizeof(nome_res)
            );
            portENTER_CRITICAL(&s_sync_mux);
            s_pin_resp_status = res;
            strncpy(s_pin_resp_nome, nome_res, sizeof(s_pin_resp_nome) - 1);
            s_pin_resp_nome[sizeof(s_pin_resp_nome) - 1] = '\0';
            portEXIT_CRITICAL(&s_sync_mux);

            xSemaphoreGive(s_sem_pin_pronto);
            continue;
        }

        // Prioridade 3: Polling de comando remoto do app
        consultar_comando_remoto();
    }
}

// ------------------------------------------
// API PUBLICA
// ------------------------------------------
void cloud_sync_notificar_arrombamento(const char *detalhes) {
    if (!s_sem_acordar) return;
    portENTER_CRITICAL(&s_sync_mux);
    s_alarme_pendente = true;
    strncpy(s_alarme_detalhes,
            (detalhes && detalhes[0]) ? detalhes : "Arrombamento detectado na fechadura!",
            sizeof(s_alarme_detalhes) - 1);
    s_alarme_detalhes[sizeof(s_alarme_detalhes) - 1] = '\0';
    portEXIT_CRITICAL(&s_sync_mux);

    xSemaphoreGive(s_sem_acordar);
}

pin_online_resultado_t cloud_sync_validar_pin(const char *pin,
                                              bool desarmar_alarme,
                                              char *nome_out,
                                              size_t nome_len) {
    if (!pin || !s_sem_acordar || !s_sem_pin_pronto || !s_mutex_pin_api) {
        return PIN_ONLINE_OFFLINE;
    }
    if (!wifi_manager_esta_conectado()) {
        return PIN_ONLINE_OFFLINE;
    }

    if (xSemaphoreTake(s_mutex_pin_api, pdMS_TO_TICKS(500)) != pdTRUE) {
        return PIN_ONLINE_OFFLINE;
    }

    xSemaphoreTake(s_sem_pin_pronto, 0);

    portENTER_CRITICAL(&s_sync_mux);
    strncpy(s_pin_req, pin, sizeof(s_pin_req) - 1);
    s_pin_req[sizeof(s_pin_req) - 1] = '\0';
    s_pin_req_desarmar = desarmar_alarme;
    s_pin_pendente = true;
    portEXIT_CRITICAL(&s_sync_mux);

    xSemaphoreGive(s_sem_acordar);

    pin_online_resultado_t resultado = PIN_ONLINE_OFFLINE;
    if (xSemaphoreTake(s_sem_pin_pronto, pdMS_TO_TICKS(CLOUD_HTTP_TIMEOUT_MS + 1000)) == pdTRUE) {
        portENTER_CRITICAL(&s_sync_mux);
        resultado = s_pin_resp_status;
        if (nome_out && nome_len > 0) {
            strncpy(nome_out, s_pin_resp_nome, nome_len - 1);
            nome_out[nome_len - 1] = '\0';
        }
        portEXIT_CRITICAL(&s_sync_mux);
    }

    xSemaphoreGive(s_mutex_pin_api);
    return resultado;
}

esp_err_t cloud_sync_iniciar(void) {
    s_jpeg_alerta = heap_caps_malloc(CLOUD_JPEG_MAX_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_jpeg_alerta) {
        ESP_LOGW(TAG, "Sem PSRAM para foto de alerta (alertas serao enviados sem imagem).");
    }

    s_sem_acordar = xSemaphoreCreateBinary();
    s_sem_pin_pronto = xSemaphoreCreateBinary();
    s_mutex_pin_api = xSemaphoreCreateMutex();
    if (!s_sem_acordar || !s_sem_pin_pronto || !s_mutex_pin_api) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(cloud_sync_task, "cloud_sync_task", 8192, NULL, 3, NULL, 0);
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}
