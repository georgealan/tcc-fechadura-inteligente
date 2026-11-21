#include "wifi_manager.h"
#include "wifi_credenciais.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_log.h"

static const char *TAG = "WIFI_MGR";

#define WIFI_MAX_SCAN_APS           20
#define WIFI_TIMEOUT_CONEXAO_MS     12000
#define WIFI_INTERVALO_RETRY_MS     5000

#define BIT_STA_INICIADO            BIT0
#define BIT_CONECTADO               BIT1
#define BIT_DESCONECTADO            BIT2

static EventGroupHandle_t s_wifi_eg = NULL;
static portMUX_TYPE s_wifi_mux = portMUX_INITIALIZER_UNLOCKED;

static wifi_status_t s_status = WIFI_STATUS_DESCONECTADO;
static char s_ip_atual[16] = "0.0.0.0";
static char s_ssid_atual[33] = "";
static bool s_sntp_iniciado = false;

// Buffer estatico para nao consumir stack da task durante o scan
static wifi_ap_record_t s_ap_list[WIFI_MAX_SCAN_APS];

static void definir_estado(wifi_status_t novo_status, const char *ip, const char *ssid) {
    portENTER_CRITICAL(&s_wifi_mux);
    s_status = novo_status;
    if (ip != NULL) {
        strncpy(s_ip_atual, ip, sizeof(s_ip_atual) - 1);
        s_ip_atual[sizeof(s_ip_atual) - 1] = '\0';
    }
    if (ssid != NULL) {
        strncpy(s_ssid_atual, ssid, sizeof(s_ssid_atual) - 1);
        s_ssid_atual[sizeof(s_ssid_atual) - 1] = '\0';
    }
    portEXIT_CRITICAL(&s_wifi_mux);
}

static void iniciar_sntp_uma_vez(void) {
    if (s_sntp_iniciado) return;

    // Fuso horario oficial de Brasilia (UTC-3, sem horario de verao)
    setenv("TZ", "BRT3", 1);
    tzset();

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    if (esp_netif_sntp_init(&config) == ESP_OK) {
        s_sntp_iniciado = true;
        ESP_LOGI(TAG, "SNTP iniciado (fuso BRT3 / pool.ntp.org).");
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    (void)arg;

    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            xEventGroupSetBits(s_wifi_eg, BIT_STA_INICIADO);
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGW(TAG, "Wi-Fi desconectado (motivo=%u).", disc ? disc->reason : 0);
            definir_estado(WIFI_STATUS_DESCONECTADO, "0.0.0.0", "");
            xEventGroupClearBits(s_wifi_eg, BIT_CONECTADO);
            xEventGroupSetBits(s_wifi_eg, BIT_DESCONECTADO);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        char ip_buf[16];
        snprintf(ip_buf, sizeof(ip_buf), IPSTR, IP2STR(&event->ip_info.ip));

        definir_estado(WIFI_STATUS_CONECTADO, ip_buf, NULL);
        ESP_LOGI(TAG, ">>> WIFI CONECTADO! SSID: '%s' | IP: %s", s_ssid_atual, ip_buf);

        iniciar_sntp_uma_vez();
        xEventGroupClearBits(s_wifi_eg, BIT_DESCONECTADO);
        xEventGroupSetBits(s_wifi_eg, BIT_CONECTADO);
    }
}

// Escaneia redes 2,4 GHz e retorna o indice em WIFI_REDES_CONHECIDAS com melhor RSSI (-1 se nenhuma)
static int buscar_melhor_rede_conhecida(int8_t *melhor_rssi_out) {
    wifi_scan_config_t scan_cfg = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 250,
    };

    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Falha no scan Wi-Fi: %s", esp_err_to_name(err));
        return -1;
    }

    uint16_t qtd_aps = WIFI_MAX_SCAN_APS;
    err = esp_wifi_scan_get_ap_records(&qtd_aps, s_ap_list);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao ler lista de APs: %s", esp_err_to_name(err));
        return -1;
    }

    int melhor_idx = -1;
    int8_t melhor_rssi = -127;

    for (uint16_t i = 0; i < qtd_aps; i++) {
        const char *ap_ssid = (const char *)s_ap_list[i].ssid;
        if (ap_ssid[0] == '\0') continue;

        for (size_t j = 0; j < WIFI_QTD_REDES_CONHECIDAS; j++) {
            const char *cand = WIFI_REDES_CONHECIDAS[j].ssid;
            if (cand == NULL || cand[0] == '\0') continue;

            if (strcmp(ap_ssid, cand) == 0 && s_ap_list[i].rssi > melhor_rssi) {
                melhor_rssi = s_ap_list[i].rssi;
                melhor_idx = (int)j;
            }
        }
    }

    if (melhor_rssi_out != NULL) {
        *melhor_rssi_out = melhor_rssi;
    }
    return melhor_idx;
}

static void wifi_mgr_task(void *arg) {
    (void)arg;

    xEventGroupWaitBits(s_wifi_eg, BIT_STA_INICIADO, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Gerenciador Multi-Wi-Fi pronto (%u slot(s) configurado(s)).",
             (unsigned)WIFI_QTD_REDES_CONHECIDAS);

    while (1) {
        definir_estado(WIFI_STATUS_BUSCANDO, "0.0.0.0", "");
        ESP_LOGI(TAG, "Escaneando redes 2,4 GHz conhecidas...");

        int8_t rssi = -127;
        int idx = buscar_melhor_rede_conhecida(&rssi);

        if (idx < 0) {
            ESP_LOGW(TAG, "Nenhuma rede conhecida encontrada. Nova tentativa em %d s...",
                     WIFI_INTERVALO_RETRY_MS / 1000);
            definir_estado(WIFI_STATUS_DESCONECTADO, "0.0.0.0", "");
            vTaskDelay(pdMS_TO_TICKS(WIFI_INTERVALO_RETRY_MS));
            continue;
        }

        const wifi_rede_conhecida_t *rede = &WIFI_REDES_CONHECIDAS[idx];
        ESP_LOGI(TAG, "Rede encontrada: '%s' (RSSI=%d dBm). Conectando...", rede->ssid, (int)rssi);

        wifi_config_t wifi_cfg = {0};
        strncpy((char *)wifi_cfg.sta.ssid, rede->ssid, sizeof(wifi_cfg.sta.ssid) - 1);
        if (rede->senha != NULL && rede->senha[0] != '\0') {
            strncpy((char *)wifi_cfg.sta.password, rede->senha, sizeof(wifi_cfg.sta.password) - 1);
            wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
        } else {
            wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
        }
        wifi_cfg.sta.pmf_cfg.capable = true;
        wifi_cfg.sta.pmf_cfg.required = false;

        definir_estado(WIFI_STATUS_CONECTANDO, "0.0.0.0", rede->ssid);
        xEventGroupClearBits(s_wifi_eg, BIT_CONECTADO | BIT_DESCONECTADO);

        if (esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg) != ESP_OK ||
            esp_wifi_connect() != ESP_OK) {
            ESP_LOGE(TAG, "Erro ao iniciar conexao com '%s'.", rede->ssid);
            definir_estado(WIFI_STATUS_DESCONECTADO, "0.0.0.0", "");
            vTaskDelay(pdMS_TO_TICKS(WIFI_INTERVALO_RETRY_MS));
            continue;
        }

        EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                               BIT_CONECTADO | BIT_DESCONECTADO,
                                               pdFALSE,
                                               pdFALSE,
                                               pdMS_TO_TICKS(WIFI_TIMEOUT_CONEXAO_MS));

        if (bits & BIT_CONECTADO) {
            // Fica ocioso aqui ate cair a conexao (zero consumo de CPU enquanto conectado)
            xEventGroupWaitBits(s_wifi_eg, BIT_DESCONECTADO, pdFALSE, pdTRUE, portMAX_DELAY);
            ESP_LOGW(TAG, "Conexao perdida. Aguardando para reconectar...");
            vTaskDelay(pdMS_TO_TICKS(3000));
        } else {
            ESP_LOGW(TAG, "Nao foi possivel conectar em '%s' (verifique senha ou sinal).", rede->ssid);
            esp_wifi_disconnect();
            definir_estado(WIFI_STATUS_DESCONECTADO, "0.0.0.0", "");
            vTaskDelay(pdMS_TO_TICKS(WIFI_INTERVALO_RETRY_MS));
        }
    }
}

bool wifi_manager_esta_conectado(void) {
    portENTER_CRITICAL(&s_wifi_mux);
    bool ok = (s_status == WIFI_STATUS_CONECTADO);
    portEXIT_CRITICAL(&s_wifi_mux);
    return ok;
}

wifi_status_t wifi_manager_obter_info(char *ip_buf, size_t ip_len, char *ssid_buf, size_t ssid_len) {
    portENTER_CRITICAL(&s_wifi_mux);
    wifi_status_t st = s_status;
    if (ip_buf != NULL && ip_len > 0) {
        strncpy(ip_buf, s_ip_atual, ip_len - 1);
        ip_buf[ip_len - 1] = '\0';
    }
    if (ssid_buf != NULL && ssid_len > 0) {
        strncpy(ssid_buf, s_ssid_atual, ssid_len - 1);
        ssid_buf[ssid_len - 1] = '\0';
    }
    portEXIT_CRITICAL(&s_wifi_mux);
    return st;
}

esp_err_t wifi_manager_iniciar(void) {
    // 1. Inicializa NVS (obrigatorio para o Wi-Fi e util depois para senhas da fechadura)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;

    // 2. Pilha de rede e loop padrao de eventos
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    esp_netif_create_default_wifi_sta();

    // 3. Driver Wi-Fi em modo Station
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) return err;

    s_wifi_eg = xEventGroupCreate();
    if (s_wifi_eg == NULL) return ESP_ERR_NO_MEM;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    xTaskCreatePinnedToCore(wifi_mgr_task, "wifi_mgr_task", 4096, NULL, 3, NULL, 0);
    return ESP_OK;
}
// FIM DO ARQUIVO
