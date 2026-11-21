#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef enum {
    WIFI_STATUS_DESCONECTADO = 0,
    WIFI_STATUS_BUSCANDO,
    WIFI_STATUS_CONECTANDO,
    WIFI_STATUS_CONECTADO
} wifi_status_t;

// Inicializa NVS, pilha TCP/IP e task de Multi-Wi-Fi em segundo plano (nao bloqueia o boot)
esp_err_t wifi_manager_iniciar(void);

// Retorna true se o ESP32-S3 estiver conectado e com IP valido
bool wifi_manager_esta_conectado(void);

// Consulta thread-safe do status atual, IP e SSID conectado (para o HUD e telemetria)
wifi_status_t wifi_manager_obter_info(char *ip_buf, size_t ip_len, char *ssid_buf, size_t ssid_len);

#endif // WIFI_MANAGER_H
// FIM DO ARQUIVO
