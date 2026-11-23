#ifndef CLOUD_SYNC_H
#define CLOUD_SYNC_H

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef enum {
    PIN_ONLINE_AUTORIZADO = 0, // HTTP 200: PIN valido (libera porta ou desarma alarme)
    PIN_ONLINE_NEGADO,         // HTTP 401/403: PIN invalido, visitante expirado ou sem permissao
    PIN_ONLINE_OFFLINE         // Wi-Fi desconectado ou servidor indisponivel -> usar fallback NVS
} pin_online_resultado_t;

// Aloca buffer na PSRAM e inicia a task de polling remoto, PIN online e alertas
esp_err_t cloud_sync_iniciar(void);

// Valida PIN digitado no teclado 4x4 contra o backend (executa na task com stack de 8 KB)
pin_online_resultado_t cloud_sync_validar_pin(const char *pin,
                                              bool desarmar_alarme,
                                              char *nome_out,
                                              size_t nome_len);

// Enfileira envio imediato (nao bloqueante) do alerta de arrombamento + foto JPEG da PSRAM
void cloud_sync_notificar_arrombamento(const char *detalhes);

#endif // CLOUD_SYNC_H
