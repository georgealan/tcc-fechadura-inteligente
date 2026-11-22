#ifndef OLHO_MAGICO_H
#define OLHO_MAGICO_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Inicializa buffers, decodificador MJPEG (esp_new_jpeg) e task de renderizacao no Core 1
esp_err_t olho_magico_iniciar(void);

// Chamada no callback da camera UVC (nao-bloqueante: descarta frame se decoder ocupado)
void olho_magico_enviar_frame(const uint8_t *jpeg_data, size_t jpeg_len);

// Copia o ultimo frame JPEG valido da PSRAM para 'destino' (thread-safe, nao trava o video)
bool olho_magico_capturar_jpeg(uint8_t *destino, size_t max_len, size_t *out_len);

#endif // OLHO_MAGICO_H
