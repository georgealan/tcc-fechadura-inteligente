#ifndef OLHO_MAGICO_H
#define OLHO_MAGICO_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Inicializa buffers, decodificador MJPEG (esp_new_jpeg) e task de renderizacao no Core 1
esp_err_t olho_magico_iniciar(void);

// Chamada no callback da camera UVC (nao-bloqueante: descarta frame se decoder ocupado)
void olho_magico_enviar_frame(const uint8_t *jpeg_data, size_t jpeg_len);

#endif // OLHO_MAGICO_H
// FIM DO ARQUIVO
