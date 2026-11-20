#ifndef CAMERA_UVC_H
#define CAMERA_UVC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Instala o USB Host + driver UVC e inicia as tasks de captura em segundo plano
esp_err_t camera_uvc_iniciar(void);

// Retorna true se a camera UVC estiver conectada e transmitindo frames
bool camera_uvc_esta_conectada(void);

// Captura o proximo frame JPEG valido para envio ao servidor (ex.: reconhecimento facial)
// Nao interrompe o video ao vivo do Olho Magico.
esp_err_t camera_uvc_capturar_foto(uint8_t *destino, size_t tam_max, size_t *tam_real, uint32_t timeout_ms);

#endif // CAMERA_UVC_H
