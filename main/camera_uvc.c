#include "camera_uvc.h"
#include "olho_magico.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "usb/usb_host.h"
#include "usb/uvc_host.h"

#define UVC_HOST_PRIORITY       (15)
#define UVC_FRAME_COUNT         (2)
#define UVC_JPG_MIN_SIZE        (40 * 1024)
#define UVC_JPG_COMP_RATIO      (6)
#define UVC_MAX(a, b)           ((a) > (b) ? (a) : (b))
#define UVC_ESTIMATE_JPG_SIZE_B(width, height, bpp) \
    (UVC_MAX(((width) * (height) * (bpp) / 8) / UVC_JPG_COMP_RATIO, UVC_JPG_MIN_SIZE))

static const char *TAG = "CAMERA_UVC";

static QueueHandle_t s_rx_frames_queue = NULL;
static volatile bool s_dev_connected = false;

// Controle de snapshot sob demanda (para envio ao backend de reconhecimento facial)
static SemaphoreHandle_t s_foto_mutex = NULL;
static SemaphoreHandle_t s_sem_foto_pronta = NULL;
static volatile bool s_foto_solicitada = false;
static uint8_t *s_foto_destino = NULL;
static size_t s_foto_tam_max = 0;
static size_t s_foto_tam_real = 0;
static esp_err_t s_foto_status = ESP_ERR_TIMEOUT;

static void atender_captura_foto(const uint8_t *data, size_t len) {
    // Garante assinatura SOI valida de imagem JPEG (0xFF 0xD8)
    if (len < 4 || data[0] != 0xFF || data[1] != 0xD8) {
        return;
    }

    if (xSemaphoreTake(s_foto_mutex, 0) == pdTRUE) {
        if (s_foto_solicitada && s_foto_destino != NULL) {
            if (len <= s_foto_tam_max) {
                memcpy(s_foto_destino, data, len);
                s_foto_tam_real = len;
                s_foto_status = ESP_OK;
            } else {
                s_foto_tam_real = 0;
                s_foto_status = ESP_ERR_NO_MEM;
            }
            s_foto_solicitada = false;
            xSemaphoreGive(s_sem_foto_pronta);
        }
        xSemaphoreGive(s_foto_mutex);
    }
}

// Callback 1: Chamado no contexto do driver USB a cada frame recebido (mantido ultraleve)
static bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx) {
    QueueHandle_t frame_q = *((QueueHandle_t *)user_ctx);
    BaseType_t result = xQueueSendToBack(frame_q, &frame, 0);
    if (pdPASS != result) {
        return true; // Fila cheia: devolve buffer imediatamente ao driver
    }
    return false;
}

// Callback 2: Eventos da stream (erros, overflow, desconexao)
static void stream_callback(const uvc_host_stream_event_data_t *event, void *user_ctx) {
    if (event->type == UVC_HOST_TRANSFER_ERROR) {
        ESP_LOGE(TAG, "Erro de transferencia USB: %d", event->transfer_error.error);
    } else if (event->type == UVC_HOST_FRAME_BUFFER_OVERFLOW) {
        ESP_LOGW(TAG, "Buffer de frame insuficiente; aumente frame_size");
    } else if (event->type == UVC_HOST_DEVICE_DISCONNECTED) {
        ESP_LOGW(TAG, "Camera desconectada subitamente!");
        s_dev_connected = false;
        uvc_host_stream_close(event->device_disconnected.stream_hdl);
    }
}

// Tarefa 1: Distribui cada frame recebido para o Olho Magico e para capturas sob demanda
static void frame_handling_task(void *arg) {
    uvc_host_stream_config_t *stream_config = (uvc_host_stream_config_t *)arg;
    QueueHandle_t frame_q = *((QueueHandle_t *)(stream_config->user_ctx));
    uvc_host_stream_hdl_t uvc_stream = NULL;

    ESP_LOGI(TAG, "Abrindo Camera UVC...");
    esp_err_t err = uvc_host_stream_open(stream_config, pdMS_TO_TICKS(5000), &uvc_stream);

    if (ESP_OK == err) {
        s_dev_connected = true;
        ESP_LOGI(TAG, "Camera ABERTA! Iniciando captura (480x320 @ 15 FPS)...");
        uvc_host_stream_start(uvc_stream);

        while (s_dev_connected) {
            uvc_host_frame_t *frame = NULL;
            if (xQueueReceive(frame_q, &frame, pdMS_TO_TICKS(5000)) == pdPASS) {
                if (frame != NULL && frame->data != NULL && frame->data_len >= 4) {
                    // 1. Alimenta o Olho Magico ao vivo (descarta se decoder ocupado)
                    olho_magico_enviar_frame(frame->data, frame->data_len);

                    // 2. Se a API de reconhecimento facial pediu uma foto, copia o JPEG aqui
                    if (s_foto_solicitada) {
                        atender_captura_foto(frame->data, frame->data_len);
                    }
                }

                // Devolve o frame ao driver USB
                uvc_host_frame_return(uvc_stream, frame);
            } else {
                ESP_LOGW(TAG, "Timeout: nenhum frame recebido da camera.");
            }
        }
    } else {
        ESP_LOGE(TAG, "Falha ao abrir stream UVC: %s", esp_err_to_name(err));
    }

    vTaskDelete(NULL);
}

// Callback 3: Quando a camera e conectada na porta USB OTG
static void uvc_event_cb(const uvc_host_driver_event_data_t *event, void *user_ctx) {
    if (event->type == UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) {
        ESP_LOGI(TAG, "Camera conectada na porta USB!");

        static uvc_host_stream_config_t stream_config = {
            .event_cb = stream_callback,
            .frame_cb = frame_callback,
            .user_ctx = &s_rx_frames_queue,
            .usb = {
                .vid = UVC_HOST_ANY_VID,
                .pid = UVC_HOST_ANY_PID,
                .uvc_stream_index = 0,
            },
            .vs_format = {
                .h_res = 480,
                .v_res = 320,
                .fps = 15,
                .format = UVC_VS_FORMAT_MJPEG,
            },
            .advanced = {
                .number_of_frame_buffers = UVC_FRAME_COUNT,
                .frame_size = UVC_ESTIMATE_JPG_SIZE_B(480, 320, 16),
                .number_of_urbs = 4,
                .urb_size = 10 * 1024,
            },
        };

        xTaskCreatePinnedToCore(frame_handling_task, "uvc_handling", 4096,
                                (void *)&stream_config, UVC_HOST_PRIORITY - 2, NULL, tskNO_AFFINITY);
    }
}

// Tarefa 2: Mantem a pilha USB Host ativa em segundo plano
static void usb_lib_task(void *arg) {
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

bool camera_uvc_esta_conectada(void) {
    return s_dev_connected;
}

esp_err_t camera_uvc_capturar_foto(uint8_t *destino, size_t tam_max, size_t *tam_real, uint32_t timeout_ms) {
    if (destino == NULL || tam_max == 0 || tam_real == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_dev_connected || s_foto_mutex == NULL || s_sem_foto_pronta == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_foto_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_foto_solicitada) {
        xSemaphoreGive(s_foto_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    // Descarta eventual sinal antigo antes de esperar o novo frame
    xSemaphoreTake(s_sem_foto_pronta, 0);

    s_foto_destino = destino;
    s_foto_tam_max = tam_max;
    s_foto_tam_real = 0;
    s_foto_status = ESP_ERR_TIMEOUT;
    s_foto_solicitada = true;

    xSemaphoreGive(s_foto_mutex);

    if (xSemaphoreTake(s_sem_foto_pronta, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        xSemaphoreTake(s_foto_mutex, portMAX_DELAY);
        esp_err_t status = s_foto_status;
        *tam_real = s_foto_tam_real;
        s_foto_destino = NULL;
        xSemaphoreGive(s_foto_mutex);
        return status;
    }

    // Timeout: cancela pedido para nao escrever em ponteiro expirado
    xSemaphoreTake(s_foto_mutex, portMAX_DELAY);
    s_foto_solicitada = false;
    s_foto_destino = NULL;
    xSemaphoreGive(s_foto_mutex);
    *tam_real = 0;
    return ESP_ERR_TIMEOUT;
}

esp_err_t camera_uvc_iniciar(void) {
    s_rx_frames_queue = xQueueCreate(UVC_FRAME_COUNT, sizeof(uvc_host_frame_t *));
    s_foto_mutex = xSemaphoreCreateMutex();
    s_sem_foto_pronta = xSemaphoreCreateBinary();
    if (s_rx_frames_queue == NULL || s_foto_mutex == NULL || s_sem_foto_pronta == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) return err;

    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, UVC_HOST_PRIORITY, NULL, tskNO_AFFINITY);

    const uvc_host_driver_config_t uvc_driver_config = {
        .driver_task_stack_size = 8 * 1024,
        .driver_task_priority = UVC_HOST_PRIORITY + 1,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = uvc_event_cb,
    };
    err = uvc_host_install(&uvc_driver_config);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Driver USB Host / UVC iniciado. Aguardando camera OTG...");
    return ESP_OK;
}
