#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "usb/usb_host.h"
#include "usb/uvc_host.h"

#define EXAMPLE_USB_HOST_PRIORITY   (15)
#define EXAMPLE_FRAME_COUNT         (2)
#define UVC_DESC_DWFRAMEINTERVAL_TO_FPS(dwFrameInterval) (((dwFrameInterval) != 0) ? 10000000 / ((float)(dwFrameInterval)) : 0)

static const char *TAG = "CAMERA_USB";
static QueueHandle_t rx_frames_queue;
static bool dev_connected = false;

// Callback 1: Chamado para cada frame (imagem) recebido
bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx) {
    QueueHandle_t frame_q = *((QueueHandle_t *)user_ctx);
    ESP_LOGI(TAG, "Frame recebido! Tamanho: %d bytes", frame->data_len);

    // Passa o frame para a Queue sem bloquear o USB
    BaseType_t result = xQueueSendToBack(frame_q, &frame, 0);
    if (pdPASS != result) {
        return true; // Fila cheia, descartar frame
    }
    return false;
}

// Callback 2: Eventos da stream (erros, desconexões)
static void stream_callback(const uvc_host_stream_event_data_t *event, void *user_ctx) {
    if (event->type == UVC_HOST_DEVICE_DISCONNECTED) {
        ESP_LOGI(TAG, "Câmera desconectada subitamente!");
        dev_connected = false;
        uvc_host_stream_close(event->device_disconnected.stream_hdl);
    }
}

// Tarefa 1: Processa as imagens enviadas pela câmera
static void frame_handling_task(void *arg) {
    uvc_host_stream_config_t *stream_config = (uvc_host_stream_config_t *)arg;
    QueueHandle_t frame_q = *((QueueHandle_t *)(stream_config->user_ctx));
    uvc_host_stream_hdl_t uvc_stream = NULL;

    ESP_LOGI(TAG, "Abrindo Câmera UVC...");
    esp_err_t err = uvc_host_stream_open(stream_config, pdMS_TO_TICKS(5000), &uvc_stream);

    if (ESP_OK == err) {
        dev_connected = true;
        ESP_LOGI(TAG, "Câmera ABERTA! Iniciando captura...");
        uvc_host_stream_start(uvc_stream);

        while (dev_connected) {
            uvc_host_frame_t *frame;
            // Espera até 5 segundos por um novo frame na fila
            if (xQueueReceive(frame_q, &frame, pdMS_TO_TICKS(5000)) == pdPASS) {

                // AQUI É ONDE VOCÊ TRABALHA COM A IMAGEM!
                // Os bytes da imagem JPEG estão em: frame->data

                // Após usar a imagem, tem de a devolver ao sistema USB
                uvc_host_frame_return(uvc_stream, frame);
            } else {
                ESP_LOGW(TAG, "Timeout: Nenhum frame recebido da câmera.");
            }
        }
    }
    vTaskDelete(NULL); // Termina a tarefa se a câmera falhar
}

// Callback 3: Quando a câmera é espetada fisicamente no cabo
static void uvc_event_cb(const uvc_host_driver_event_data_t *event, void *user_ctx) {
    if (event->type == UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) {
        ESP_LOGI(TAG, "Câmera Conectada na porta USB!");

        static uvc_host_stream_config_t stream_config = {
            .event_cb = stream_callback,
            .frame_cb = frame_callback,
            .user_ctx = &rx_frames_queue,
            .usb = {
                .vid = UVC_HOST_ANY_VID,
                .pid = UVC_HOST_ANY_PID,
                .uvc_stream_index = 0,
            },
            // Altere a resolução solicitada
            .vs_format = {
                .h_res = 480,
                .v_res = 320,
                .fps = 15,
                .format = UVC_VS_FORMAT_MJPEG,
            },
            .advanced = {
                .number_of_frame_buffers = EXAMPLE_FRAME_COUNT,
                // Aumente a memória máxima alocada para caber a imagem HD
                .frame_size = 480 * 320 * 2, // Tamanho máximo estimado, aumente o cálculo aqui
                .number_of_urbs = 4,
                .urb_size = 10 * 1024,
            },
        };

        // Inicia a tarefa de captura de imagem
        xTaskCreatePinnedToCore(frame_handling_task, "uvc_handling", 4096, (void *)&stream_config, EXAMPLE_USB_HOST_PRIORITY - 2, NULL, tskNO_AFFINITY);
    }
}

// Tarefa 2: Mantém o driver USB vivo em segundo plano
static void usb_lib_task(void *arg) {
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Fechadura Inteligente (TCC)...");

    // Prepara a fila para receber os frames de imagem
    rx_frames_queue = xQueueCreate(EXAMPLE_FRAME_COUNT, sizeof(uvc_host_frame_t *));

    // 1. Instala o Driver Base USB
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED, // Note que a flag correta no v6.1 é LOWMED, não LEVEL1
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Inicia a tarefa base do USB
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, EXAMPLE_USB_HOST_PRIORITY, NULL, tskNO_AFFINITY);

    // 2. Instala o Driver UVC (Câmera)
    const uvc_host_driver_config_t uvc_driver_config = {
        .driver_task_stack_size = 4 * 1024,
        .driver_task_priority = EXAMPLE_USB_HOST_PRIORITY + 1,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = uvc_event_cb,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_driver_config));

    ESP_LOGI(TAG, "Sistema pronto. Aguardando conexao da camera OTG...");
}
