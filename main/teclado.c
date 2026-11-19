#include "teclado.h"
#include "pinagem.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "TECLADO_4X4";

static const gpio_num_t PINOS_LINHAS[4] = {
    PIN_TECLADO_L1, PIN_TECLADO_L2, PIN_TECLADO_L3, PIN_TECLADO_L4
};

static const gpio_num_t PINOS_COLUNAS[4] = {
    PIN_TECLADO_C1, PIN_TECLADO_C2, PIN_TECLADO_C3, PIN_TECLADO_C4
};

static const char MAPA_TECLAS[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

static void configurar_gpios_teclado(void) {
    // Linhas em Open-Drain com pull-up: evita curto se duas teclas forem apertadas juntas
    uint64_t mascara_linhas = 0;
    for (int i = 0; i < 4; i++) {
        mascara_linhas |= (1ULL << PINOS_LINHAS[i]);
    }

    gpio_config_t cfg_linhas = {
        .pin_bit_mask = mascara_linhas,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg_linhas);

    for (int i = 0; i < 4; i++) {
        gpio_set_level(PINOS_LINHAS[i], 1);
    }

    // Colunas como Entrada com Pull-Up interno (repouso = 1, pressionada = 0)
    uint64_t mascara_colunas = 0;
    for (int i = 0; i < 4; i++) {
        mascara_colunas |= (1ULL << PINOS_COLUNAS[i]);
    }

    gpio_config_t cfg_colunas = {
        .pin_bit_mask = mascara_colunas,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg_colunas);
}

static char escanear_matriz(void) {
    char tecla_detectada = '\0';

    for (int l = 0; l < 4; l++) {
        gpio_set_level(PINOS_LINHAS[l], 0);
        esp_rom_delay_us(15); // Tempo de estabilização da linha na protoboard

        for (int c = 0; c < 4; c++) {
            if (gpio_get_level(PINOS_COLUNAS[c]) == 0) {
                tecla_detectada = MAPA_TECLAS[l][c];
                break;
            }
        }

        gpio_set_level(PINOS_LINHAS[l], 1);

        if (tecla_detectada != '\0') {
            break;
        }
    }

    return tecla_detectada;
}

static void teclado_task(void *arg) {
    configurar_gpios_teclado();
    ESP_LOGI(TAG, "Teclado 4x4 iniciado nos GPIOs 1,2,42,41 (L) e 40,39,38,47 (C).");

    char ultima_tecla = '\0';

    while (1) {
        char tecla_atual = escanear_matriz();

        // Filtro de debounce (25 ms) para evitar leituras duplicadas
        if (tecla_atual != '\0' && tecla_atual != ultima_tecla) {
            vTaskDelay(pdMS_TO_TICKS(25));
            if (escanear_matriz() == tecla_atual) {
                ESP_LOGI(TAG, "Tecla pressionada: [ %c ]", tecla_atual);
                // Futuro: enviar 'tecla_atual' via xQueueSend() para a Máquina de Estados
                ultima_tecla = tecla_atual;
            }
        } else if (tecla_atual == '\0') {
            ultima_tecla = '\0';
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void teclado_iniciar(void) {
    // Prioridade 5: bem abaixo da prioridade 15 do USB Host para nunca interferir na câmera
    xTaskCreatePinnedToCore(teclado_task, "teclado_task", 3072, NULL, 5, NULL, tskNO_AFFINITY);
}
