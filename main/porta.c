#include "porta.h"
#include "pinagem.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "audio_i2s.h"

static const char *TAG = "PORTA";

// --- Ajustes de polaridade do hardware ---
// Modulo rele azul com optoacoplador normalmente e ativo em LOW (0 = aciona rele / destranca).
// Se o seu rele ligar invertido no teste, troque RELE_NIVEL_ATIVO para 1.
#define RELE_NIVEL_ATIVO            0
#define RELE_NIVEL_INATIVO          (RELE_NIVEL_ATIVO ? 0 : 1)

// MC-38 com pull-up interno:
// 0 = reed switch FECHA quando o ima encosta (puxa GPIO 18 para GND)
// 1 = reed switch ABRE quando o ima encosta (solta GPIO 18 para 3,3 V)
#define MC38_NIVEL_PORTA_FECHADA    0

// Botao de saida ligado entre GPIO 6 e GND (pull-up interno)
#define BOTAO_NIVEL_PRESSIONADO     0

// Limite de seguranca para nunca superaquecer a bobina da solenoide 12V
#define PORTA_TEMPO_MAX_MS          10000
#define PORTA_POLL_MS               20
#define DEBOUNCE_AMOSTRAS           3   // 3 x 20 ms = 60 ms

static SemaphoreHandle_t s_porta_mutex = NULL;
static volatile bool s_destrancada = false;
static volatile bool s_porta_fechada = false;
static int64_t s_instante_trancar_us = 0;

static void aplicar_rele_locked(bool destrancar) {
    gpio_set_level(PIN_RELE_SOLENOIDE, destrancar ? RELE_NIVEL_ATIVO : RELE_NIVEL_INATIVO);
    s_destrancada = destrancar;
}

void porta_destrancar(uint32_t duracao_ms) {
    if (s_porta_mutex == NULL) return;

    if (duracao_ms == 0) {
        duracao_ms = PORTA_TEMPO_ABERTURA_PADRAO_MS;
    } else if (duracao_ms > PORTA_TEMPO_MAX_MS) {
        duracao_ms = PORTA_TEMPO_MAX_MS;
    }

    if (xSemaphoreTake(s_porta_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        aplicar_rele_locked(true);
        s_instante_trancar_us = esp_timer_get_time() + ((int64_t)duracao_ms * 1000LL);
        xSemaphoreGive(s_porta_mutex);
        ESP_LOGI(TAG, "SOLENOIDE ACIONADA -> Porta DESTRANCADA por %lu ms.", (unsigned long)duracao_ms);
        audio_i2s_tocar(AUDIO_SOM_ACESSO_OK);
    }
}

void porta_trancar(void) {
    if (s_porta_mutex == NULL) return;

    if (xSemaphoreTake(s_porta_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        aplicar_rele_locked(false);
        s_instante_trancar_us = 0;
        xSemaphoreGive(s_porta_mutex);
        ESP_LOGI(TAG, "SOLENOIDE DESLIGADA -> Porta TRANCADA.");
    }
}

bool porta_esta_destrancada(void) {
    return s_destrancada;
}

bool porta_esta_fechada(void) {
    return s_porta_fechada;
}

static void porta_task(void *arg) {
    (void)arg;

    // Leitura inicial dos sensores
    int nivel_botao_estavel = gpio_get_level(PIN_BOTAO_SAIDA);
    int nivel_botao_candidato = nivel_botao_estavel;
    int cont_botao = 0;

    int nivel_mc38_estavel = gpio_get_level(PIN_SENSOR_PORTA);
    int nivel_mc38_candidato = nivel_mc38_estavel;
    int cont_mc38 = 0;

    s_porta_fechada = (nivel_mc38_estavel == MC38_NIVEL_PORTA_FECHADA);
    ESP_LOGI(TAG, "Estado inicial MC-38 (GPIO%d = %d): Porta %s",
             PIN_SENSOR_PORTA, nivel_mc38_estavel, s_porta_fechada ? "FECHADA" : "ABERTA");

    while (1) {
        // 1. Verifica temporizador de retrancamento automatico da solenoide
        if (s_destrancada) {
            bool expirado = false;
            if (xSemaphoreTake(s_porta_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (s_destrancada && s_instante_trancar_us > 0 && esp_timer_get_time() >= s_instante_trancar_us) {
                    aplicar_rele_locked(false);
                    s_instante_trancar_us = 0;
                    expirado = true;
                }
                xSemaphoreGive(s_porta_mutex);
            }
            if (expirado) {
                ESP_LOGI(TAG, "Tempo expirado -> Solenoide desligada (Porta TRANCADA).");
            }
        }

        // 2. Debounce do Botao Interno de Saida (GPIO 6)
        int leitura_botao = gpio_get_level(PIN_BOTAO_SAIDA);
        if (leitura_botao == nivel_botao_candidato) {
            if (cont_botao < DEBOUNCE_AMOSTRAS) {
                cont_botao++;
            }
        } else {
            nivel_botao_candidato = leitura_botao;
            cont_botao = 1;
        }

        if (cont_botao >= DEBOUNCE_AMOSTRAS && nivel_botao_candidato != nivel_botao_estavel) {
            nivel_botao_estavel = nivel_botao_candidato;
            if (nivel_botao_estavel == BOTAO_NIVEL_PRESSIONADO) {
                ESP_LOGW(TAG, ">>> BOTAO DE SAIDA PRESSIONADO! Acionando abertura...");
                porta_destrancar(PORTA_TEMPO_ABERTURA_PADRAO_MS);
            }
        }

        // 3. Debounce do Sensor Magnetico MC-38 (GPIO 18)
        int leitura_mc38 = gpio_get_level(PIN_SENSOR_PORTA);
        if (leitura_mc38 == nivel_mc38_candidato) {
            if (cont_mc38 < DEBOUNCE_AMOSTRAS) {
                cont_mc38++;
            }
        } else {
            nivel_mc38_candidato = leitura_mc38;
            cont_mc38 = 1;
        }

        if (cont_mc38 >= DEBOUNCE_AMOSTRAS && nivel_mc38_candidato != nivel_mc38_estavel) {
            nivel_mc38_estavel = nivel_mc38_candidato;
            s_porta_fechada = (nivel_mc38_estavel == MC38_NIVEL_PORTA_FECHADA);
            ESP_LOGI(TAG, ">>> SENSOR MC-38 (GPIO%d = %d): Folha da porta %s!",
                     PIN_SENSOR_PORTA, nivel_mc38_estavel, s_porta_fechada ? "FECHADA" : "ABERTA");
        }

        vTaskDelay(pdMS_TO_TICKS(PORTA_POLL_MS));
    }
}

esp_err_t porta_iniciar(void) {
    s_porta_mutex = xSemaphoreCreateMutex();
    if (s_porta_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Pre-carrega nivel INATIVO antes de habilitar saida para evitar estalo no boot
    gpio_set_level(PIN_RELE_SOLENOIDE, RELE_NIVEL_INATIVO);

    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << PIN_RELE_SOLENOIDE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&out_cfg);
    if (err != ESP_OK) return err;

    // Garante novamente estado trancado
    gpio_set_level(PIN_RELE_SOLENOIDE, RELE_NIVEL_INATIVO);
    s_destrancada = false;

    // Configura entradas com pull-up interno (Botao de saida + MC-38)
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << PIN_BOTAO_SAIDA) | (1ULL << PIN_SENSOR_PORTA),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&in_cfg);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Controle de porta iniciado (Rele=GPIO%d, Botao=GPIO%d, MC38=GPIO%d).",
             PIN_RELE_SOLENOIDE, PIN_BOTAO_SAIDA, PIN_SENSOR_PORTA);

    xTaskCreatePinnedToCore(porta_task, "porta_task", 3072, NULL, 5, NULL, tskNO_AFFINITY);
    return ESP_OK;
}
