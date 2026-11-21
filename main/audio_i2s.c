#include "audio_i2s.h"
#include "pinagem.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s_std.h"
#include "esp_log.h"

static const char *TAG = "AUDIO_I2S";

#define AUDIO_SAMPLE_RATE       22050
#define AUDIO_CHUNK_FRAMES      256
#define AUDIO_PI                3.14159265358979323846f
#define AUDIO_VOLUME_INICIAL    50  // 20% para teste confortavel na bancada

static i2s_chan_handle_t s_tx_handle = NULL;
static QueueHandle_t s_audio_queue = NULL;
static volatile uint8_t s_volume_pct = AUDIO_VOLUME_INICIAL;

// Buffer estereo (L + R) estatico: 256 frames x 2 canais = 1024 bytes
// Enviar o mesmo sinal nos dois canais garante som no MAX98357A em qualquer configuracao do pino SD
static int16_t s_pcm_buf[AUDIO_CHUNK_FRAMES * 2];

static void enviar_silencio(uint32_t duracao_ms) {
    if (s_tx_handle == NULL || duracao_ms == 0) return;

    memset(s_pcm_buf, 0, sizeof(s_pcm_buf));
    uint32_t frames_restantes = (AUDIO_SAMPLE_RATE * duracao_ms) / 1000;

    while (frames_restantes > 0) {
        uint32_t bloco = (frames_restantes > AUDIO_CHUNK_FRAMES) ? AUDIO_CHUNK_FRAMES : frames_restantes;
        size_t escritos = 0;
        i2s_channel_write(s_tx_handle, s_pcm_buf, bloco * 2 * sizeof(int16_t), &escritos, 1000);
        frames_restantes -= bloco;
    }
}

static void tocar_tom(uint32_t freq_hz, uint32_t duracao_ms) {
    if (s_tx_handle == NULL || duracao_ms == 0) return;

    if (freq_hz == 0 || s_volume_pct == 0) {
        enviar_silencio(duracao_ms);
        return;
    }

    uint32_t total_frames = (AUDIO_SAMPLE_RATE * duracao_ms) / 1000;
    uint32_t fade_frames = (AUDIO_SAMPLE_RATE * 5) / 1000; // 5 ms de fade-in/fade-out anti-estalo
    if (fade_frames > total_frames / 2) {
        fade_frames = total_frames / 2;
    }

    float amp = (32767.0f * (float)s_volume_pct) / 100.0f;
    float fase = 0.0f;
    float inc_fase = (2.0f * AUDIO_PI * (float)freq_hz) / (float)AUDIO_SAMPLE_RATE;

    uint32_t frame_atual = 0;
    while (frame_atual < total_frames) {
        uint32_t bloco = total_frames - frame_atual;
        if (bloco > AUDIO_CHUNK_FRAMES) bloco = AUDIO_CHUNK_FRAMES;

        for (uint32_t i = 0; i < bloco; i++) {
            uint32_t idx = frame_atual + i;
            float env = 1.0f;

            if (fade_frames > 0) {
                if (idx < fade_frames) {
                    env = (float)idx / (float)fade_frames;
                } else if (idx > (total_frames - fade_frames)) {
                    env = (float)(total_frames - idx) / (float)fade_frames;
                }
            }

            int16_t amostra = (int16_t)(sinf(fase) * amp * env);
            s_pcm_buf[i * 2]     = amostra; // Canal Esquerdo (L)
            s_pcm_buf[i * 2 + 1] = amostra; // Canal Direito (R)

            fase += inc_fase;
            if (fase >= 2.0f * AUDIO_PI) {
                fase -= 2.0f * AUDIO_PI;
            }
        }

        size_t escritos = 0;
        i2s_channel_write(s_tx_handle, s_pcm_buf, bloco * 2 * sizeof(int16_t), &escritos, 1000);
        frame_atual += bloco;
    }
}

static void executar_efeito(audio_efeito_t efeito) {
    switch (efeito) {
        case AUDIO_SOM_BOOT:
            tocar_tom(523, 90);   // Do5
            enviar_silencio(20);
            tocar_tom(659, 90);   // Mi5
            enviar_silencio(20);
            tocar_tom(784, 140);  // Sol5
            break;

        case AUDIO_SOM_TECLA:
            tocar_tom(1400, 35);
            break;

        case AUDIO_SOM_ACESSO_OK:
            tocar_tom(659, 90);   // Mi5
            enviar_silencio(25);
            tocar_tom(988, 180);  // Si5
            break;

        case AUDIO_SOM_ACESSO_NEGADO:
            tocar_tom(392, 140);  // Sol4
            enviar_silencio(30);
            tocar_tom(262, 240);  // Do4
            break;

        case AUDIO_SOM_CAMPAINHA:
            tocar_tom(784, 180);  // Sol5
            enviar_silencio(30);
            tocar_tom(659, 320);  // Mi5
            break;

        case AUDIO_SOM_ALARME:
            for (int i = 0; i < 3; i++) {
                tocar_tom(1760, 120);
                tocar_tom(1175, 120);
            }
            break;

        default:
            break;
    }

    // Garante drenagem limpa do DMA para silencio total apos o som
    enviar_silencio(50);
}

static void audio_task(void *arg) {
    (void)arg;
    audio_efeito_t efeito;

    while (1) {
        if (xQueueReceive(s_audio_queue, &efeito, portMAX_DELAY) == pdTRUE) {
            executar_efeito(efeito);
        }
    }
}

void audio_i2s_definir_volume(uint8_t volume_pct) {
    if (volume_pct > 100) volume_pct = 100;
    s_volume_pct = volume_pct;
}

void audio_i2s_tocar(audio_efeito_t efeito) {
    if (s_audio_queue == NULL) return;
    // Nao bloqueia se a fila encher (ex.: teclas muito rapidas)
    xQueueSend(s_audio_queue, &efeito, 0);
}

esp_err_t audio_i2s_iniciar(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Limpa automaticamente buffers DMA ao terminar (evita repeticao/ruido)

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx_handle, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao criar canal I2S: %s", esp_err_to_name(err));
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCLK,
            .ws   = PIN_I2S_LRC,
            .dout = PIN_I2S_DIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(s_tx_handle, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao configurar modo STD I2S: %s", esp_err_to_name(err));
        return err;
    }

    err = i2s_channel_enable(s_tx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao habilitar canal I2S: %s", esp_err_to_name(err));
        return err;
    }

    // Zera os buffers iniciais imediatamente para calar qualquer ruido do amplificador
    enviar_silencio(40);

    s_audio_queue = xQueueCreate(8, sizeof(audio_efeito_t));
    if (s_audio_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    xTaskCreatePinnedToCore(audio_task, "audio_task", 3072, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "Audio I2S (MAX98357A) iniciado (LRC=GPIO%d, BCLK=GPIO%d, DIN=GPIO%d, Vol=%u%%).",
             PIN_I2S_LRC, PIN_I2S_BCLK, PIN_I2S_DIN, s_volume_pct);
    return ESP_OK;
}
