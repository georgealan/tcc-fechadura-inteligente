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

#define AUDIO_SAMPLE_RATE       24000
#define AUDIO_CHUNK_FRAMES      256
#define AUDIO_PI                3.14159265358979323846f
#define AUDIO_VOLUME_INICIAL    50
// Troque para 0 antes da apresentacao no SENAI
#define SILENCIAR_ALARME_BANCADA 1

typedef enum {
    AUDIO_CMD_EFEITO = 0,
    AUDIO_CMD_WAV
} audio_cmd_tipo_t;

typedef struct {
    audio_cmd_tipo_t tipo;
    audio_efeito_t efeito;
    const uint8_t *wav_dados;
    size_t wav_tamanho;
} audio_cmd_t;

static i2s_chan_handle_t s_tx_handle = NULL;
static QueueHandle_t s_audio_queue = NULL;
static volatile uint8_t s_volume_pct = AUDIO_VOLUME_INICIAL;
static uint32_t s_taxa_atual = AUDIO_SAMPLE_RATE;

// Buffer estereo (L + R) estatico: 256 frames x 2 canais = 1024 bytes
static int16_t s_pcm_buf[AUDIO_CHUNK_FRAMES * 2];

static inline uint16_t ler_u16_le(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t ler_u32_le(const uint8_t *p) {
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static esp_err_t ajustar_sample_rate(uint32_t nova_taxa) {
    if (s_tx_handle == NULL || nova_taxa == 0) return ESP_ERR_INVALID_ARG;
    if (nova_taxa == s_taxa_atual) return ESP_OK;

    esp_err_t err = i2s_channel_disable(s_tx_handle);
    if (err != ESP_OK) return err;

    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(nova_taxa);
    err = i2s_channel_reconfig_std_clock(s_tx_handle, &clk_cfg);
    if (err == ESP_OK) {
        s_taxa_atual = nova_taxa;
    } else {
        ESP_LOGE(TAG, "Falha ao reconfigurar clock I2S para %lu Hz: %s",
                 (unsigned long)nova_taxa, esp_err_to_name(err));
    }

    esp_err_t en_err = i2s_channel_enable(s_tx_handle);
    return (err != ESP_OK) ? err : en_err;
}

static void enviar_silencio(uint32_t duracao_ms) {
    if (s_tx_handle == NULL || duracao_ms == 0) return;

    memset(s_pcm_buf, 0, sizeof(s_pcm_buf));
    uint32_t frames_restantes = (s_taxa_atual * duracao_ms) / 1000;

    while (frames_restantes > 0) {
        uint32_t bloco = (frames_restantes > AUDIO_CHUNK_FRAMES) ? AUDIO_CHUNK_FRAMES : frames_restantes;
        size_t escritos = 0;
        i2s_channel_write(s_tx_handle, s_pcm_buf, bloco * 2 * sizeof(int16_t), &escritos, 1000);
        frames_restantes -= bloco;
    }
}

static void tocar_tom(uint32_t freq_hz, uint32_t duracao_ms) {
    if (s_tx_handle == NULL || duracao_ms == 0) return;

    ajustar_sample_rate(AUDIO_SAMPLE_RATE);

    if (freq_hz == 0 || s_volume_pct == 0) {
        enviar_silencio(duracao_ms);
        return;
    }

    uint32_t total_frames = (s_taxa_atual * duracao_ms) / 1000;
    uint32_t fade_frames = (s_taxa_atual * 5) / 1000;
    if (fade_frames > total_frames / 2) {
        fade_frames = total_frames / 2;
    }

    float amp = (32767.0f * (float)s_volume_pct) / 100.0f;
    float fase = 0.0f;
    float inc_fase = (2.0f * AUDIO_PI * (float)freq_hz) / (float)s_taxa_atual;

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
            s_pcm_buf[i * 2]     = amostra;
            s_pcm_buf[i * 2 + 1] = amostra;

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
            tocar_tom(523, 90);
            enviar_silencio(20);
            tocar_tom(659, 90);
            enviar_silencio(20);
            tocar_tom(784, 140);
            break;

        case AUDIO_SOM_TECLA:
            tocar_tom(1400, 35);
            break;

        case AUDIO_SOM_ACESSO_OK:
            tocar_tom(659, 90);
            enviar_silencio(25);
            tocar_tom(988, 180);
            break;

        case AUDIO_SOM_ACESSO_NEGADO:
            tocar_tom(392, 140);
            enviar_silencio(30);
            tocar_tom(262, 240);
            break;

        case AUDIO_SOM_CAMPAINHA:
            tocar_tom(784, 180);
            enviar_silencio(30);
            tocar_tom(659, 320);
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

    enviar_silencio(50);
}

static void executar_wav(const uint8_t *dados, size_t tamanho) {
    if (s_tx_handle == NULL || dados == NULL || tamanho < 44) {
        ESP_LOGE(TAG, "WAV invalido ou muito pequeno (%u bytes).", (unsigned)tamanho);
        return;
    }

    if (memcmp(dados, "RIFF", 4) != 0 || memcmp(dados + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Arquivo nao possui cabecalho RIFF/WAVE valido.");
        return;
    }

    uint16_t audio_format = 0;
    uint16_t num_canais = 0;
    uint32_t sample_rate = 0;
    uint16_t bits_por_amostra = 0;
    const uint8_t *pcm_ptr = NULL;
    size_t pcm_bytes = 0;

    // Percorre os chunks RIFF (suporta metadados como LIST/JUNK antes de 'data')
    size_t offset = 12;
    while (offset + 8 <= tamanho) {
        const uint8_t *chunk_id = dados + offset;
        uint32_t chunk_size = ler_u32_le(dados + offset + 4);
        size_t payload_offset = offset + 8;

        if (payload_offset > tamanho) break;
        size_t disponivel = tamanho - payload_offset;
        size_t tamanho_real = (chunk_size <= disponivel) ? chunk_size : disponivel;

        if (memcmp(chunk_id, "fmt ", 4) == 0 && tamanho_real >= 16) {
            audio_format     = ler_u16_le(dados + payload_offset + 0);
            num_canais       = ler_u16_le(dados + payload_offset + 2);
            sample_rate      = ler_u32_le(dados + payload_offset + 4);
            bits_por_amostra = ler_u16_le(dados + payload_offset + 14);
        } else if (memcmp(chunk_id, "data", 4) == 0) {
            pcm_ptr = dados + payload_offset;
            pcm_bytes = tamanho_real;
            break;
        }

        // Chunks RIFF tem alinhamento par (padding de 1 byte se tamanho for impar)
        size_t avanco = 8 + (size_t)chunk_size + (chunk_size & 1U);
        if (avanco < 8 || offset + avanco > tamanho) break;
        offset += avanco;
    }

    if (pcm_ptr == NULL || pcm_bytes == 0) {
        ESP_LOGE(TAG, "Chunk 'data' nao encontrado no arquivo WAV.");
        return;
    }

    if (audio_format != 1 || bits_por_amostra != 16 || (num_canais != 1 && num_canais != 2)) {
        ESP_LOGE(TAG, "Formato WAV nao suportado (fmt=%u, canais=%u, bits=%u). Converta para PCM 16-bit.",
                 audio_format, num_canais, bits_por_amostra);
        return;
    }

    ESP_LOGI(TAG, "Tocando WAV: %lu Hz | %u canal(is) | %u-bit | %u bytes PCM",
             (unsigned long)sample_rate, num_canais, bits_por_amostra, (unsigned)pcm_bytes);

    if (ajustar_sample_rate(sample_rate) != ESP_OK) {
        return;
    }

    size_t bytes_por_frame = (size_t)num_canais * sizeof(int16_t);
    size_t total_frames = pcm_bytes / bytes_por_frame;
    size_t frame_atual = 0;

    while (frame_atual < total_frames) {
        size_t bloco = total_frames - frame_atual;
        if (bloco > AUDIO_CHUNK_FRAMES) bloco = AUDIO_CHUNK_FRAMES;

        int32_t vol = (int32_t)s_volume_pct;
        const uint8_t *p = pcm_ptr + (frame_atual * bytes_por_frame);

        if (num_canais == 1) {
            for (size_t i = 0; i < bloco; i++) {
                int16_t raw = (int16_t)ler_u16_le(p + (i * 2));
                int16_t amostra = (int16_t)(((int32_t)raw * vol) / 100);
                s_pcm_buf[i * 2]     = amostra;
                s_pcm_buf[i * 2 + 1] = amostra;
            }
        } else {
            for (size_t i = 0; i < bloco; i++) {
                int16_t raw_l = (int16_t)ler_u16_le(p + (i * 4));
                int16_t raw_r = (int16_t)ler_u16_le(p + (i * 4) + 2);
                s_pcm_buf[i * 2]     = (int16_t)(((int32_t)raw_l * vol) / 100);
                s_pcm_buf[i * 2 + 1] = (int16_t)(((int32_t)raw_r * vol) / 100);
            }
        }

        size_t escritos = 0;
        i2s_channel_write(s_tx_handle, s_pcm_buf, bloco * 2 * sizeof(int16_t), &escritos, 1000);
        frame_atual += bloco;
    }

    enviar_silencio(50);
}

static void audio_task(void *arg) {
    (void)arg;
    audio_cmd_t cmd;

    while (1) {
        if (xQueueReceive(s_audio_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            if (cmd.tipo == AUDIO_CMD_EFEITO) {
                executar_efeito(cmd.efeito);
            } else if (cmd.tipo == AUDIO_CMD_WAV) {
                executar_wav(cmd.wav_dados, cmd.wav_tamanho);
            }
        }
    }
}

void audio_i2s_definir_volume(uint8_t volume_pct) {
    if (volume_pct > 100) volume_pct = 100;
    s_volume_pct = volume_pct;
}

void audio_i2s_tocar(audio_efeito_t efeito) {
    #if SILENCIAR_ALARME_BANCADA
        if (efeito == AUDIO_SOM_ALARME) return; // confira o nome exato do enum no seu audio_i2s.h
    #endif

    if (s_audio_queue == NULL) return;
    audio_cmd_t cmd = {
        .tipo = AUDIO_CMD_EFEITO,
        .efeito = efeito,
        .wav_dados = NULL,
        .wav_tamanho = 0,
    };
    xQueueSend(s_audio_queue, &cmd, 0);
}

void audio_i2s_tocar_wav(const uint8_t *dados, size_t tamanho) {
    if (s_audio_queue == NULL || dados == NULL || tamanho == 0) return;
    audio_cmd_t cmd = {
        .tipo = AUDIO_CMD_WAV,
        .efeito = AUDIO_SOM_BOOT,
        .wav_dados = dados,
        .wav_tamanho = tamanho,
    };
    xQueueSend(s_audio_queue, &cmd, 0);
}

esp_err_t audio_i2s_iniciar(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx_handle, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao criar canal I2S: %s", esp_err_to_name(err));
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode      = I2S_SLOT_MODE_STEREO,
            .slot_mask      = I2S_STD_SLOT_BOTH,
            .ws_width       = I2S_SLOT_BIT_WIDTH_32BIT,
            .ws_pol         = false,
            .bit_shift      = true,
            .left_align     = true,
            .big_endian     = false,
            .bit_order_lsb  = false,
        },
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

    s_taxa_atual = AUDIO_SAMPLE_RATE;
    enviar_silencio(40);

    s_audio_queue = xQueueCreate(8, sizeof(audio_cmd_t));
    if (s_audio_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    xTaskCreatePinnedToCore(audio_task, "audio_task", 4096, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "Audio I2S (MAX98357A) iniciado (LRC=GPIO%d, BCLK=GPIO%d, DIN=GPIO%d, Vol=%u%%).",
             PIN_I2S_LRC, PIN_I2S_BCLK, PIN_I2S_DIN, s_volume_pct);
    return ESP_OK;
}
