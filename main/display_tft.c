#include "display_tft.h"
#include "pinagem.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "TFT_ST7796";

// 20 MHz e estavel em protoboard; na placa final pode subir para 40 MHz
#define TFT_SPI_CLOCK_HZ     (20 * 1000 * 1000)
#define TFT_LINHAS_DMA       20
#define TFT_BUF_PIXELS       (TFT_LARGURA * TFT_LINHAS_DMA)

static spi_device_handle_t s_spi_tft = NULL;
static uint16_t *s_dma_buf = NULL;

static inline uint16_t swap16(uint16_t v) {
    return (uint16_t)((v >> 8) | (v << 8));
}

static void tft_enviar_cmd(uint8_t cmd) {
    gpio_set_level(PIN_TFT_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_polling_transmit(s_spi_tft, &t);
}

static void tft_enviar_dados(const uint8_t *dados, size_t len) {
    if (len == 0) return;
    gpio_set_level(PIN_TFT_DC, 1);
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = dados,
    };
    spi_device_polling_transmit(s_spi_tft, &t);
}

static void tft_enviar_dado_u8(uint8_t val) {
    tft_enviar_dados(&val, 1);
}

static void tft_definir_janela(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t col[4] = {
        (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF),
        (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF)
    };
    uint8_t row[4] = {
        (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF),
        (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF)
    };

    tft_enviar_cmd(0x2A); // CASET
    tft_enviar_dados(col, 4);

    tft_enviar_cmd(0x2B); // RASET
    tft_enviar_dados(row, 4);

    tft_enviar_cmd(0x2C); // RAMWR
}

static void st7796s_inicializar_registradores(void) {
    // Reset fisico
    gpio_set_level(PIN_TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    tft_enviar_cmd(0x01); // Software Reset
    vTaskDelay(pdMS_TO_TICKS(120));

    tft_enviar_cmd(0x11); // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));

    // Destrava comandos estendidos do ST7796S
    tft_enviar_cmd(0xF0); tft_enviar_dado_u8(0xC3);
    tft_enviar_cmd(0xF0); tft_enviar_dado_u8(0x96);

    // Orientacao retrato (320x480) + ordem BGR
    tft_enviar_cmd(0x36); tft_enviar_dado_u8(0x88);

    // Formato de cor: 16-bit RGB565
    tft_enviar_cmd(0x3A); tft_enviar_dado_u8(0x55);

    // Controle de inversao e saida do painel
    tft_enviar_cmd(0xB4); tft_enviar_dado_u8(0x01);

    const uint8_t b6_data[] = { 0x80, 0x02, 0x3B };
    tft_enviar_cmd(0xB6); tft_enviar_dados(b6_data, sizeof(b6_data));

    const uint8_t e8_data[] = { 0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33 };
    tft_enviar_cmd(0xE8); tft_enviar_dados(e8_data, sizeof(e8_data));

    tft_enviar_cmd(0xC1); tft_enviar_dado_u8(0x06);
    tft_enviar_cmd(0xC2); tft_enviar_dado_u8(0xA7);
    tft_enviar_cmd(0xC5); tft_enviar_dado_u8(0x18);

    const uint8_t gamma_pos[] = {
        0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F,
        0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B
    };
    tft_enviar_cmd(0xE0); tft_enviar_dados(gamma_pos, sizeof(gamma_pos));

    const uint8_t gamma_neg[] = {
        0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B,
        0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B
    };
    tft_enviar_cmd(0xE1); tft_enviar_dados(gamma_neg, sizeof(gamma_neg));

    // Trava comandos estendidos
    tft_enviar_cmd(0xF0); tft_enviar_dado_u8(0x3C);
    tft_enviar_cmd(0xF0); tft_enviar_dado_u8(0x69);

    // Painel IPS normalmente requer INVON (0x21). Se as cores ficarem negativas, troque para 0x20.
    tft_enviar_cmd(0x20);

    tft_enviar_cmd(0x29); // Display ON
    vTaskDelay(pdMS_TO_TICKS(50));
}

void display_tft_backlight(bool ligado) {
    gpio_set_level(PIN_TFT_BL, ligado ? 1 : 0);
}

void display_tft_preencher_retangulo(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t cor_rgb565) {
    if (s_spi_tft == NULL || s_dma_buf == NULL || w == 0 || h == 0) return;
    if (x >= TFT_LARGURA || y >= TFT_ALTURA) return;
    if (x + w > TFT_LARGURA) w = TFT_LARGURA - x;
    if (y + h > TFT_ALTURA) h = TFT_ALTURA - y;

    uint16_t cor_be = swap16(cor_rgb565);
    size_t total_pixels = (size_t)w * (size_t)h;
    size_t chunk_pixels = (total_pixels < TFT_BUF_PIXELS) ? total_pixels : TFT_BUF_PIXELS;

    for (size_t i = 0; i < chunk_pixels; i++) {
        s_dma_buf[i] = cor_be;
    }

    tft_definir_janela(x, y, x + w - 1, y + h - 1);
    gpio_set_level(PIN_TFT_DC, 1);

    size_t restantes = total_pixels;
    while (restantes > 0) {
        size_t enviar = (restantes < chunk_pixels) ? restantes : chunk_pixels;
        spi_transaction_t t = {
            .length = enviar * 16,
            .tx_buffer = s_dma_buf,
        };
        spi_device_polling_transmit(s_spi_tft, &t);
        restantes -= enviar;
    }
}

void display_tft_desenhar_rgb565(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pixels, bool inverter_bytes) {
    if (s_spi_tft == NULL || s_dma_buf == NULL || pixels == NULL || w == 0 || h == 0) return;
    if (x >= TFT_LARGURA || y >= TFT_ALTURA) return;

    tft_definir_janela(x, y, x + w - 1, y + h - 1);
    gpio_set_level(PIN_TFT_DC, 1);

    size_t total_pixels = (size_t)w * (size_t)h;
    size_t offset = 0;

    while (offset < total_pixels) {
        size_t enviar = total_pixels - offset;
        if (enviar > TFT_BUF_PIXELS) enviar = TFT_BUF_PIXELS;

        if (inverter_bytes) {
            for (size_t i = 0; i < enviar; i++) {
                s_dma_buf[i] = swap16(pixels[offset + i]);
            }
        } else {
            memcpy(s_dma_buf, &pixels[offset], enviar * sizeof(uint16_t));
        }

        spi_transaction_t t = {
            .length = enviar * 16,
            .tx_buffer = s_dma_buf,
        };
        spi_device_polling_transmit(s_spi_tft, &t);
        offset += enviar;
    }
}

static void tft_teste_visual_task(void *arg) {
    // 1. Teste RGB de cores + medicao real de FPS para o Olho Magico
    int64_t t0 = esp_timer_get_time();
    display_tft_preencher_retangulo(0, 0, TFT_LARGURA, TFT_ALTURA, COR_VERMELHO);
    int64_t t1 = esp_timer_get_time();

    float ms_tela_cheia = (t1 - t0) / 1000.0f;
    float fps_tela_cheia = 1000.0f / ms_tela_cheia;
    float fps_olho_magico_qvga = fps_tela_cheia * 2.0f; // 320x240 e metade da tela 320x480

    ESP_LOGI(TAG, "Benchmark SPI @ %d MHz -> Tela cheia (320x480): %.1f ms (%.1f FPS) | Janela Olho Magico (320x240): ~%.1f FPS",
             TFT_SPI_CLOCK_HZ / 1000000, ms_tela_cheia, fps_tela_cheia, fps_olho_magico_qvga);

    vTaskDelay(pdMS_TO_TICKS(700));
    display_tft_preencher_retangulo(0, 0, TFT_LARGURA, TFT_ALTURA, COR_VERDE);
    vTaskDelay(pdMS_TO_TICKS(700));
    display_tft_preencher_retangulo(0, 0, TFT_LARGURA, TFT_ALTURA, COR_AZUL);
    vTaskDelay(pdMS_TO_TICKS(700));

    // 2. Layout de simulacao do Olho Magico Digital (320x240 em cima + painel de status embaixo)
    display_tft_preencher_retangulo(0, 0, TFT_LARGURA, TFT_ALTURA, COR_PRETO);

    // Barra superior de titulo (altura 40 px)
    display_tft_preencher_retangulo(0, 0, TFT_LARGURA, 40, COR_AZUL);

    // Janela do Olho Magico 320x240 (y = 50 ate 290)
    display_tft_preencher_retangulo(0, 50, 320, 240, COR_CINZA_ESC);
    // Moldura verde simulando "Camera Ativa"
    display_tft_preencher_retangulo(0, 50, 320, 4, COR_VERDE);
    display_tft_preencher_retangulo(0, 286, 320, 4, COR_VERDE);
    display_tft_preencher_retangulo(0, 50, 4, 240, COR_VERDE);
    display_tft_preencher_retangulo(316, 50, 4, 240, COR_VERDE);

    // Cards inferiores de status (Porta / Presenca / Alarme)
    display_tft_preencher_retangulo(15, 310, 290, 65, COR_VERDE);
    display_tft_preencher_retangulo(15, 390, 138, 70, COR_CIANO);
    display_tft_preencher_retangulo(167, 390, 138, 70, COR_AMARELO);

    ESP_LOGI(TAG, "Display ST7796S validado com layout do Olho Magico!");
    vTaskDelete(NULL);
}

esp_err_t display_tft_iniciar(void) {
    // Configura GPIOs de controle (DC, RST, BL)
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << PIN_TFT_DC) | (1ULL << PIN_TFT_RST) | (1ULL << PIN_TFT_BL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_cfg);
    if (err != ESP_OK) return err;

    display_tft_backlight(false);

    // Aloca buffer DMA para transferencias rapidas
    s_dma_buf = heap_caps_malloc(TFT_BUF_PIXELS * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (s_dma_buf == NULL) {
        ESP_LOGE(TAG, "Falha ao alocar buffer DMA do TFT!");
        return ESP_ERR_NO_MEM;
    }

    // Inicializa barramento SPI2 (somente escrita: MISO = -1)
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_TFT_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_TFT_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_BUF_PIXELS * sizeof(uint16_t) + 8,
    };

    err = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar SPI2: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = TFT_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = PIN_TFT_CS,
        .queue_size = 7,
    };

    err = spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_spi_tft);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar ST7796S no SPI2: %s", esp_err_to_name(err));
        return err;
    }

    st7796s_inicializar_registradores();
    display_tft_backlight(true);

    ESP_LOGI(TAG, "ST7796S iniciado (SCK=GPIO%d, MOSI=GPIO%d, CS=GPIO%d, DC=GPIO%d, RST=GPIO%d, BL=GPIO%d)",
             PIN_TFT_SCK, PIN_TFT_MOSI, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST, PIN_TFT_BL);

    xTaskCreatePinnedToCore(tft_teste_visual_task, "tft_teste_task", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    return ESP_OK;
}
// FIM DO ARQUIVO
