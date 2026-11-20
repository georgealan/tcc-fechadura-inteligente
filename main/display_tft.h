#ifndef DISPLAY_TFT_H
#define DISPLAY_TFT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define TFT_LARGURA  320
#define TFT_ALTURA   480

// Cores padrao em RGB565
#define COR_PRETO     0x0000
#define COR_BRANCO    0xFFFF
#define COR_VERMELHO  0xF800
#define COR_VERDE     0x07E0
#define COR_AZUL      0x001F
#define COR_AMARELO   0xFFE0
#define COR_CIANO     0x07FF
#define COR_CINZA_ESC 0x2104

// Inicializa SPI2 + ST7796S e desenha o layout base do Olho Magico
esp_err_t display_tft_iniciar(void);

// Retorna true quando o layout inicial ja foi desenhado e o SPI esta liberado para o video
bool display_tft_pronto(void);

// Preenche um retangulo com cor solida (RGB565) - protegido por Mutex
void display_tft_preencher_retangulo(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t cor_rgb565);

// Envia um bloco de pixels RGB565 para a tela - protegido por Mutex
void display_tft_desenhar_rgb565(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pixels, bool inverter_bytes);

// Liga ou desliga o backlight (GPIO 10)
void display_tft_backlight(bool ligado);

#endif // DISPLAY_TFT_H
// FIM DO ARQUIVO
