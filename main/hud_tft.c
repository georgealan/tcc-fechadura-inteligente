#include "hud_tft.h"
#include "display_tft.h"
#include "fechadura_fsm.h"
#include "sensor_tof.h"
#include "wifi_manager.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "facial_client.h"

static const char *TAG = "HUD_TFT";

// 0 = Tela sempre acesa (recomendado para bancada e apresentacao do TCC)
// 1 = Standby automatico do backlight apos inatividade em repouso
#define HUD_STANDBY_ATIVO           0
#define HUD_STANDBY_TIMEOUT_S       30

// Paleta adicional RGB565
#define COR_AZUL_MARINHO            0x08E5
#define COR_CARD_FUNDO              0x18E3
#define COR_CARD_BORDA              0x39E7
#define COR_VERDE_ESCURO            0x03E0
#define COR_LARANJA                 0xFC00
#define COR_LARANJA_ESC             0xA280
#define COR_CIANO_ESC               0x0390
#define COR_CINZA_CLARO             0xBDF7
#define COR_VERMELHO_ESC            0x9000

// Buffer estatico para desenhar 1 linha de texto ate escala 2 (320 x 16 px = 10 KB)
#define HUD_MAX_TEXTO_H             16
static uint16_t s_linha_buf[TFT_LARGURA * HUD_MAX_TEXTO_H];

static volatile int64_t s_acordar_manual_us = 0;

static inline uint16_t rgb565_be(uint16_t cor) {
    return (uint16_t)((cor >> 8) | (cor << 8));
}

// Fonte bitmap 5x7 (5 colunas, bit 0 = topo, bit 6 = base)
static void obter_glifo_5x7(char c, uint8_t col[5]) {
    if (c >= 'a' && c <= 'z') {
        c = (char)(c - 'a' + 'A');
    }

    static const uint8_t vazio[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    const uint8_t *g = vazio;

    switch (c) {
        case '!': { static const uint8_t d[5] = {0x00, 0x00, 0x5F, 0x00, 0x00}; g = d; break; }
        case '%': { static const uint8_t d[5] = {0x23, 0x13, 0x08, 0x64, 0x62}; g = d; break; }
        case '(': { static const uint8_t d[5] = {0x00, 0x1C, 0x22, 0x41, 0x00}; g = d; break; }
        case ')': { static const uint8_t d[5] = {0x00, 0x41, 0x22, 0x1C, 0x00}; g = d; break; }
        case '-': { static const uint8_t d[5] = {0x08, 0x08, 0x08, 0x08, 0x08}; g = d; break; }
        case '.': { static const uint8_t d[5] = {0x00, 0x60, 0x60, 0x00, 0x00}; g = d; break; }
        case '/': { static const uint8_t d[5] = {0x20, 0x10, 0x08, 0x04, 0x02}; g = d; break; }
        case ':': { static const uint8_t d[5] = {0x00, 0x00, 0x14, 0x00, 0x00}; g = d; break; }
        case '>': { static const uint8_t d[5] = {0x00, 0x41, 0x22, 0x14, 0x08}; g = d; break; }
        case '0': { static const uint8_t d[5] = {0x3E, 0x51, 0x49, 0x45, 0x3E}; g = d; break; }
        case '1': { static const uint8_t d[5] = {0x00, 0x42, 0x7F, 0x40, 0x00}; g = d; break; }
        case '2': { static const uint8_t d[5] = {0x42, 0x61, 0x51, 0x49, 0x46}; g = d; break; }
        case '3': { static const uint8_t d[5] = {0x22, 0x41, 0x49, 0x49, 0x36}; g = d; break; }
        case '4': { static const uint8_t d[5] = {0x18, 0x14, 0x12, 0x7F, 0x10}; g = d; break; }
        case '5': { static const uint8_t d[5] = {0x27, 0x45, 0x45, 0x45, 0x39}; g = d; break; }
        case '6': { static const uint8_t d[5] = {0x3C, 0x4A, 0x49, 0x49, 0x30}; g = d; break; }
        case '7': { static const uint8_t d[5] = {0x01, 0x71, 0x09, 0x05, 0x03}; g = d; break; }
        case '8': { static const uint8_t d[5] = {0x36, 0x49, 0x49, 0x49, 0x36}; g = d; break; }
        case '9': { static const uint8_t d[5] = {0x06, 0x49, 0x49, 0x29, 0x1E}; g = d; break; }
        case 'A': { static const uint8_t d[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E}; g = d; break; }
        case 'B': { static const uint8_t d[5] = {0x7F, 0x49, 0x49, 0x49, 0x36}; g = d; break; }
        case 'C': { static const uint8_t d[5] = {0x3E, 0x41, 0x41, 0x41, 0x22}; g = d; break; }
        case 'D': { static const uint8_t d[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C}; g = d; break; }
        case 'E': { static const uint8_t d[5] = {0x7F, 0x49, 0x49, 0x49, 0x41}; g = d; break; }
        case 'F': { static const uint8_t d[5] = {0x7F, 0x09, 0x09, 0x09, 0x01}; g = d; break; }
        case 'G': { static const uint8_t d[5] = {0x3E, 0x41, 0x49, 0x49, 0x7A}; g = d; break; }
        case 'H': { static const uint8_t d[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F}; g = d; break; }
        case 'I': { static const uint8_t d[5] = {0x00, 0x41, 0x7F, 0x41, 0x00}; g = d; break; }
        case 'J': { static const uint8_t d[5] = {0x20, 0x40, 0x41, 0x3F, 0x01}; g = d; break; }
        case 'K': { static const uint8_t d[5] = {0x7F, 0x08, 0x14, 0x22, 0x41}; g = d; break; }
        case 'L': { static const uint8_t d[5] = {0x7F, 0x40, 0x40, 0x40, 0x40}; g = d; break; }
        case 'M': { static const uint8_t d[5] = {0x7F, 0x02, 0x0C, 0x02, 0x7F}; g = d; break; }
        case 'N': { static const uint8_t d[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F}; g = d; break; }
        case 'O': { static const uint8_t d[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E}; g = d; break; }
        case 'P': { static const uint8_t d[5] = {0x7F, 0x09, 0x09, 0x09, 0x06}; g = d; break; }
        case 'Q': { static const uint8_t d[5] = {0x3E, 0x41, 0x51, 0x21, 0x5E}; g = d; break; }
        case 'R': { static const uint8_t d[5] = {0x7F, 0x09, 0x19, 0x29, 0x46}; g = d; break; }
        case 'S': { static const uint8_t d[5] = {0x46, 0x49, 0x49, 0x49, 0x31}; g = d; break; }
        case 'T': { static const uint8_t d[5] = {0x01, 0x01, 0x7F, 0x01, 0x01}; g = d; break; }
        case 'U': { static const uint8_t d[5] = {0x3F, 0x40, 0x40, 0x40, 0x3F}; g = d; break; }
        case 'V': { static const uint8_t d[5] = {0x1F, 0x20, 0x40, 0x20, 0x1F}; g = d; break; }
        case 'W': { static const uint8_t d[5] = {0x3F, 0x40, 0x38, 0x40, 0x3F}; g = d; break; }
        case 'X': { static const uint8_t d[5] = {0x63, 0x14, 0x08, 0x14, 0x63}; g = d; break; }
        case 'Y': { static const uint8_t d[5] = {0x07, 0x08, 0x70, 0x08, 0x07}; g = d; break; }
        case 'Z': { static const uint8_t d[5] = {0x61, 0x51, 0x49, 0x45, 0x43}; g = d; break; }
        default:  g = vazio; break;
    }
    memcpy(col, g, 5);
}

// Renderiza uma faixa de texto centralizada em uma unica transacao SPI (zero flicker)
static void desenhar_texto_centralizado(uint16_t x, uint16_t y, uint16_t w,
                                        uint8_t escala, const char *texto,
                                        uint16_t cor_texto, uint16_t cor_fundo) {
    if (texto == NULL || w == 0 || w > TFT_LARGURA) return;
    if (escala < 1) escala = 1;
    if (escala > 2) escala = 2;

    uint16_t h = (uint16_t)(7 * escala);
    uint16_t bg_be = rgb565_be(cor_fundo);
    uint16_t fg_be = rgb565_be(cor_texto);

    size_t total_px = (size_t)w * (size_t)h;
    for (size_t i = 0; i < total_px; i++) {
        s_linha_buf[i] = bg_be;
    }

    size_t len = strlen(texto);
    uint16_t passo_char = (uint16_t)(6 * escala);
    uint16_t largura_texto = (len > 0) ? (uint16_t)(len * passo_char - escala) : 0;
    int16_t start_x = (largura_texto < w) ? (int16_t)((w - largura_texto) / 2) : 0;

    for (size_t idx = 0; idx < len; idx++) {
        int16_t cx = start_x + (int16_t)(idx * passo_char);
        if (cx >= (int16_t)w) break;

        uint8_t glifo[5];
        obter_glifo_5x7(texto[idx], glifo);

        for (int col = 0; col < 5; col++) {
            uint8_t bits = glifo[col];
            for (int row = 0; row < 7; row++) {
                if (bits & (1U << row)) {
                    for (int dy = 0; dy < escala; dy++) {
                        int py = row * escala + dy;
                        for (int dx = 0; dx < escala; dx++) {
                            int px = cx + col * escala + dx;
                            if (px >= 0 && px < (int)w && py >= 0 && py < (int)h) {
                                s_linha_buf[(size_t)py * w + (size_t)px] = fg_be;
                            }
                        }
                    }
                }
            }
        }
    }

    display_tft_desenhar_rgb565(x, y, w, h, s_linha_buf, false);
}

static void desenhar_borda_card(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t cor) {
    display_tft_preencher_retangulo(x, y, w, 1, cor);
    display_tft_preencher_retangulo(x, y + h - 1, w, 1, cor);
    display_tft_preencher_retangulo(x, y, 1, h, cor);
    display_tft_preencher_retangulo(x + w - 1, y, 1, h, cor);
}

static void desenhar_base_hud(void) {
    // Limpa apenas as regioes de HUD (jamais encosta em y=50..289 do video)
    display_tft_preencher_retangulo(0, 0, TFT_LARGURA, 48, COR_AZUL_MARINHO);
    display_tft_preencher_retangulo(0, 48, TFT_LARGURA, 2, COR_CARD_BORDA);
    display_tft_preencher_retangulo(0, 290, TFT_LARGURA, 2, COR_CARD_BORDA);
    display_tft_preencher_retangulo(0, 292, TFT_LARGURA, TFT_ALTURA - 292, COR_PRETO);

    // Molduras dos cartoes inferiores
    display_tft_preencher_retangulo(8, 364, 148, 52, COR_CARD_FUNDO);
    desenhar_borda_card(8, 364, 148, 52, COR_CARD_BORDA);
    desenhar_texto_centralizado(10, 371, 144, 1, "PORTA (MC-38)", COR_CINZA_CLARO, COR_CARD_FUNDO);

    display_tft_preencher_retangulo(164, 364, 148, 52, COR_CARD_FUNDO);
    desenhar_borda_card(164, 364, 148, 52, COR_CARD_BORDA);
    desenhar_texto_centralizado(166, 371, 144, 1, "ALARME (MPU)", COR_CINZA_CLARO, COR_CARD_FUNDO);

    display_tft_preencher_retangulo(8, 424, 304, 48, COR_CARD_FUNDO);
    desenhar_borda_card(8, 424, 304, 48, COR_CARD_BORDA);
    desenhar_texto_centralizado(10, 430, 300, 1, "PRESENCA EXTERNA (VL53L1X)", COR_CINZA_CLARO, COR_CARD_FUNDO);
}

static void atualizar_topo_e_banner(const fechadura_hud_info_t *info, bool presenca, bool piscar_alarme, facial_fase_t fase_ia) {
    uint16_t cor_topo = COR_AZUL_MARINHO;
    uint16_t cor_txt_topo = COR_BRANCO;
    const char *txt_topo = "FECHADURA INTELIGENTE";

    uint16_t cor_card = COR_AZUL_MARINHO;
    uint16_t cor_txt_l1 = COR_BRANCO;
    uint16_t cor_txt_l2 = COR_CIANO;
    char linha1[32] = {0};
    char linha2[32] = {0};

    switch (info->estado) {
        case ESTADO_REPOUSO_TRANCADA:
            if (fase_ia == FACIAL_FASE_NEGADO) {
                cor_topo = COR_LARANJA_ESC;
                txt_topo = "ACESSO NEGADO";
                cor_card = COR_LARANJA_ESC;
                cor_txt_l1 = COR_BRANCO;
                cor_txt_l2 = COR_AMARELO;
                snprintf(linha1, sizeof(linha1), "ROSTO DESCONHECIDO");
                snprintf(linha2, sizeof(linha2), "USE A SENHA");
                break;
            } else if (fase_ia == FACIAL_FASE_SEM_ROSTO) {
                cor_topo = COR_AZUL_MARINHO;
                txt_topo = "FECHADURA INTELIGENTE";
                cor_card = COR_CARD_FUNDO;
                cor_txt_l1 = COR_AMARELO;
                cor_txt_l2 = COR_BRANCO;
                snprintf(linha1, sizeof(linha1), "ROSTO NAO VISIVEL");
                snprintf(linha2, sizeof(linha2), "APROXIME-SE OU SENHA");
                break;
            }
            if (presenca) {
                cor_topo = COR_CIANO_ESC;
                txt_topo = "VISITANTE NA PORTA";
                cor_card = COR_AMARELO;
                cor_txt_l1 = COR_PRETO;
                cor_txt_l2 = COR_PRETO;
                snprintf(linha1, sizeof(linha1), "VISITANTE NA PORTA!");
                snprintf(linha2, sizeof(linha2), "OLHE O OLHO MAGICO");
            } else {
                cor_topo = COR_AZUL_MARINHO;
                txt_topo = "FECHADURA INTELIGENTE";
                cor_card = COR_AZUL_MARINHO;
                cor_txt_l1 = COR_BRANCO;
                cor_txt_l2 = COR_CINZA_CLARO;
                snprintf(linha1, sizeof(linha1), "PORTA TRANCADA");
                snprintf(linha2, sizeof(linha2), "SISTEMA PROTEGIDO");
            }
            break;

        case ESTADO_DIGITANDO_SENHA:
            cor_topo = COR_CIANO_ESC;
            txt_topo = "ACESSO EXTERNO";
            cor_card = COR_CIANO_ESC;
            cor_txt_l1 = COR_BRANCO;
            cor_txt_l2 = COR_AMARELO;
            snprintf(linha1, sizeof(linha1), "SENHA EM DIGITACAO");
            snprintf(linha2, sizeof(linha2), "TECLADO EXTERNO ATIVO");
            break;

        case ESTADO_DESTRANCADA_AUTORIZADA:
            if (info->usuario_autorizado[0] != '\0') {
                snprintf(linha1, sizeof(linha1), "OLA, %.14s!", info->usuario_autorizado);
            } else {
                snprintf(linha1, sizeof(linha1), "PORTA DESTRANCADA");
            }
            cor_topo = COR_VERDE_ESCURO;
            txt_topo = "ACESSO LIBERADO";
            cor_card = COR_VERDE_ESCURO;
            cor_txt_l1 = COR_BRANCO;
            cor_txt_l2 = COR_AMARELO;
            snprintf(linha1, sizeof(linha1), "PORTA DESTRANCADA");
            snprintf(linha2, sizeof(linha2), "TRANCA EM %lus", (unsigned long)info->tempo_restante_s);
            break;

        case ESTADO_PORTA_ABERTA:
            cor_topo = info->porta_esquecida ? COR_LARANJA_ESC : COR_CIANO_ESC;
            txt_topo = info->porta_esquecida ? "ATENCAO: PORTA ABERTA" : "PORTA ABERTA";
            cor_card = info->porta_esquecida ? COR_LARANJA : COR_AMARELO;
            cor_txt_l1 = COR_PRETO;
            cor_txt_l2 = COR_PRETO;
            snprintf(linha1, sizeof(linha1), "%s", info->porta_esquecida ? "PORTA ESQUECIDA!" : "PORTA ABERTA");
            snprintf(linha2, sizeof(linha2), "%s", info->porta_esquecida ? "ABERTA HA >30s" : "AGUARDANDO FECHAR");
            break;

        case ESTADO_REARMANDO_ALARME:
            cor_topo = COR_AZUL_MARINHO;
            txt_topo = "ESTABILIZANDO";
            cor_card = COR_CARD_FUNDO;
            cor_txt_l1 = COR_CIANO;
            cor_txt_l2 = COR_BRANCO;
            snprintf(linha1, sizeof(linha1), "REARMANDO ALARME");
            snprintf(linha2, sizeof(linha2), "CALIBRANDO %lus", (unsigned long)info->tempo_restante_s);
            break;

        case ESTADO_BLOQUEIO_TENTATIVAS:
            cor_topo = COR_LARANJA_ESC;
            txt_topo = "ALERTA DE SEGURANCA";
            cor_card = COR_LARANJA_ESC;
            cor_txt_l1 = COR_BRANCO;
            cor_txt_l2 = COR_AMARELO;
            snprintf(linha1, sizeof(linha1), "TECLADO BLOQUEADO");
            snprintf(linha2, sizeof(linha2), "LIBERA EM %lus", (unsigned long)info->tempo_restante_s);
            break;

        case ESTADO_ALARME_ARROMBAMENTO:
            cor_topo = piscar_alarme ? COR_VERMELHO : COR_VERMELHO_ESC;
            cor_txt_topo = COR_BRANCO;
            txt_topo = "!!! ALARME ATIVO !!!";
            cor_card = piscar_alarme ? COR_VERMELHO : COR_BRANCO;
            cor_txt_l1 = piscar_alarme ? COR_BRANCO : COR_VERMELHO;
            cor_txt_l2 = piscar_alarme ? COR_AMARELO : COR_PRETO;
            snprintf(linha1, sizeof(linha1), "ARROMBAMENTO!");
            snprintf(linha2, sizeof(linha2), "BOTAO OU SENHA");
            break;

        case ESTADO_RECONHECIMENTO_FACIAL:
            cor_topo = COR_CIANO_ESC;
            txt_topo = "RECONHECIMENTO FACIAL";
            cor_card = COR_CIANO_ESC;
            cor_txt_l1 = COR_BRANCO;
            cor_txt_l2 = COR_AMARELO;
            if (fase_ia == FACIAL_FASE_ENVIANDO) {
                snprintf(linha1, sizeof(linha1), "ANALISANDO ROSTO...");
                snprintf(linha2, sizeof(linha2), "AGUARDE SERVIDOR IA");
            } else {
                snprintf(linha1, sizeof(linha1), "VISITANTE DETECTADO");
                snprintf(linha2, sizeof(linha2), "OLHE PARA A CAMERA");
            }
            break;
    }

    // Barra superior de titulo (y=0..31) — deixa y=32..47 reservado para a faixa do Wi-Fi
    display_tft_preencher_retangulo(0, 0, TFT_LARGURA, 32, cor_topo);
    desenhar_texto_centralizado(4, 9, TFT_LARGURA - 8, 2, txt_topo, cor_txt_topo, cor_topo);

    // Cartao principal (x=8, y=298, w=304, h=58)
    display_tft_preencher_retangulo(8, 298, 304, 58, cor_card);
    desenhar_borda_card(8, 298, 304, 58, COR_BRANCO);
    desenhar_texto_centralizado(12, 308, 296, 2, linha1, cor_txt_l1, cor_card);
    desenhar_texto_centralizado(12, 333, 296, 2, linha2, cor_txt_l2, cor_card);
}

static void atualizar_card_porta(bool porta_fechada, fechadura_estado_t est) {
    const char *txt = porta_fechada ? "FECHADA" : "ABERTA";
    uint16_t cor_txt = porta_fechada ? COR_VERDE : ((est == ESTADO_ALARME_ARROMBAMENTO) ? COR_VERMELHO : COR_AMARELO);
    desenhar_texto_centralizado(10, 388, 144, 2, txt, cor_txt, COR_CARD_FUNDO);
}

static void atualizar_card_alarme(bool armado, fechadura_estado_t est) {
    const char *txt = "PAUSADO";
    uint16_t cor_txt = COR_CIANO;

    if (est == ESTADO_ALARME_ARROMBAMENTO) {
        txt = "VIOLADO!";
        cor_txt = COR_VERMELHO;
    } else if (armado) {
        txt = "ARMADO";
        cor_txt = COR_VERDE;
    }

    desenhar_texto_centralizado(166, 388, 144, 2, txt, cor_txt, COR_CARD_FUNDO);
}

static void atualizar_card_presenca(bool valida, uint16_t dist_mm, bool presenca) {
    char txt[32];
    uint16_t cor_txt = COR_CINZA_CLARO;

    if (presenca && valida) {
        snprintf(txt, sizeof(txt), "PROXIMO: %u CM", (unsigned)(dist_mm / 10));
        cor_txt = COR_AMARELO;
    } else if (valida) {
        snprintf(txt, sizeof(txt), "LIVRE (%u CM)", (unsigned)(dist_mm / 10));
        cor_txt = COR_VERDE;
    } else {
        snprintf(txt, sizeof(txt), "AREA LIVRE");
        cor_txt = COR_CINZA_CLARO;
    }

    desenhar_texto_centralizado(10, 446, 300, 2, txt, cor_txt, COR_CARD_FUNDO);
}

static void atualizar_barra_wifi(wifi_status_t st, const char *ip) {
    char txt[40];
    uint16_t cor_txt = COR_CINZA_CLARO;

    switch (st) {
        case WIFI_STATUS_CONECTADO:
            snprintf(txt, sizeof(txt), "WIFI: OK - %s", ip);
            cor_txt = COR_VERDE;
            break;
        case WIFI_STATUS_CONECTANDO:
            snprintf(txt, sizeof(txt), "WIFI: CONECTANDO...");
            cor_txt = COR_CIANO;
            break;
        case WIFI_STATUS_BUSCANDO:
            snprintf(txt, sizeof(txt), "WIFI: BUSCANDO REDE...");
            cor_txt = COR_AMARELO;
            break;
        default:
            snprintf(txt, sizeof(txt), "WIFI: OFFLINE");
            cor_txt = COR_LARANJA;
            break;
    }

    display_tft_preencher_retangulo(0, 32, TFT_LARGURA, 16, COR_CARD_FUNDO);
    desenhar_texto_centralizado(4, 37, TFT_LARGURA - 8, 1, txt, cor_txt, COR_CARD_FUNDO);
}

void hud_tft_acordar(void) {
    s_acordar_manual_us = esp_timer_get_time();
}

static void hud_tft_task(void *arg) {
    (void)arg;

    // Aguarda o ST7796S concluir a inicializacao
    while (!display_tft_pronto()) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "HUD interno iniciado (standby=%s).", HUD_STANDBY_ATIVO ? "ATIVO" : "SEMPRE ACESO");
    desenhar_base_hud();

    bool primeiro_desenho = true;
    fechadura_hud_info_t ult_info = {0};
    bool ult_presenca = false;
    bool ult_tof_valido = false;
    uint16_t ult_dist_cm = 0xFFFF;
    bool ult_piscar = false;
    bool backlight_ligado = true;
    wifi_status_t ult_wifi_st = (wifi_status_t)(-1);
    char ult_wifi_ip[16] = "";
    facial_fase_t ult_fase_ia = (facial_fase_t)(-1);

    while (1) {
        fechadura_hud_info_t info = {0};
        fechadura_fsm_obter_hud_info(&info);
        facial_fase_t fase_ia = facial_client_obter_fase();

        uint16_t dist_mm = 0;
        bool presenca = false;
        bool tof_valido = sensor_tof_obter_leitura(&dist_mm, &presenca);
        char wifi_ip[16] = "";
        wifi_status_t wifi_st = wifi_manager_obter_info(wifi_ip, sizeof(wifi_ip), NULL, 0);
        bool mudou_wifi = primeiro_desenho || (wifi_st != ult_wifi_st) || (strcmp(wifi_ip, ult_wifi_ip) != 0);
        uint16_t dist_cm = tof_valido ? (dist_mm / 10) : 0xFFFF;

        bool piscar_alarme = false;
        if (info.estado == ESTADO_ALARME_ARROMBAMENTO) {
            piscar_alarme = (((esp_timer_get_time() / 500000LL) & 1LL) != 0);
        }

#if HUD_STANDBY_ATIVO
        int64_t agora_us = esp_timer_get_time();
        int64_t ult_ref = (s_acordar_manual_us > info.ultima_atividade_us)
                          ? s_acordar_manual_us : info.ultima_atividade_us;
        bool deve_acender = (info.estado != ESTADO_REPOUSO_TRANCADA) ||
                            presenca ||
                            (((agora_us - ult_ref) / 1000000LL) < HUD_STANDBY_TIMEOUT_S);
        if (deve_acender != backlight_ligado) {
            backlight_ligado = deve_acender;
            display_tft_backlight(backlight_ligado);
            ESP_LOGI(TAG, "Backlight TFT -> %s", backlight_ligado ? "LIGADO" : "STANDBY");
        }
#else
        (void)backlight_ligado;
#endif

        bool mudou_banner = primeiro_desenho ||
                            (info.estado != ult_info.estado) ||
                            (info.tempo_restante_s != ult_info.tempo_restante_s) ||
                            (info.porta_esquecida != ult_info.porta_esquecida) ||
                            (presenca != ult_presenca) ||
                            (piscar_alarme != ult_piscar) ||
                            (fase_ia != ult_fase_ia);

        bool mudou_porta = primeiro_desenho ||
                           (info.porta_fechada != ult_info.porta_fechada) ||
                           ((info.estado == ESTADO_ALARME_ARROMBAMENTO) != (ult_info.estado == ESTADO_ALARME_ARROMBAMENTO));

        bool mudou_alarme = primeiro_desenho ||
                            (info.alarme_armado != ult_info.alarme_armado) ||
                            ((info.estado == ESTADO_ALARME_ARROMBAMENTO) != (ult_info.estado == ESTADO_ALARME_ARROMBAMENTO));

        // Atualiza distancia na tela se mudar presenca, validade ou variar >= 3 cm (evita ruido visual)
        int diff_cm = (dist_cm > ult_dist_cm) ? (int)(dist_cm - ult_dist_cm) : (int)(ult_dist_cm - dist_cm);
        bool mudou_tof = primeiro_desenho ||
                         (presenca != ult_presenca) ||
                         (tof_valido != ult_tof_valido) ||
                         (tof_valido && diff_cm >= 3);

        if (mudou_banner) {
            atualizar_topo_e_banner(&info, presenca, piscar_alarme, fase_ia);
            ult_fase_ia = fase_ia;
        }
        if (mudou_porta) {
            atualizar_card_porta(info.porta_fechada, info.estado);
        }
        if (mudou_alarme) {
            atualizar_card_alarme(info.alarme_armado, info.estado);
        }
        if (mudou_tof) {
            atualizar_card_presenca(tof_valido, dist_mm, presenca);
            ult_dist_cm = dist_cm;
            ult_tof_valido = tof_valido;
        }
        if (mudou_wifi) {
            atualizar_barra_wifi(wifi_st, wifi_ip);
            ult_wifi_st = wifi_st;
            strncpy(ult_wifi_ip, wifi_ip, sizeof(ult_wifi_ip) - 1);
            ult_wifi_ip[sizeof(ult_wifi_ip) - 1] = '\0';
        }

        ult_info = info;
        ult_presenca = presenca;
        ult_piscar = piscar_alarme;
        primeiro_desenho = false;

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

esp_err_t hud_tft_iniciar(void) {
    BaseType_t ok = xTaskCreatePinnedToCore(hud_tft_task, "hud_tft_task", 4096, NULL, 3, NULL, 0);
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}
// FIM DO ARQUIVO
