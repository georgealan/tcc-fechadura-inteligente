#ifndef HUD_TFT_H
#define HUD_TFT_H

#include "esp_err.h"

// Inicializa a task responsavel pelo HUD da caixa interna no display TFT
esp_err_t hud_tft_iniciar(void);

// Acorda a tela manualmente caso HUD_STANDBY_ATIVO esteja habilitado
void hud_tft_acordar(void);

#endif // HUD_TFT_H
// FIM DO ARQUIVO
