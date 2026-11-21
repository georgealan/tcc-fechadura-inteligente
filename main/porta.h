#ifndef PORTA_H
#define PORTA_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define PORTA_TEMPO_ABERTURA_PADRAO_MS  5000

// Inicializa GPIOs (rele, botao de saida e MC-38) e inicia a task de monitoramento
esp_err_t porta_iniciar(void);

// Aciona o rele para destrancar a solenoide por 'duracao_ms' (tranca automaticamente ao terminar)
void porta_destrancar(uint32_t duracao_ms);

// Desliga o rele imediatamente (trava a solenoide)
void porta_trancar(void);

// Retorna true se a solenoide estiver energizada (destrancada)
bool porta_esta_destrancada(void);

// Retorna true se o sensor magnetico MC-38 indicar folha da porta fechada
bool porta_esta_fechada(void);

#endif // PORTA_H
