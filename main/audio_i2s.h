#ifndef AUDIO_I2S_H
#define AUDIO_I2S_H

#include <stdint.h>
#include "esp_err.h"

typedef enum {
    AUDIO_SOM_BOOT = 0,       // Melodia curta de inicializacao
    AUDIO_SOM_TECLA,          // Bipe rapido de tecla do teclado 4x4
    AUDIO_SOM_ACESSO_OK,      // Confirmacao positiva (porta destrancada)
    AUDIO_SOM_ACESSO_NEGADO,  // Erro (senha incorreta / acesso negado)
    AUDIO_SOM_CAMPAINHA,      // Din-don suave (presenca / visitante)
    AUDIO_SOM_ALARME          // Sirene curta (violacao / arrombamento)
} audio_efeito_t;

// Inicializa o barramento I2S (MAX98357A) e a task de reproducao
esp_err_t audio_i2s_iniciar(void);

// Enfileira um efeito sonoro para tocar sem bloquear quem chamou
void audio_i2s_tocar(audio_efeito_t efeito);

// Ajusta o volume de software (0 a 100%)
void audio_i2s_definir_volume(uint8_t volume_pct);

#endif // AUDIO_I2S_H
