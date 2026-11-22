#ifndef FACIAL_CLIENT_H
#define FACIAL_CLIENT_H

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef enum {
    FACIAL_FASE_OCIOSO = 0,
    FACIAL_FASE_POSICIONANDO,   // Aguardando pessoa olhar para a camera (1,5s)
    FACIAL_FASE_ENVIANDO,       // Enviando foto e aguardando resposta da IA + Kokoro
    FACIAL_FASE_NEGADO,         // Rosto desconhecido (exibe aviso breve)
    FACIAL_FASE_SEM_ROSTO,      // Nenhum rosto detectado na foto
    FACIAL_FASE_ERRO_REDE       // Wi-Fi offline ou servidor indisponivel
} facial_fase_t;

// Aloca buffers na PSRAM e cria a task de reconhecimento facial
esp_err_t facial_client_iniciar(void);

// Solicita inicio de um ciclo de reconhecimento (retorna false se ocupado ou em cooldown)
bool facial_client_solicitar_ciclo(void);

// Consulta thread-safe da fase atual e do ultimo nome reconhecido (ASCII limpo para o TFT)
facial_fase_t facial_client_obter_fase(void);
void facial_client_obter_ultimo_nome(char *buf, size_t len);

#endif // FACIAL_CLIENT_H
// FIM DO ARQUIVO
