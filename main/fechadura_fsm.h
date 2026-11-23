#ifndef FECHADURA_FSM_H
#define FECHADURA_FSM_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    ESTADO_REPOUSO_TRANCADA = 0,    // Porta fechada e trancada (alarme ARMADO)
    ESTADO_DIGITANDO_SENHA,         // Recebendo digitos do teclado 4x4 (alarme ARMADO)
    ESTADO_RECONHECIMENTO_FACIAL,
    ESTADO_DESTRANCADA_AUTORIZADA,  // Solenoide ativa por 5s (alarme DESARMADO)
    ESTADO_PORTA_ABERTA,            // MC-38 aberto apos autorizacao (alarme DESARMADO)
    ESTADO_REARMANDO_ALARME,        // Porta fechou: aguarda 3s estabilizar antes de armar MPU
    ESTADO_BLOQUEIO_TENTATIVAS,     // 3 senhas erradas: teclado travado por 30s (alarme ARMADO)
    ESTADO_ALARME_ARROMBAMENTO      // Violacao de carcaca (MPU) ou abertura forcada (MC-38)
} fechadura_estado_t;

typedef enum {
    EVT_TECLA_PRESSIONADA = 0,      // Tecla do teclado 4x4 (usa campo 'tecla')
    EVT_BOTAO_SAIDA,                // Botao fisico interno pressionado
    EVT_PORTA_ABRIU,                // Sensor MC-38 mudou para ABERTA
    EVT_PORTA_FECHOU,               // Sensor MC-38 mudou para FECHADA
    EVT_PRESENCA_DETECTADA,         // Sensor VL53L1X detectou pessoa proxima
    EVT_MPU_VIOLACAO,               // Impacto forte ou inclinacao da caixa > 18 graus
    EVT_ACESSO_REMOTO_OK,           // Gancho futuro: reconhecimento facial / comando da VPS
    EVT_FACIAL_NEGADO,
    EVT_FACIAL_ERRO
} fechadura_evento_tipo_t;

typedef struct {
    fechadura_estado_t estado;
    bool alarme_armado;
    bool porta_fechada;
    bool porta_esquecida;
    uint32_t tempo_restante_s;
    int64_t ultima_atividade_us;
    char usuario_autorizado[32];
} fechadura_hud_info_t;

// Retorna um snapshot thread-safe do estado da fechadura para o display TFT
void fechadura_fsm_obter_hud_info(fechadura_hud_info_t *info);

// Inicializa a fila e a task principal da maquina de estados
esp_err_t fechadura_fsm_iniciar(void);

// Envia um evento para a maquina de estados (thread-safe, nao bloqueante)
void fechadura_fsm_enviar_evento(fechadura_evento_tipo_t tipo, char tecla);

// Indica para o sensor_mpu.c se o monitoramento anti-arrombamento esta armado
bool fechadura_fsm_alarme_armado(void);

// Retorna o estado atual (util para o display TFT e telemetria)
fechadura_estado_t fechadura_fsm_obter_estado(void);

// define o nome exibido no HUD (para funcionar tanto para reconhecimento facial quanto para abertura remota e PIN):
void fechadura_fsm_definir_usuario_autorizado(const char *nome);

#endif // FECHADURA_FSM_H
