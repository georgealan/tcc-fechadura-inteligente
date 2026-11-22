#include "fechadura_fsm.h"
#include "porta.h"
#include "audio_i2s.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "facial_client.h"

static const char *TAG = "FECHADURA_FSM";

// Parametros operacionais da fechadura
#define SENHA_MESTRA                "1234"
#define TAMANHO_MAX_SENHA           8
#define MAX_TENTATIVAS_ERRADAS      3
#define TEMPO_DESTRANCADA_MS        5000    // Solenoide aberta por 5s
#define TIMEOUT_DIGITACAO_MS        10000   // Limpa senha se ficar 10s sem digitar
#define TEMPO_ESTABILIZACAO_MPU_MS  3000    // Aguarda 3s apos fechar porta para rearmar MPU
#define TEMPO_BLOQUEIO_TECLADO_MS   30000   // Bloqueia teclado por 30s apos 3 erros
#define TEMPO_PORTA_ESQUECIDA_MS    30000   // Aviso se porta ficar aberta por 30s
#define INTERVALO_SIRENE_MS         3000    // Repete som de alarme a cada 3s enquanto violada

typedef struct {
    fechadura_evento_tipo_t tipo;
    char tecla;
} fsm_msg_t;

static QueueHandle_t s_fsm_queue = NULL;
static portMUX_TYPE s_fsm_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile fechadura_estado_t s_estado = ESTADO_REPOUSO_TRANCADA;
static volatile bool s_alarme_armado = true;

// Variaveis internas da task FSM (leitura externa sempre protegida por s_fsm_mux)
static char s_buffer_senha[TAMANHO_MAX_SENHA + 1] = {0};
static size_t s_len_senha = 0;
static uint8_t s_erros_consecutivos = 0;
static int64_t s_timestamp_estado_us = 0;
static int64_t s_ultima_atividade_us = 0;
static int64_t s_ultima_tecla_us = 0;
static int64_t s_ultima_sirene_us = 0;
static bool s_aviso_porta_aberta_enviado = false;
static char s_usuario_autorizado[32] = "";

static const char *nome_estado(fechadura_estado_t est) {
    switch (est) {
        case ESTADO_REPOUSO_TRANCADA:       return "REPOUSO_TRANCADA";
        case ESTADO_DIGITANDO_SENHA:        return "DIGITANDO_SENHA";
        case ESTADO_DESTRANCADA_AUTORIZADA: return "DESTRANCADA_AUTORIZADA";
        case ESTADO_PORTA_ABERTA:           return "PORTA_ABERTA";
        case ESTADO_REARMANDO_ALARME:       return "REARMANDO_ALARME";
        case ESTADO_BLOQUEIO_TENTATIVAS:    return "BLOQUEIO_TENTATIVAS";
        case ESTADO_ALARME_ARROMBAMENTO:    return "ALARME_ARROMBAMENTO";
        case ESTADO_RECONHECIMENTO_FACIAL:  return "RECONHECIMENTO_FACIAL";
        default:                            return "DESCONHECIDO";
    }
}

static void limpar_buffer_senha(void) {
    memset(s_buffer_senha, 0, sizeof(s_buffer_senha));
    s_len_senha = 0;
}

static void mudar_estado(fechadura_estado_t novo_estado) {
    if (s_estado == novo_estado) return;

    ESP_LOGI(TAG, "Transicao: %s -> %s", nome_estado(s_estado), nome_estado(novo_estado));
    int64_t agora_us = esp_timer_get_time();

    // Alarme MPU-6050 so fica armado quando a porta esta fechada e sem abertura autorizada
    bool armado = (novo_estado == ESTADO_REPOUSO_TRANCADA ||
                   novo_estado == ESTADO_DIGITANDO_SENHA  ||
                   novo_estado == ESTADO_RECONHECIMENTO_FACIAL ||
                   novo_estado == ESTADO_BLOQUEIO_TENTATIVAS);

    portENTER_CRITICAL(&s_fsm_mux);
    s_estado = novo_estado;
    s_alarme_armado = armado;
    s_timestamp_estado_us = agora_us;
    s_ultima_atividade_us = agora_us;
    if (novo_estado == ESTADO_REPOUSO_TRANCADA) {
        s_usuario_autorizado[0] = '\0';
    }
    if (novo_estado == ESTADO_PORTA_ABERTA) {
        s_aviso_porta_aberta_enviado = false;
    }
    portEXIT_CRITICAL(&s_fsm_mux);
}

static void liberar_acesso(const char *origem) {
    ESP_LOGI(TAG, ">>> ACESSO LIBERADO (%s)!", origem);
    limpar_buffer_senha();
    s_erros_consecutivos = 0;
    mudar_estado(ESTADO_DESTRANCADA_AUTORIZADA);
    porta_destrancar(TEMPO_DESTRANCADA_MS);
}

static void disparar_alarme(const char *motivo) {
    if (s_estado == ESTADO_ALARME_ARROMBAMENTO) return;

    ESP_LOGE(TAG, "========================================================");
    ESP_LOGE(TAG, "!!! ALARME DE ARROMBAMENTO DISPARADO: %s !!!", motivo);
    ESP_LOGE(TAG, "    (Digite a senha + '#' ou aperte o botao interno)");
    ESP_LOGE(TAG, "========================================================");
    // Futuro: disparar evento MQTT/HTTP para a VPS com foto da camera UVC

    limpar_buffer_senha();
    mudar_estado(ESTADO_ALARME_ARROMBAMENTO);
    s_ultima_sirene_us = esp_timer_get_time();
    audio_i2s_tocar(AUDIO_SOM_ALARME);
}

static void processar_tecla(char tecla) {
    if (s_estado == ESTADO_REPOUSO_TRANCADA || s_estado == ESTADO_RECONHECIMENTO_FACIAL) {
        mudar_estado(ESTADO_DIGITANDO_SENHA);
    }

    if (s_estado == ESTADO_BLOQUEIO_TENTATIVAS) {
        int64_t decorrido_ms = (esp_timer_get_time() - s_timestamp_estado_us) / 1000;
        int64_t restante_s = (TEMPO_BLOQUEIO_TECLADO_MS - decorrido_ms) / 1000;
        if (restante_s < 1) restante_s = 1;
        ESP_LOGW(TAG, "Teclado bloqueado por excesso de erros. Aguarde %lld s.", (long long)restante_s);
        audio_i2s_tocar(AUDIO_SOM_ACESSO_NEGADO);
        return;
    }

    // Ignora digitacao enquanto a porta ja esta destrancada ou aberta
    if (s_estado == ESTADO_DESTRANCADA_AUTORIZADA ||
        s_estado == ESTADO_PORTA_ABERTA ||
        s_estado == ESTADO_REARMANDO_ALARME) {
        return;
    }

    s_ultima_tecla_us = esp_timer_get_time();

    if (tecla == '*') {
        limpar_buffer_senha();
        ESP_LOGI(TAG, "Digitacao cancelada ('*').");
        if (s_estado == ESTADO_DIGITANDO_SENHA) {
            mudar_estado(ESTADO_REPOUSO_TRANCADA);
        }
        return;
    }

    if (tecla == '#') {
        if (s_len_senha == 0) return;

        if (strcmp(s_buffer_senha, SENHA_MESTRA) == 0) {
            if (s_estado == ESTADO_ALARME_ARROMBAMENTO) {
                ESP_LOGI(TAG, ">>> ALARME DESARMADO POR SENHA VALIDA.");
                limpar_buffer_senha();
                s_erros_consecutivos = 0;
                if (s_estado == ESTADO_ALARME_ARROMBAMENTO) {
                ESP_LOGI(TAG, ">>> ALARME DESARMADO POR SENHA VALIDA.");
                limpar_buffer_senha();
                s_erros_consecutivos = 0;
                audio_i2s_tocar(AUDIO_SOM_ACESSO_OK);

                if (porta_esta_fechada()) {
                    mudar_estado(ESTADO_REARMANDO_ALARME);
                } else {
                    ESP_LOGW(TAG, "Alarme desarmado, mas folha da porta segue ABERTA (MC-38).");
                    mudar_estado(ESTADO_PORTA_ABERTA);
                }
            } else {
                liberar_acesso("Senha Teclado 4x4");
            }
                audio_i2s_tocar(AUDIO_SOM_ACESSO_OK);
            } else {
                liberar_acesso("Senha Teclado 4x4");
            }
        } else {
            s_erros_consecutivos++;
            ESP_LOGW(TAG, ">>> SENHA INCORRETA! Tentativa %u de %u.",
                     s_erros_consecutivos, MAX_TENTATIVAS_ERRADAS);
            limpar_buffer_senha();
            audio_i2s_tocar(AUDIO_SOM_ACESSO_NEGADO);

            if (s_estado != ESTADO_ALARME_ARROMBAMENTO) {
                if (s_erros_consecutivos >= MAX_TENTATIVAS_ERRADAS) {
                    ESP_LOGE(TAG, ">>> LIMITE DE TENTATIVAS ATINGIDO! Teclado bloqueado por %d s.",
                             TEMPO_BLOQUEIO_TECLADO_MS / 1000);
                    mudar_estado(ESTADO_BLOQUEIO_TENTATIVAS);
                } else {
                    mudar_estado(ESTADO_REPOUSO_TRANCADA);
                }
            }
        }
        return;
    }

    // Aceita apenas digitos numericos '0' a '9' na senha
    if (tecla >= '0' && tecla <= '9') {
        if (s_estado == ESTADO_REPOUSO_TRANCADA) {
            mudar_estado(ESTADO_DIGITANDO_SENHA);
        }
        if (s_len_senha < TAMANHO_MAX_SENHA) {
            s_buffer_senha[s_len_senha++] = tecla;
            s_buffer_senha[s_len_senha] = '\0';

            char mascara[TAMANHO_MAX_SENHA + 1];
            memset(mascara, '*', s_len_senha);
            mascara[s_len_senha] = '\0';
            ESP_LOGI(TAG, "Senha em digitacao: [ %s ] (%u digitos)", mascara, (unsigned)s_len_senha);
        }
    }
}

static void processar_evento(const fsm_msg_t *msg) {
    portENTER_CRITICAL(&s_fsm_mux);
    s_ultima_atividade_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_fsm_mux);

    switch (msg->tipo) {
        case EVT_TECLA_PRESSIONADA:
            processar_tecla(msg->tecla);
            break;

        case EVT_BOTAO_SAIDA:
            // Regra de seguranca: botao interno SEMPRE abre a porta (e silencia alarme)
            liberar_acesso("Botao de Saida Interno");
            break;

        case EVT_ACESSO_REMOTO_OK: {
            char nome_ia[32] = "";
            facial_client_obter_ultimo_nome(nome_ia, sizeof(nome_ia));
            portENTER_CRITICAL(&s_fsm_mux);
            strncpy(s_usuario_autorizado, nome_ia, sizeof(s_usuario_autorizado) - 1);
            s_usuario_autorizado[sizeof(s_usuario_autorizado) - 1] = '\0';
            portEXIT_CRITICAL(&s_fsm_mux);
            liberar_acesso("Reconhecimento Facial / IA");
            break;
        }

        case EVT_PORTA_ABRIU:
            if (s_estado == ESTADO_DESTRANCADA_AUTORIZADA || s_estado == ESTADO_REARMANDO_ALARME) {
                ESP_LOGI(TAG, "Porta aberta fisicamente (MC-38). Monitoramento MPU pausado.");
                mudar_estado(ESTADO_PORTA_ABERTA);
            } else if (s_alarme_armado) {
                disparar_alarme("Porta aberta a forca sem autorizacao (MC-38)");
            }
            break;

        case EVT_PORTA_FECHOU:
            if (s_estado == ESTADO_PORTA_ABERTA || s_estado == ESTADO_DESTRANCADA_AUTORIZADA) {
                ESP_LOGI(TAG, "Porta fechada (MC-38). Aguardando %d s para estabilizar e rearmar MPU...",
                         TEMPO_ESTABILIZACAO_MPU_MS / 1000);
                mudar_estado(ESTADO_REARMANDO_ALARME);
            }
            break;

        case EVT_MPU_VIOLACAO:
            if (s_alarme_armado) {
                disparar_alarme("Violacao fisica / inclinacao da carcaca (MPU-6050)");
            }
            break;

        case EVT_PRESENCA_DETECTADA:
            if (s_estado == ESTADO_REPOUSO_TRANCADA) {
                if (facial_client_solicitar_ciclo()) {
                    ESP_LOGI(TAG, "Visitante detectado (VL53L1X) -> Iniciando Reconhecimento Facial...");
                    mudar_estado(ESTADO_RECONHECIMENTO_FACIAL);
                }
            }
            break;

        case EVT_FACIAL_NEGADO:
            audio_i2s_tocar(AUDIO_SOM_ACESSO_NEGADO);
            if (s_estado == ESTADO_RECONHECIMENTO_FACIAL) {
                mudar_estado(ESTADO_REPOUSO_TRANCADA);
            }
            break;

        case EVT_FACIAL_ERRO:
            if (s_estado == ESTADO_RECONHECIMENTO_FACIAL) {
                mudar_estado(ESTADO_REPOUSO_TRANCADA);
            }
            break;

        default:
            break;
    }
}

static void verificar_temporizadores(void) {
    int64_t agora_us = esp_timer_get_time();
    int64_t tempo_no_estado_ms = (agora_us - s_timestamp_estado_us) / 1000;

    switch (s_estado) {
        case ESTADO_DIGITANDO_SENHA:
            if (((agora_us - s_ultima_tecla_us) / 1000) >= TIMEOUT_DIGITACAO_MS) {
                ESP_LOGI(TAG, "Timeout de digitacao. Buffer de senha limpo.");
                limpar_buffer_senha();
                mudar_estado(ESTADO_REPOUSO_TRANCADA);
            }
            break;

        case ESTADO_DESTRANCADA_AUTORIZADA:
            if (tempo_no_estado_ms >= TEMPO_DESTRANCADA_MS) {
                if (porta_esta_fechada()) {
                    ESP_LOGI(TAG, "Tempo de abertura encerrado sem abrir a porta. Rearmando...");
                    mudar_estado(ESTADO_REARMANDO_ALARME);
                } else {
                    ESP_LOGI(TAG, "Solenoide desligada, mas porta segue ABERTA (MC-38).");
                    mudar_estado(ESTADO_PORTA_ABERTA);
                }
            }
            break;

        case ESTADO_PORTA_ABERTA:
            if (!s_aviso_porta_aberta_enviado && tempo_no_estado_ms >= TEMPO_PORTA_ESQUECIDA_MS) {
                portENTER_CRITICAL(&s_fsm_mux);
                s_aviso_porta_aberta_enviado = true;
                s_ultima_atividade_us = agora_us;
                portEXIT_CRITICAL(&s_fsm_mux);

                ESP_LOGW(TAG, ">>> ATENCAO: Porta aberta ha mais de %d segundos!",
                         TEMPO_PORTA_ESQUECIDA_MS / 1000);
                audio_i2s_tocar(AUDIO_SOM_CAMPAINHA);
            }
            break;

        case ESTADO_REARMANDO_ALARME:
            if (!porta_esta_fechada()) {
                ESP_LOGW(TAG, "Rearme cancelado: porta encontra-se ABERTA (MC-38).");
                mudar_estado(ESTADO_PORTA_ABERTA);
            } else if (tempo_no_estado_ms >= TEMPO_ESTABILIZACAO_MPU_MS) {
                ESP_LOGI(TAG, ">>> SISTEMA ARMADO: Porta trancada e MPU-6050 monitorando.");
                mudar_estado(ESTADO_REPOUSO_TRANCADA);
            }
            break;

        case ESTADO_BLOQUEIO_TENTATIVAS:
            if (tempo_no_estado_ms >= TEMPO_BLOQUEIO_TECLADO_MS) {
                ESP_LOGI(TAG, "Bloqueio de teclado encerrado.");
                s_erros_consecutivos = 0;
                mudar_estado(ESTADO_REPOUSO_TRANCADA);
            }
            break;

        case ESTADO_ALARME_ARROMBAMENTO:
            if (((agora_us - s_ultima_sirene_us) / 1000) >= INTERVALO_SIRENE_MS) {
                s_ultima_sirene_us = agora_us;
                ESP_LOGW(TAG, "!!! ALARME ATIVO — aguardando senha valida ou botao interno !!!");
                audio_i2s_tocar(AUDIO_SOM_ALARME);
            }
            break;

        default:
            break;
    }
}

static void fsm_task(void *arg) {
    (void)arg;
    fsm_msg_t msg;

    int64_t agora_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_fsm_mux);
    s_timestamp_estado_us = agora_us;
    s_ultima_atividade_us = agora_us;
    portEXIT_CRITICAL(&s_fsm_mux);

    ESP_LOGI(TAG, "Maquina de Estados iniciada. Estado: %s | Senha padrao: %s",
             nome_estado(s_estado), SENHA_MESTRA);

    while (1) {
        if (xQueueReceive(s_fsm_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            processar_evento(&msg);
        }
        verificar_temporizadores();
    }
}

void fechadura_fsm_enviar_evento(fechadura_evento_tipo_t tipo, char tecla) {
    if (s_fsm_queue == NULL) return;
    fsm_msg_t msg = { .tipo = tipo, .tecla = tecla };
    xQueueSend(s_fsm_queue, &msg, 0);
}

bool fechadura_fsm_alarme_armado(void) {
    return s_alarme_armado;
}

fechadura_estado_t fechadura_fsm_obter_estado(void) {
    return s_estado;
}

void fechadura_fsm_obter_hud_info(fechadura_hud_info_t *info) {
    if (info == NULL) return;

    int64_t agora_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_fsm_mux);
    fechadura_estado_t est = s_estado;
    bool armado = s_alarme_armado;
    bool esquecida = s_aviso_porta_aberta_enviado;
    int64_t ts_estado = s_timestamp_estado_us;
    int64_t ult_ativ = s_ultima_atividade_us;
    strncpy(info->usuario_autorizado, s_usuario_autorizado, sizeof(info->usuario_autorizado) - 1);
    info->usuario_autorizado[sizeof(info->usuario_autorizado) - 1] = '\0';
    portEXIT_CRITICAL(&s_fsm_mux);

    uint32_t restante_s = 0;
    int64_t decorrido_ms = (agora_us - ts_estado) / 1000;
    int64_t limite_ms = 0;

    if (est == ESTADO_DESTRANCADA_AUTORIZADA) {
        limite_ms = TEMPO_DESTRANCADA_MS;
    } else if (est == ESTADO_REARMANDO_ALARME) {
        limite_ms = TEMPO_ESTABILIZACAO_MPU_MS;
    } else if (est == ESTADO_BLOQUEIO_TENTATIVAS) {
        limite_ms = TEMPO_BLOQUEIO_TECLADO_MS;
    }

    if (limite_ms > 0 && decorrido_ms < limite_ms) {
        restante_s = (uint32_t)((limite_ms - decorrido_ms + 999) / 1000);
    }

    info->estado = est;
    info->alarme_armado = armado;
    info->porta_fechada = porta_esta_fechada();
    info->porta_esquecida = esquecida;
    info->tempo_restante_s = restante_s;
    info->ultima_atividade_us = ult_ativ;
}

esp_err_t fechadura_fsm_iniciar(void) {
    s_fsm_queue = xQueueCreate(16, sizeof(fsm_msg_t));
    if (s_fsm_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    xTaskCreatePinnedToCore(fsm_task, "fsm_task", 4096, NULL, 6, NULL, 0);
    return ESP_OK;
}
// FIM DO ARQUIVO
