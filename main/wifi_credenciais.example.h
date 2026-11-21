#ifndef WIFI_CREDENCIAIS_H
#define WIFI_CREDENCIAIS_H

typedef struct {
    const char *ssid;
    const char *senha;
} wifi_rede_conhecida_t;

// Ordem livre: o ESP32-S3 escolhe automaticamente a rede conhecida visivel com melhor sinal (2,4 GHz).
// Slots com ssid vazio ("") sao ignorados automaticamente.
static const wifi_rede_conhecida_t WIFI_REDES_CONHECIDAS[] = {
    { "NOME_DO_SEU_WIFI_24GHZ", "SENHA_DO_SEU_WIFI" },  // 1. Roteador de casa
    { "", "" },                                         // 2. Hotspot do celular (preencher depois)
};

#define WIFI_QTD_REDES_CONHECIDAS (sizeof(WIFI_REDES_CONHECIDAS) / sizeof(WIFI_REDES_CONHECIDAS[0]))

#endif // WIFI_CREDENCIAIS_H
// FIM DO ARQUIVO
