#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

// Inicializa o barramento I2C mestre (GPIO 4 SDA / GPIO 5 SCL) e faz scanner inicial
esp_err_t i2c_bus_iniciar(void);

// Executa reset de barramento (9 pulsos de clock no SCL) para destravar sensores apos timeout
esp_err_t i2c_bus_recuperar(void);

// Retorna o handle compartilhado para registrar sensores
i2c_master_bus_handle_t i2c_bus_obter_handle(void);

// Protecao de concorrencia (Mutex) entre tarefas que dividem o mesmo barramento I2C
bool i2c_bus_lock(uint32_t timeout_ms);
void i2c_bus_unlock(void);

#endif // I2C_BUS_H
