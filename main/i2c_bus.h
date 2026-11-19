#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c_master.h"
#include "esp_err.h"

// Inicializa o barramento I2C mestre (GPIO 4 SDA / GPIO 5 SCL)
esp_err_t i2c_bus_iniciar(void);

// Retorna o handle compartilhado para registrar sensores (VL53L1X, MPU-6050, etc.)
i2c_master_bus_handle_t i2c_bus_obter_handle(void);

#endif // I2C_BUS_H
