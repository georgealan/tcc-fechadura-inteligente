#ifndef SENSOR_TOF_H
#define SENSOR_TOF_H

#include <stdint.h>
#include <stdbool.h>

void sensor_tof_iniciar(void);

// Retorna true se a ultima leitura do VL53L1X for valida; preenche distancia (mm) e flag de presenca
bool sensor_tof_obter_leitura(uint16_t *dist_mm, bool *presenca_ativa);

#endif // SENSOR_TOF_H
// FIM DO ARQUIVO
