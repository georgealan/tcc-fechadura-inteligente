#ifndef PINAGEM_H
#define PINAGEM_H

#include "driver/gpio.h"

// --- Teclado Matricial 4x4 (Lado Direito) ---
// Pinos 1 a 4 do conector = Colunas (C1 a C4)
#define PIN_TECLADO_C1      GPIO_NUM_1
#define PIN_TECLADO_C2      GPIO_NUM_2
#define PIN_TECLADO_C3      GPIO_NUM_42
#define PIN_TECLADO_C4      GPIO_NUM_41

// Pinos 5 a 8 do conector = Linhas (L1 a L4)
#define PIN_TECLADO_L1      GPIO_NUM_40
#define PIN_TECLADO_L2      GPIO_NUM_39
#define PIN_TECLADO_L3      GPIO_NUM_38
#define PIN_TECLADO_L4      GPIO_NUM_47

// --- Barramento I2C Compartilhado (VL53L1X + MPU-6050) ---
#define PIN_I2C_SDA         GPIO_NUM_4
#define PIN_I2C_SCL         GPIO_NUM_5

// --- Controle da Porta (Relé, Botão de Saída e Sensor Magnético MC-38) ---
#define PIN_BOTAO_SAIDA     GPIO_NUM_6
#define PIN_RELE_SOLENOIDE  GPIO_NUM_7
#define PIN_SENSOR_PORTA    GPIO_NUM_18

// --- Áudio I2S (MAX98357A) ---
#define PIN_I2S_LRC         GPIO_NUM_17
#define PIN_I2S_BCLK        GPIO_NUM_16
#define PIN_I2S_DIN         GPIO_NUM_15

// --- Display TFT SPI (ST7796S 3.5" 320x480 IPS) ---
#define PIN_TFT_DC          GPIO_NUM_8
#define PIN_TFT_RST         GPIO_NUM_9
#define PIN_TFT_BL          GPIO_NUM_10
#define PIN_TFT_CS          GPIO_NUM_11
#define PIN_TFT_MOSI        GPIO_NUM_12
#define PIN_TFT_SCK         GPIO_NUM_14

#endif // PINAGEM_H
