#include "hardware/i2c.h" // NOWA BIBLIOTEKA DLA I2C
#include <stdio.h>
#include "pico/stdlib.h"

// --- DEFINICJE DLA ADXL345 ---
#define ADXL345_I2C_ADDR 0x53 // Domyślny adres I2C dla ADXL345
#define ADXL345_POWER_CTL 0x2D // Rejestr włączania pomiaru
#define ADXL345_DATA_FORMAT 0x31 // Rejestr formatu danych (zakres)
#define ADXL345_DATAX0 0x32 // Początek rejestrów danych X
#define ADXL345_BW_RATE 0x2C // Rejestr częstotliwości próbkowania

// Piny I2C dla ADXL345
#define I2C_PORT i2c0
#define I2C_SDA_PIN 12  // Możesz zmienić na inne wolne piny
#define I2C_SCL_PIN 13  // np. GP4/GP5

void adxl345_write_reg(uint8_t reg, uint8_t data);
void adxl345_read_multi(uint8_t reg, uint8_t *buf, size_t len);
void ADXL345_init(void);
int16_t ADXL345_read_X_g(void);