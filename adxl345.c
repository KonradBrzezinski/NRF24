#include "adxl345.h"

void adxl345_write_reg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(I2C_PORT, ADXL345_I2C_ADDR, buf, 2, false);
}

void adxl345_read_multi(uint8_t reg, uint8_t *buf, size_t len) {
    i2c_write_blocking(I2C_PORT, ADXL345_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ADXL345_I2C_ADDR, buf, len, false);
}

void ADXL345_init(void) {
    // Inicjalizacja I2C
    i2c_init(I2C_PORT, 400 * 1000); // 400 kHz

    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Konfiguracja ADXL345:
    
    // 1. Ustawienie Data Rate na 3200 Hz (0x0F)
    adxl345_write_reg(ADXL345_BW_RATE, 0b00001111);
    // 2. Ustawienie formatu: Pełna rozdzielczość, +/- 16g (0x0B)
    adxl345_write_reg(ADXL345_DATA_FORMAT, 0b00001011);
    // 3. Uruchomienie trybu pomiaru (0x08)
    adxl345_write_reg(ADXL345_POWER_CTL, 0x08);

    printf("ADXL345 zainicjalizowany.\n");
}

int16_t ADXL345_read_X_g(void) {
    uint8_t buffer[2];
    int16_t raw_x;
    
    // Odczyt X-axis: DATAX0 (0x32) i DATAX1 (0x33)
    adxl345_read_multi(ADXL345_DATAX0, buffer, 2);
    
    // Łączenie bajtów LSB (buffer[0]) i MSB (buffer[1])
    raw_x = (int16_t)(buffer[0] | (buffer[1] << 8));
    
    return raw_x;
}
