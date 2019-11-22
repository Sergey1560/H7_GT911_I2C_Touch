#ifndef I2C_H
#define I2C_H
#include "common_defs.h"


void i2c_gpio_init(void);
void i2c_init(void);

uint8_t i2c_write_cmd(uint8_t adr,uint16_t cmd);
uint8_t i2c_read(uint8_t adr,uint8_t *data,uint8_t len);
uint8_t i2c_write(uint8_t adr,uint8_t *data,uint8_t len);
uint8_t i2c_write_reg(uint8_t adr,uint16_t reg,uint8_t *data,uint8_t len);

#endif