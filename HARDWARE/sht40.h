#ifndef _SHT30_H_
#define _SHT30_H_

#include "stdint.h"

uint8_t sht40_init();
uint8_t I2C_SHT40_read(uint16_t reg);
uint8_t I2C_SHT40_read_nodelay(uint16_t reg);
#endif