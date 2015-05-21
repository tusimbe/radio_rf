#ifndef __EEPROM_H__
#define __EEPROM_H__

#include "i2c.h"

uint8_t eeprom_read_byte
(
    I2C_INSTANCE_STRU *i2c,
    uint8_t addr
);

void eeprom_write_byte
(
    I2C_INSTANCE_STRU *i2c,
    uint8_t addr, uint8_t byte
);

int32_t eeprom_read_bytes
(
    I2C_INSTANCE_STRU *i2c,
    uint8_t start_addr, 
    uint8_t *buf, 
    uint16_t len
);

int32_t eeprom_write_bytes
(
    I2C_INSTANCE_STRU *i2c,
    uint8_t start_addr, 
    uint8_t *buf, 
    uint16_t len
);


#endif
