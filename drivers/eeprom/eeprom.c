#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "eeprom.h"
#include <stdio.h>
#include <string.h>


uint8_t eeprom_read_byte
(
    I2C_INSTANCE_STRU *i2c,
    uint8_t addr
)
{
	uint8_t  byte_read;
	uint8_t  slave_addr = 0xa0 >> 1;
	uint8_t  txbuf;

	txbuf = addr;
    i2c_master_transmit(i2c, slave_addr, &txbuf, 1, &byte_read, 1);
	return byte_read;
}

void eeprom_write_byte
(
    I2C_INSTANCE_STRU *i2c,
    uint8_t addr, 
    uint8_t byte
)
{
	uint8_t  slave_addr = 0xa0 >> 1;
	uint8_t  txbuf[2];

	txbuf[0] = addr;
	txbuf[1] = byte;
    i2c_master_transmit(i2c, slave_addr, &txbuf, 2, NULL, 0);
    osDelay(2);
	return;
}

int32_t eeprom_read_bytes
(
    I2C_INSTANCE_STRU *i2c,
    uint8_t start_addr, 
    uint8_t *buf, 
    uint16_t len
)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        buf[i] = eeprom_read_byte(i2c, start_addr + i);
    }

    return 0;
}

int32_t eeprom_write_bytes
(
    I2C_INSTANCE_STRU *i2c,
    uint8_t start_addr, 
    uint8_t *buf, 
    uint16_t len
)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        eeprom_write_byte(i2c, start_addr + i, buf[i]);
    }

    return 0;
}

