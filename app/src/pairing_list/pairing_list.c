#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "pairing_list.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>


I2C_INSTANCE_STRU eeprom_i2c;

uint8_t eeprom_read_byte(uint8_t addr);
void eeprom_write_byte(uint8_t addr, uint8_t byte);
int32_t eeprom_read_bytes(uint8_t start_addr, uint8_t *buf, uint16_t len);
int32_t eeprom_write_bytes(uint8_t start_addr, uint8_t *buf, uint16_t len);


int32_t pairing_list_addr_write(uint8_t rx_num, uint8_t *addr)
{
    uint8_t eeprom_addr;

    if (rx_num >= PAIRING_LIST_MAX_ADDR_NUM)
    {
        printf("[%s, L%d] rx num(%d) excess max.\r\n", __FILE__, __LINE__, rx_num);
        return -1;
    }
   
    eeprom_addr = PAIRING_LIST_ADDR_LEN * rx_num;
    (void)eeprom_write_bytes(eeprom_addr, addr, PAIRING_LIST_ADDR_LEN);

    return 0;
}

int32_t pairing_list_addr_read(uint8_t rx_num, uint8_t *addr)
{
    uint8_t eeprom_addr;

    if (rx_num >= PAIRING_LIST_MAX_ADDR_NUM)
    {
        printf("[%s, L%d] rx num(%d) excess max.\r\n", __FILE__, __LINE__, rx_num);
        return -1;
    }
      
    eeprom_addr = PAIRING_LIST_ADDR_LEN * rx_num;
    (void)eeprom_read_bytes(eeprom_addr, addr, PAIRING_LIST_ADDR_LEN);

    return 0;    
}

int32_t pairing_list_init(void)
{
    int32_t ret;
    I2C_WIRE_STRU i2c_wire;

    i2c_wire.scl.port = GPIOC;
    i2c_wire.scl.port_num = GPIO_PIN_12;
    
    i2c_wire.sda.port = GPIOC;
    i2c_wire.sda.port_num = GPIO_PIN_11;
    ret = i2c_init(&eeprom_i2c, &i2c_wire);
    if (I2C_RET_OK != ret)
    {
        printf("i2c init failed return %d.\r\n", (int)ret);
        return ret;
    }
    else
    {
        printf("i2c init ok!\r\n");
        return 0;
    }
}


uint8_t eeprom_read_byte(uint8_t addr)
{
	uint8_t  byte_read;
	uint8_t  slave_addr = 0xa0 >> 1;
	uint8_t  txbuf;

	txbuf = addr;
    i2c_master_transmit(&eeprom_i2c, slave_addr, &txbuf, 1, &byte_read, 1);
	return byte_read;
}

void eeprom_write_byte(uint8_t addr, uint8_t byte)
{
	uint8_t  slave_addr = 0xa0 >> 1;
	uint8_t  txbuf[2];

	txbuf[0] = addr;
	txbuf[1] = byte;
    i2c_master_transmit(&eeprom_i2c, slave_addr, &txbuf, 2, NULL, 0);
	return;    
}

int32_t eeprom_read_bytes(uint8_t start_addr, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        buf[i] = eeprom_read_byte(start_addr + i);
    }

    return 0;
}

int32_t eeprom_write_bytes(uint8_t start_addr, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        eeprom_write_byte(start_addr + i, buf[i]);
    }

    return 0;
}

void pairing_list_entry_show(uint8_t idx)
{
    uint8_t addr[PAIRING_LIST_ADDR_LEN];
    (void)pairing_list_addr_read(idx, addr);

    printf("pairing list idx %d: %02x%02x%02x%02x \r\n",
        idx, addr[0], addr[1], addr[2], addr[3]);
    return;
}

