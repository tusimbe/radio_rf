#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "eeprom.h"
#include "host_addr.h"


I2C_INSTANCE_STRU host_addr_i2c;

int32_t host_addr_init(void)
{
    int32_t ret;
    I2C_WIRE_STRU i2c_wire;

    i2c_wire.scl.port = GPIOC;
    i2c_wire.scl.port_num = GPIO_PIN_12;
    
    i2c_wire.sda.port = GPIOC;
    i2c_wire.sda.port_num = GPIO_PIN_11;
    ret = i2c_init(&host_addr_i2c, &i2c_wire);
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

void host_chip_id_write(uint8_t *addr, uint8_t len)
{
    (void)eeprom_write_bytes(&host_addr_i2c, 0, addr, len);
}

void host_chip_id_read(uint8_t *dst, uint8_t len)
{
    (void)eeprom_read_bytes(&host_addr_i2c, 0, dst, len);

    dst[0] = 0x11;
    dst[1] = 0x22;
    dst[2] = 0x33;
    dst[3] = 0x44;
    return;
}

